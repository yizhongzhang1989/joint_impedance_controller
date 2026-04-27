// Control-law implementation for my_joint_impedance_controller.
//
// Pipeline per update():
//   1. Read <joint>/{position, velocity} state interfaces.
//   2. On first cycle after activation, seed q_d to measured q (hold pose).
//   3. Pick up latest target message from the RT buffer and, if it validates
//      (length/name/finiteness checks, clamp to URDF limits), overwrite q_d.
//   4. Compute tau = K (q_d - q) + D (qdot_d - qdot).
//   5. Apply torque rate saturation, then absolute torque saturation.
//   6. Write to <joint>/effort command interfaces and publish to ~/tau_d.

#include "my_joint_impedance_controller/my_joint_impedance_controller.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>

#include "pluginlib/class_list_macros.hpp"
#include "urdf/model.h"

#include "my_joint_impedance_controller/math.hpp"

namespace my_joint_impedance_controller {

namespace {

// Pull the URDF XML from /robot_state_publisher.
std::string fetch_robot_description(const rclcpp_lifecycle::LifecycleNode::SharedPtr& node) {
    auto client = std::make_shared<rclcpp::AsyncParametersClient>(node, "robot_state_publisher");
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(),
                     "/robot_state_publisher parameter service not available within 5 s");
        return {};
    }
    auto future = client->get_parameters({"robot_description"});
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
        RCLCPP_ERROR(node->get_logger(),
                     "Timed out waiting for /robot_state_publisher to return robot_description");
        return {};
    }
    const auto result = future.get();
    if (result.empty()) {
        return {};
    }
    return result[0].value_to_string();
}

} // namespace

controller_interface::CallbackReturn MyJointImpedanceController::on_init() {
    try {
        param_listener_ = std::make_shared<ParamListener>(get_node());
        params_ = param_listener_->get_params();
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Exception while initialising parameter listener: %s", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
MyJointImpedanceController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cfg.names.reserve(params_.joints.size());
    for (const auto& joint : params_.joints) {
        cfg.names.push_back(joint + "/effort");
    }
    return cfg;
}

controller_interface::InterfaceConfiguration
MyJointImpedanceController::state_interface_configuration() const {
    controller_interface::InterfaceConfiguration cfg;
    cfg.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    cfg.names.reserve(params_.joints.size() * 2);
    for (const auto& joint : params_.joints) {
        cfg.names.push_back(joint + "/position");
        cfg.names.push_back(joint + "/velocity");
    }
    return cfg;
}

controller_interface::CallbackReturn
MyJointImpedanceController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/) {
    params_ = param_listener_->get_params();
    const std::size_t n = params_.joints.size();

    const auto check_length = [&](const char* name, const std::vector<double>& v) {
        if (v.size() != n) {
            RCLCPP_ERROR(
                get_node()->get_logger(),
                "Parameter '%s' has length %zu but 'joints' has length %zu; these must match.",
                name, v.size(), n);
            return false;
        }
        return true;
    };
    if (!check_length("k", params_.k) || !check_length("d", params_.d) ||
        !check_length("tau_max", params_.tau_max) ||
        !check_length("max_delta_tau", params_.max_delta_tau)) {
        return controller_interface::CallbackReturn::ERROR;
    }

    // Parse URDF once on configure for joint-limit clamping.
    const std::string urdf_xml = fetch_robot_description(get_node());
    if (urdf_xml.empty()) {
        return controller_interface::CallbackReturn::ERROR;
    }
    urdf::Model model;
    if (!model.initString(urdf_xml)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to parse robot_description URDF");
        return controller_interface::CallbackReturn::ERROR;
    }
    q_lower_.assign(n, 0.0);
    q_upper_.assign(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        const auto joint = model.getJoint(params_.joints[i]);
        if (!joint) {
            RCLCPP_ERROR(get_node()->get_logger(), "Joint '%s' not found in robot_description URDF",
                         params_.joints[i].c_str());
            return controller_interface::CallbackReturn::ERROR;
        }
        if (joint->limits) {
            q_lower_[i] = joint->limits->lower;
            q_upper_[i] = joint->limits->upper;
        } else if (joint->type == urdf::Joint::CONTINUOUS) {
            q_lower_[i] = -std::numeric_limits<double>::infinity();
            q_upper_[i] = std::numeric_limits<double>::infinity();
        } else {
            RCLCPP_ERROR(get_node()->get_logger(),
                         "Joint '%s' has no <limit> tag in URDF and is not CONTINUOUS",
                         params_.joints[i].c_str());
            return controller_interface::CallbackReturn::ERROR;
        }
    }

    // Clear any stale target before subscribing.
    target_rt_buffer_.writeFromNonRT(std::shared_ptr<sensor_msgs::msg::JointState>());
    target_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
        params_.command_topic, rclcpp::SystemDefaultsQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
            target_rt_buffer_.writeFromNonRT(msg);
        });

    tau_d_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
        params_.diagnostics_topic, rclcpp::SystemDefaultsQoS());
    tau_d_rt_pub_ = std::make_shared<RealtimeJointStatePublisher>(tau_d_pub_);
    {
        auto& msg = tau_d_rt_pub_->msg_;
        msg.name = params_.joints;
        msg.effort.assign(n, 0.0);
    }

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MyJointImpedanceController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/) {
    const std::size_t n = params_.joints.size();

    // Refresh gains so a reconfigure between deactivate→activate is picked up.
    params_ = param_listener_->get_params();
    k_ = params_.k;
    d_ = params_.d;
    tau_max_ = params_.tau_max;
    max_delta_tau_ = params_.max_delta_tau;
    if (!auto_fill_critical_damping(k_, d_)) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "auto_fill_critical_damping failed: k/d length mismatch (%zu vs %zu)",
                     k_.size(), d_.size());
        return controller_interface::CallbackReturn::ERROR;
    }

    q_d_.assign(n, 0.0);
    qdot_d_.clear();
    tau_prev_.assign(n, 0.0);
    seed_on_first_update_ = true;

    // Drop any stale target message from before activation.
    target_rt_buffer_.writeFromNonRT(std::shared_ptr<sensor_msgs::msg::JointState>());

    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
MyJointImpedanceController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/) {
    // Hand over a clean effort command so whoever takes the interfaces next
    // doesn't start from our last torque.
    for (auto& iface : command_interfaces_) {
        iface.set_value(0.0);
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type
MyJointImpedanceController::update(const rclcpp::Time& time,
                                   const rclcpp::Duration& /*period*/) {
    const std::size_t n = params_.joints.size();

    // 1. Read measured state.
    std::vector<double> q(n, 0.0);
    std::vector<double> qdot(n, 0.0);
    for (std::size_t i = 0; i < n; ++i) {
        q[i] = state_interfaces_[2 * i].get_value();
        qdot[i] = state_interfaces_[2 * i + 1].get_value();
        if (!std::isfinite(q[i]) || !std::isfinite(qdot[i])) {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                                 "Non-finite state reading on joint '%s'; skipping update.",
                                 params_.joints[i].c_str());
            return controller_interface::return_type::OK;
        }
    }

    // 2. Seed hold target on first cycle post-activation.
    if (seed_on_first_update_) {
        q_d_ = q;
        qdot_d_.clear();
        seed_on_first_update_ = false;
    }

    // 3. Apply the latest user target (non-blocking).
    auto msg_ptr = *(target_rt_buffer_.readFromRT());
    if (msg_ptr) {
        std::vector<double> q_d_candidate;
        std::vector<double> qdot_d_candidate;
        const bool valid =
            validate_target_joint_state(msg_ptr->name, msg_ptr->position, msg_ptr->velocity,
                                        params_.joints, q_d_candidate, qdot_d_candidate);
        if (valid && clamp_to_limits(q_d_candidate, q_lower_, q_upper_)) {
            q_d_ = std::move(q_d_candidate);
            qdot_d_ = std::move(qdot_d_candidate);
        } else {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                                 "Ignoring malformed target_joint message on topic '%s' "
                                 "(expected %zu positions matching configured joint order).",
                                 params_.command_topic.c_str(), n);
        }
    }

    // 4. Pure PD.
    std::vector<double> tau;
    if (!compute_pd_torque(q, q_d_, qdot, qdot_d_, k_, d_, tau)) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                              "compute_pd_torque failed (internal length mismatch)");
        return controller_interface::return_type::ERROR;
    }

    // 5. Rate and absolute saturation.
    std::vector<double> tau_out;
    if (!saturate_torque_rate(tau, tau_prev_, max_delta_tau_, tau_out)) {
        return controller_interface::return_type::ERROR;
    }
    if (!saturate_torque_abs(tau_out, tau_max_)) {
        return controller_interface::return_type::ERROR;
    }

    // 6. Write commands and publish diagnostics.
    for (std::size_t i = 0; i < n; ++i) {
        command_interfaces_[i].set_value(tau_out[i]);
    }
    tau_prev_ = tau_out;

    if (tau_d_rt_pub_ && tau_d_rt_pub_->trylock()) {
        auto& msg = tau_d_rt_pub_->msg_;
        msg.header.stamp = time;
        msg.name = params_.joints;
        msg.effort = tau_out;
        tau_d_rt_pub_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

} // namespace my_joint_impedance_controller

PLUGINLIB_EXPORT_CLASS(my_joint_impedance_controller::MyJointImpedanceController,
                       controller_interface::ControllerInterface)
