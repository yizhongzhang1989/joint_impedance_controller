# joint_impedance_controller

A minimal, dependency-light **joint-space PD impedance controller** for any
robot whose hardware exposes per-joint `effort` command interfaces under the
ROS 2 [`ros2_control`](https://control.ros.org) framework.

The control law per cycle is the standard joint-space impedance:

$$
\tau_i \;=\; k_i\,(q^d_i - q_i) \;+\; d_i\,(\dot q^d_i - \dot q_i)
$$

with per-joint torque-rate and absolute-torque saturation, URDF joint-limit
clamping on the desired configuration, and seed-on-activate hold-pose
behaviour.

It is designed to be a clean, reusable building block: no kinematics, no
dynamics model, no filtering, no trajectory generation. If you want gravity
compensation, friction compensation, Cartesian-space impedance, or
trajectory-following, add them in another node or another controller and let
this one do exactly one thing well.

---

## 1. What this controller does (and does not) do

| Feature                                | Provided by this controller |
|----------------------------------------|----------------------------|
| Joint-space PD law `τ = K·Δq + D·Δq̇` | yes |
| Per-joint torque saturation            | yes (`tau_max`) |
| Per-joint torque-rate saturation       | yes (`max_delta_tau`) |
| URDF joint-limit clamping on `q_d`     | yes |
| Critical-damping auto-fill (`d < 0`)   | yes |
| Hold pose on activation (seed `q_d ← q`) | yes |
| Real-time-safe target subscription     | yes (`realtime_tools::RealtimeBuffer`) |
| Diagnostic publish of commanded τ       | yes (`~/tau_d`) |
| Gravity compensation                    | **no** — handled by the robot or another controller |
| Coriolis / friction compensation        | **no** |
| Trajectory generation / smoothing       | **no** — feed it a target stream from upstream |
| Cartesian impedance                     | **no** |

The **only** thing this controller writes to the robot is a per-joint torque
command on the `<joint>/effort` command interface. The torque is a pure PD
spring around the latest `q_d` received on `command_topic`.

---

## 2. How a single update cycle works

Every controller cycle (`update(time, period)` is called by `ros2_control` at
the configured update rate, e.g. 500 Hz on UR e-Series), the controller does:

1. **Read current state.** Read `<joint>/position` and `<joint>/velocity` for
   every joint listed in `joints` (in the configured order).
2. **Seed on first cycle.** If this is the first `update()` after
   `on_activate`, set `q_d ← q` so the controller holds whatever pose the
   robot was in when activated. This avoids the classic surprise jump when
   activating with a stale or unset reference.
3. **Consume target (RT-safe).** Read the latest pointer from the
   `RealtimeBuffer`. If it points at a `sensor_msgs/JointState` that
   validates (right length, finite numbers, optional name match), and after
   clamping its `position` array into the URDF joint limits, update `q_d`
   (and `qdot_d` if the message carries velocities; otherwise treat the
   feed-forward velocity as zero).
4. **Compute PD torque.** `tau[i] = k[i]·(q_d[i] − q[i]) + d[i]·(qdot_d[i] − qdot[i])`.
5. **Rate-limit.** Clamp `tau − tau_prev` to ±`max_delta_tau` per joint.
   This is the dominant safety knob during target jumps and parameter
   changes.
6. **Saturate.** Clamp `tau` to ±`tau_max` per joint.
7. **Write effort commands** to each `<joint>/effort` interface.
8. **Publish diagnostics.** Push `tau` onto the `~/tau_d` topic via a
   real-time publisher (lock-free try-publish; never blocks the RT loop).

The math primitives are header-only in
[`include/joint_impedance_controller/math.hpp`](include/joint_impedance_controller/math.hpp)
so they can be unit-tested without linking the controller.

### Lifecycle

| Lifecycle hook | Behaviour |
|----------------|-----------|
| `on_init`      | construct the `ParamListener` |
| `on_configure` | load parameters; parse URDF for joint limits; create the target subscription and `~/tau_d` publisher |
| `on_activate`  | refresh parameters; auto-fill critical damping for negative `d` entries; clear any stale target; flag `seed_on_first_update` |
| `on_deactivate`| write `0.0` to every effort command interface (clean hand-over) |

### Interface configuration

| Interface kind | Names |
|----------------|-------|
| Command        | `<joint>/effort` for each joint in `joints` |
| State          | `<joint>/position` and `<joint>/velocity` for each joint in `joints` |

If your hardware does not expose these interfaces, controller `on_configure`
fails before any torque is ever computed.

---

## 3. Parameter reference

Defined in
[`src/joint_impedance_controller.yaml`](src/joint_impedance_controller.yaml)
and validated at load time by
[`generate_parameter_library`](https://github.com/PickNikRobotics/generate_parameter_library).

| Parameter           | Type           | Default          | Meaning |
|---------------------|----------------|------------------|---------|
| `joints`            | `string[]`     | required         | Ordered list of joint names. Defines the array order of every other vector parameter. |
| `k`                 | `double[]`     | required         | Per-joint stiffness `Kₚ` (N·m/rad). Same length as `joints`. Must be ≥ 0. |
| `d`                 | `double[]`     | required         | Per-joint damping `Kd` (N·m·s/rad). Same length as `joints`. **Any negative entry → critical damping** `2·√k_i` is auto-filled at activation. |
| `tau_max`           | `double[]`     | required         | Absolute per-joint torque clamp (N·m). Must be ≥ 0. The final saturation step. |
| `max_delta_tau`     | `double[]`     | required         | Per-cycle torque rate limit (N·m). Must be ≥ 0. Effective rate limit is `max_delta_tau / control_period`. At 500 Hz, `0.5` ⇒ 250 N·m/s. |
| `command_topic`     | `string`       | `/target_joint`  | Topic for the `sensor_msgs/JointState` desired configuration. |
| `diagnostics_topic` | `string`       | `~/tau_d`        | Topic on which the commanded torque is republished. |

All numeric vectors must have the same length as `joints`; otherwise
`on_configure` fails loudly.

### Why `d = -1` (the sentinel)?

A negative entry in `d` is interpreted as **"auto-fill critical damping"** at
`on_activate` time:

$$
d_i \;\leftarrow\; 2\sqrt{k_i}
$$

So `d: [-1, -1, -1, -1, -1, -1]` with `k: [300, 300, 300, 150, 150, 150]`
yields the effective damping `[34.6, 34.6, 34.6, 24.5, 24.5, 24.5]` at run
time. Mix freely: `d: [40, -1, -1, 20, 20, -1]` overrides three entries and
auto-fills the rest. Note that the parameter still **reads back** as `-1`
through the parameter API; only the cached internal value is replaced. If
you want explicit damping values to persist, set them as positive numbers.

### Topic conventions

* **Input** (`command_topic`, default `/target_joint`):
  `sensor_msgs/JointState` with `position` of length `n`. `name` is optional
  but, if present, must equal `joints` element-wise. `velocity` is optional;
  if non-empty, it acts as feed-forward `q̇_d`. Anything that fails
  validation is silently dropped — `tau_d` will simply hold the previous
  reference.
* **Output** (`diagnostics_topic`, default `~/tau_d`):
  `sensor_msgs/JointState` with `name` set to `joints` and `effort` set to
  the commanded torque. Useful for plotting, parameter tuning, and as a
  unambiguous "did the controller see my target?" indicator.

### Saving parameters back to YAML

If you tune `k` / `d` / `tau_max` / `max_delta_tau` live (e.g. through a web
dashboard) and want the values to persist, write them back via your own YAML
serializer; the controller does not auto-persist parameter changes to disk.

---

## 4. How to integrate into a `ros2_control`-based robot

Follow these four steps for any robot. Examples below use a 6-DoF arm but
nothing is hard-coded to 6 joints.

### 4.1 Make `<joint>/effort` available

Edit your `ros2_control` xacro/URDF block so each driven joint exports an
`effort` command interface and at least `position` and `velocity` state
interfaces. Example:

```xml
<ros2_control name="my_arm" type="system">
  <hardware> ... </hardware>
  <joint name="joint_1">
    <command_interface name="position"/>
    <command_interface name="velocity"/>
    <command_interface name="effort"/>     <!-- required -->
    <state_interface name="position"/>     <!-- required -->
    <state_interface name="velocity"/>     <!-- required -->
    <state_interface name="effort"/>
  </joint>
  <!-- repeat for every controlled joint -->
</ros2_control>
```

Verify with:

```sh
ros2 control list_hardware_interfaces | grep -E 'effort|position|velocity'
```

You should see `<joint>/effort [available] [unclaimed]` for every joint
before any controller is activated.

> **Hardware reality check.** Some robot drivers expose an `effort` command
> interface but do not actually pass torques to the motors (e.g. older
> firmware, simulators that ignore effort). Always run a small motion test
> first: command a non-zero target and verify both `~/tau_d` is non-zero
> *and* the joint actually moves. If the first is true and the second is
> not, the issue is downstream of this controller.

### 4.2 Declare the controller in your controller-manager YAML

Two parts. First, register the controller type with the controller manager:

```yaml
/**:
  controller_manager:
    ros__parameters:
      update_rate: 500   # Hz; pick what your hardware supports

      my_arm_impedance:                                  # ← any name
        type: joint_impedance_controller/JointImpedanceController
```

Second, give that named instance its parameters:

```yaml
my_arm_impedance:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6
    k:             [300.0, 300.0, 300.0, 150.0, 150.0, 150.0]
    d:             [-1.0,  -1.0,  -1.0,  -1.0,  -1.0,  -1.0]   # critical
    tau_max:       [100.0, 100.0, 100.0,  50.0,  50.0,  50.0]
    max_delta_tau: [3.0,   3.0,   3.0,    3.0,   3.0,   3.0]
    command_topic: "/target_joint"
    diagnostics_topic: "~/tau_d"
```

The instance name (`my_arm_impedance` here) is what you use with `ros2
control switch_controllers` and what becomes the node name. The `type:` line
must always be `joint_impedance_controller/JointImpedanceController`.

### 4.3 Spawn the controller

In your launch file:

```python
spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["my_arm_impedance", "--inactive",
               "--controller-manager", "/controller_manager"],
)
```

Spawning `--inactive` is recommended: load it during bring-up but don't let
it grab the effort interfaces until you explicitly switch to it. That way a
position-mode controller can come up first and the robot stays in a known
mode until you opt in.

### 4.4 Activate and command it

```sh
# Make this controller take over the effort interfaces (e.g. from a
# position-mode trajectory controller).
ros2 control switch_controllers \
    --activate my_arm_impedance \
    --deactivate scaled_joint_trajectory_controller \
    --strict

# Watch the commanded torque (this should always be alive).
ros2 topic echo /my_arm_impedance/tau_d

# Send a target. NOTE: ros2 topic pub --once is unreliable for a single
# message into a reliable QoS subscription. Use a sustained publisher or
# a Python node.
ros2 topic pub -r 20 /target_joint sensor_msgs/msg/JointState '{
  name: [joint_1, joint_2, joint_3, joint_4, joint_5, joint_6],
  position: [0.0, -1.5, 1.5, -1.5, -1.5, 0.0]
}'
```

When you're done, switch back to whatever was holding the position interface
before:

```sh
ros2 control switch_controllers \
    --activate scaled_joint_trajectory_controller \
    --deactivate my_arm_impedance --strict
```

The controller writes `0.0` to all effort commands on deactivate, so the
hand-over is clean.

---

## 5. Tuning per robot

The same code runs on a 1 kg cobot wrist or a 50 kg industrial elbow — only
the parameters change. The four scaling families are:

### 5.1 Stiffness `k` (N·m/rad)

* Choose `k` from the **closed-loop natural frequency** you want:
  $\omega_n = \sqrt{k/m_\mathrm{eff}}$. For a stiff feel without
  exciting structural modes, pick `ωₙ` well below the robot's first
  resonance (typically 5–15 Hz on a UR-class arm, ~30 Hz on a Franka).
* As a rule of thumb, distal joints (wrists) tolerate **less** stiffness
  than proximal joints (shoulders) because their effective inertia is
  smaller and their resonances are higher.
* Start conservative, then double until you get the steady-state tracking
  you want without buzz. Typical starting points:

  | Robot family | Shoulder/Elbow `k` | Wrist `k` |
  |--------------|--------------------|-----------|
  | Franka FR3   | 600–1200           | 200–600   |
  | UR e-Series  | 200–400            | 50–200    |
  | KUKA LBR med | 600–1000           | 200–400   |
  | Cobot wrist (≤2 kg) | 100–200      | 30–80     |

> The values in this repo's UR15 config are a starting point validated only
> against UR15 wrist friction. If you migrate, **re-tune** before trusting
> the numbers.

### 5.2 Damping `d` (N·m·s/rad)

* Default to critical damping by leaving `d_i = -1` — the controller fills
  in `2·√k_i`. This rejects over-shoot without reducing tracking bandwidth.
* If the joint feels mushy or tracks poorly, **decrease** damping below
  critical (try 0.7·√k). If it buzzes, **increase** damping (1.3·√k).
* Note: critical damping is computed against an effective inertia of 1; for
  light, low-inertia joints the formula tends to over-damp slightly.

### 5.3 Absolute torque clamp `tau_max` (N·m)

* Set this **strictly below** the joint's continuous torque rating from the
  manufacturer's datasheet. UR15 datasheet: 330 / 330 / 150 / 56 / 56 / 56;
  this repo uses 100 / 100 / 100 / 50 / 50 / 50 — well under the limit so
  ros2_control can never request more than the joint can hold.
* **It is not a velocity or acceleration limit** — a target far from the
  current state is allowed to ramp the joint up to its torque limit until
  the spring `k·Δq` is reached.
* If the controller saturates `tau_max` on every step, your stiffness is
  too high relative to your reachable torque, *or* the requested step is
  larger than the rate limit can chase.

### 5.4 Per-cycle torque rate `max_delta_tau` (N·m)

* This is the most important safety knob during normal operation. It
  bounds `dτ/dt` to `max_delta_tau · control_rate`.
* Pick it so a sudden full-range target step (`Δq = q_upper − q_lower`)
  ramps to `tau_max` over **at least ~50 ms** (or as long as your
  application can tolerate). At 500 Hz, `max_delta_tau = 3.0 N·m` ⇒
  1500 N·m/s, which on UR15 wrist (`tau_max = 50`) hits saturation in
  ~33 ms. Lower this for compliant hand-overs; raise it for snappy
  trajectory tracking.
* Setting it **very low** (e.g. `0.05`) makes the controller behave like a
  smoothed reference filter; setting it **very high** (≥ `tau_max`) makes
  it a pure abs-torque clamp.

### 5.5 Per-robot config skeleton

A clean starting point for any 6-DoF arm:

```yaml
my_arm_impedance:
  ros__parameters:
    joints: [j1, j2, j3, j4, j5, j6]
    k:             [k_proximal, k_proximal, k_proximal,
                    k_wrist,    k_wrist,    k_wrist]
    d:             [-1, -1, -1, -1, -1, -1]            # critical
    tau_max:       [0.6·τ_rated_proximal] · 3
                 + [0.6·τ_rated_wrist]    · 3
    max_delta_tau: [tau_max[i] / (0.05 * update_rate)] # ~50 ms full-step
    command_topic: "/target_joint"
    diagnostics_topic: "~/tau_d"
```

Replace the placeholders with numbers, then walk the gains up while
watching `~/tau_d` until you get the response you want.

---

## 6. Build and test

This package builds with `ament_cmake`. From a colcon workspace root:

```sh
colcon build --packages-select joint_impedance_controller --symlink-install
source install/setup.bash
```

Runtime dependencies are listed in [`package.xml`](package.xml):
`rclcpp`, `rclcpp_lifecycle`, `controller_interface`, `hardware_interface`,
`pluginlib`, `realtime_tools`, `generate_parameter_library`, `sensor_msgs`,
`urdf`.

The math kernel
([`include/joint_impedance_controller/math.hpp`](include/joint_impedance_controller/math.hpp))
is header-only and intentionally pure (no ROS types), so unit tests can
exercise the PD law, the saturations, the auto-damping, and the validation
function in isolation without spinning up a controller-manager.

---

## 7. Known limitations & non-goals

* **No gravity compensation.** Most modern robots (UR, Franka, KUKA) handle
  gravity in firmware; effort commands sent over `ros2_control` are added
  on top. If your hardware does *not* compensate for gravity internally,
  put a separate gravity-comp controller in line, or pick a different
  controller (e.g. `crisp_controllers`) that does inverse-dynamics.
* **No trajectory generation.** Targets are point-to-point; if you publish
  a step, you get a step. Smoothing, velocity profiling, Cartesian
  planning, etc. are upstream responsibilities. The controller's only
  filter is the rate limit `max_delta_tau`.
* **No friction or Coriolis compensation.** The PD law is purely error-
  driven. On low-stiffness gains you may see steady-state error from
  joint friction; raise `k`, accept the offset, or compensate upstream.
* **No nullspace projection.** This is a joint-space controller; for a
  redundant manipulator it does not preserve a Cartesian target. If you
  need Cartesian behaviour, use a Cartesian impedance controller.
* **Reliable QoS subscription on `command_topic`.** A bursty target stream
  is fine, but `ros2 topic pub --once` may not deliver before its
  publisher tears down. Use `-r N` or a Python node when sending a single
  setpoint by hand.

---

## 8. License

Apache-2.0. See [`package.xml`](package.xml) for maintainership.
