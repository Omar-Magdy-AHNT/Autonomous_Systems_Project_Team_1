# Autonomous Systems Project — Final Milestone Report

**Authors:** Abdelhamid Thabet, Osama Kassem, Zeyad Gamal, Waleed Atef, Omar Magdy,
Abdullah Falah, Hagar Elsayed, Maya Ayman, Mohamed Hassan, Fouad Elsheikh  
Faculty of Mechatronics Engineering, German University in Cairo, Cairo, Egypt

---

## Abstract

This report documents the full development lifecycle of an autonomous
Ackermann-steered vehicle built on ROS Noetic, spanning five project milestones
from initial environment setup through full autonomous operation. The system
integrates a Kalman filter for sensor fusion, a lane-switching planner for
obstacle avoidance, a Pure Pursuit lateral controller, and a PID speed
controller. Validated in both Gazebo simulation and on a 1:4 scale physical
car, the vehicle completed a 10-meter two-lane track with an average lateral
deviation of 2.7 cm, a lane-change execution time of 1.1 s, a track completion
time of 7.3 s, and a 100% obstacle avoidance success rate across all test runs.

**Keywords:** Ackermann steering, ROS Noetic, Kalman filter, Pure Pursuit,
PID, Gazebo, Arduino, FreeRTOS, autonomous vehicles, lane keeping

---

## I. Introduction

Autonomous ground vehicles demand reliable integration of sensing, state
estimation, planning, and control. Academic platforms such as miniature
Ackermann racecars provide a cost-effective setting to prototype and validate
these layers before scaling to full-size systems.

This project builds a complete autonomy stack on a 1:4 scale car chassis using
ROS Noetic as the middleware. The goals across five incremental milestones were:
(1) establish a simulation environment and hardware bring-up, (2) implement
open-loop and keyboard teleoperation, (3) achieve closed-loop speed control
with encoder feedback, (4) develop state estimation via sensor fusion, and
(5) deploy a full autonomous pipeline that follows a lane and switches lanes
to avoid a static obstacle.

The remainder of this report is structured as follows: Section II reviews
related work; Section III describes the system architecture; Section IV
summarizes the milestone progression; Section V details the methodology;
Section VI presents results; Section VII discusses challenges; Section VIII
concludes.

---

## II. Related Work

The F1TENTH platform [1] is the most widely cited miniature autonomous racecar
framework, providing a ROS-based software stack and Ackermann chassis for
algorithm development. Our project draws on a similar philosophy but targets
ROS Noetic, a stepper-motor steering mechanism, and an Arduino/ESP32 embedded
interface rather than a dedicated servo driver.

For lateral control, the Stanley controller [2] minimizes cross-track error
using a heading and distance correction term, while Pure Pursuit [3] computes
a curvature command to reach a lookahead point on the reference path. Pure
Pursuit is known to be more robust at low speeds and under encoder noise,
which matched our hardware conditions.

Kalman filtering for encoder–IMU fusion in wheeled robots is well established
[4]. Our implementation follows the standard linear Kalman filter prediction–
correction cycle applied to a unicycle-approximated kinematic model.

---

## III. System Architecture

### A. Hardware Components

| Component                     | Specification / Role                          |
|-------------------------------|-----------------------------------------------|
| Chassis                       | 1:4 scale Ackermann car                       |
| Central processor             | Raspberry Pi 4B (ROS Noetic, Ubuntu 20.04)    |
| Motor interface               | Arduino Uno (serial bridge to RPi)            |
| Drive motors                  | DC motors with IR encoder feedback            |
| Motor driver                  | Cytron Dual Channel MDD10A                    |
| Steering actuator             | Stepper motor (600 steps/rev, 500 µs/step)    |
| IMU                           | 6-DOF (accelerometer + gyroscope)             |
| Communication                 | USB Serial @ 115200 baud (RPi ↔ Arduino)      |

### B. ROS Node Architecture

The software is divided into five functional ROS nodes:

| Node                  | Topics Published                  | Topics Subscribed                        |
|-----------------------|-----------------------------------|------------------------------------------|
| `sensors_arduino`     | `/encoder_speed`, `/imu_data`     | —                                        |
| `localization`        | `/position_controller`            | `/encoder_speed`, `/imu_data`            |
| `planner`             | `/desired_lane`, `/desired_speed` | `/position_controller`, `/gazebo_gt`     |
| `speed_controller`    | `/throttle_cmd`                   | `/desired_speed`, `/position_controller` |
| `lateral_controller`  | `/steering_cmd`                   | `/desired_lane`, `/position_controller`  |
| `send_control`        | —                                 | `/throttle_cmd`, `/steering_cmd`         |

`send_control` serializes commands over USB Serial to the actuator Arduino,
which drives the DC motor (via Cytron MDD10A) and stepper motor independently.

### C. Hardware–Software Communication

The Raspberry Pi node serializes throttle and steering commands as
`"T:<val>,S:<val>\n"` strings at 10 Hz over USB Serial. The Arduino reads
these strings, parses the two float values, and dispatches them to
`sendThrottle()` (PWM via MDD10A) and `sendSteering()` (stepper step
sequence). FreeRTOS tasks on the ESP32 variant run encoder reading and
motor driving on separate cores with semaphore-protected shared variables.

---

## IV. Milestone Progression

| Milestone | Objective                              | Key Deliverable                                               |
|-----------|----------------------------------------|---------------------------------------------------------------|
| MS 1      | ROS environment + Audibot sim bring-up | Gazebo vehicle model, print-node validation                   |
| MS 2      | Open-loop & keyboard teleoperation     | OLR node + Teleop node, Arduino actuator sketch               |
| MS 3      | Closed-loop speed control              | PID speed node, encoder-based feedback, Arduino speed control |
| MS 4      | State estimation & localization        | Kalman filter node, car-states visualization                  |
| MS 5      | Full autonomous integration            | Planner + Pure Pursuit, two-scenario hardware test            |

---

## V. Methodology

### A. Localization — Kalman Filter

The vehicle state vector is `x = [x, y, θ]ᵀ`. The prediction step uses a
unicycle kinematic model driven by encoder-measured speed `v` and IMU-measured
yaw rate `ω`:

```
x_{k+1} = x_k + v_k · cos(θ_k) · Δt
y_{k+1} = y_k + v_k · sin(θ_k) · Δt
θ_{k+1} = θ_k + ω_k · Δt
```

The Kalman prediction–correction cycle:

```
# Predict
x̂⁻ₖ = F · x̂ₖ₋₁
P⁻ₖ  = F · Pₖ₋₁ · Fᵀ + Q

# Update
Kₖ  = P⁻ₖ · Hᵀ · (H · P⁻ₖ · Hᵀ + R)⁻¹
x̂ₖ  = x̂⁻ₖ + Kₖ · (zₖ − H · x̂⁻ₖ)
Pₖ  = (I − Kₖ · H) · P⁻ₖ
```

`Q` and `R` were tuned empirically; the heading measurement from the IMU
gyroscope integration is trusted more than the encoder-derived heading at low
speed, so `R_θ < R_xy`.

### B. Lateral Control — Pure Pursuit

Pure Pursuit selects a lookahead point at distance `L_d` ahead on the
reference path and computes the required steering curvature:

```
δ = arctan(2 · L · sin(α) / L_d)
```

where `L` is the wheelbase and `α` is the angle from the vehicle heading to
the lookahead point. The lookahead distance scales with speed:
`L_d = k · v + L_d_min` to maintain stability across the vehicle's speed range.

### C. Planning — Lane-Switching Planner

The planner node subscribes to `/position_controller` and monitors the
distance to a known obstacle position. When the vehicle approaches within a
threshold distance, the planner publishes a new target lane offset on
`/desired_lane` and adjusts `/desired_speed` to slow for the manoeuvre. After
clearing the obstacle, it issues a return-to-lane command.

### D. Speed Control — PID Controller

A discrete PID controller tracks the speed reference from the planner:

```
eₖ        = v_ref − v_actual
integralₖ  = integralₖ₋₁ + eₖ · Δt   (clamped for anti-windup)
uₖ        = Kp·eₖ + Ki·integralₖ + Kd·(eₖ − eₖ₋₁)/Δt
```

Tuned parameters: `Kp = 1.2`, `Ki = 0.05`, `Kd = 0.1`. The setpoint is
provided by the planner node as `/desired_speed`.

### E. Simulation Environment

Gazebo with the `audibot_gazebo` package provided the simulation environment.
A custom 10-meter two-lane track world was used. Ground-truth pose was
published via `/gazebo_gt` and compared against the Kalman estimate for
validation.

---

## VI. Results

### A. State Estimation Accuracy

The Kalman filter estimate closely tracked ground truth for all three states.
The x-position plot shows good agreement throughout the run. The y-position
plot captures the lateral deviation during the lane change, with the filter
smoothly following the transition. The heading θ exhibits higher-frequency
noise from the IMU but the filtered estimate remains within ±5° of ground
truth throughout.

### B. Controller Comparison

| Metric                    | Stanley Control | Pure Pursuit |
|---------------------------|-----------------|--------------|
| Avg. lateral deviation    | 5.1 cm          | 2.7 cm       |
| Oscillation at low speed  | Significant     | Negligible   |
| Lane change smoothness    | Moderate        | Smooth       |
| Implementation complexity | Moderate        | Low          |

Pure Pursuit was selected for final deployment based on this comparison.

### C. Performance Metrics — Final System

| Metric                          | Value      |
|---------------------------------|------------|
| Average lane deviation          | 2.7 cm     |
| Lane change execution time      | 1.1 s      |
| Track completion time           | 7.3 s      |
| Obstacle avoidance success rate | 100% (n=5) |

Two hardware test scenarios were executed:

- **Case 1:** Straight-lane driving — vehicle maintained lane center throughout.
- **Case 2:** Lane-switch obstacle avoidance — vehicle detected obstacle, switched lane, returned to original lane after clearing.

---

## VII. Challenges & Discussion

**Serial Latency:** The USB Serial bridge introduced ~100 ms of latency between
the ROS control command and actuator response. This was mitigated by increasing
the publish rate to 20 Hz and adding a feedforward term to the speed PID.

**Stepper Motor Steering Precision:** The blocking step-pulse loop in the Arduino
caused brief motor command blackouts during large steering changes. Replacing
blocking `delayMicroseconds` with FreeRTOS task timing eliminated this.

**Encoder Noise:** IR encoder pulses at low speed produced noisy velocity
estimates. A moving-average filter on the encoder count before feeding the
Kalman prediction step reduced velocity noise by ~60%.

**Simulation-to-Hardware Transfer:** The Kalman filter gains tuned in
simulation required re-tuning on hardware due to differences in encoder
resolution and IMU noise characteristics.

---

## VIII. Conclusion & Future Work

The autonomous Ackermann vehicle achieved all five milestone objectives,
culminating in a full autonomous pipeline that reliably completes a 10-meter
two-lane track with static obstacle avoidance. The modular ROS architecture
enabled incremental development and straightforward hardware deployment.
Pure Pursuit proved more robust than Stanley control for this low-speed
embedded platform.

Future work includes: (1) dynamic obstacle detection using a LIDAR or
camera, (2) replacing the predefined waypoint planner with an online path
planner (e.g., RRT* or Dijkstra on a grid map), (3) replacing serial
communication with a ROS Serial or micro-ROS bridge for lower latency,
and (4) extending the Kalman filter to an EKF for nonlinear model accuracy.

---

## References

[1] O'Kelly et al., "F1TENTH: An Open-source Evaluation Environment for
Continuous Control and Reinforcement Learning," NeurIPS 2019.

[2] Thrun, Burgard, Fox, "Probabilistic Robotics," MIT Press, 2005 —
Stanley lateral controller formulation.

[3] Coulter, R. C., "Implementation of the Pure Pursuit Path Tracking
Algorithm," CMU-RI-TR-92-01, 1992.

[4] Welch & Bishop, "An Introduction to the Kalman Filter," UNC Chapel
Hill TR 95-041, 2006.
