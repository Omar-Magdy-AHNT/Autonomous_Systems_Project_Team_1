# Autonomous Systems Project — Team 1

> A ROS Noetic autonomous Ackermann-steered vehicle capable of lane keeping
> and obstacle avoidance, validated in Gazebo simulation and on a 1:4 scale
> physical car.

---

## Project Overview

This project implements a complete autonomy stack for a 1:4 scale Ackermann
car using ROS Noetic. The system integrates:

- **Localization** via a Kalman filter fusing encoder and IMU data
- **Planning** via a lane-switching planner with velocity profiling
- **Lateral control** via Pure Pursuit (outperformed Stanley control in testing)
- **Speed control** via a discrete PID controller
- **Hardware interface** via Arduino/ESP32 over USB Serial

The vehicle was validated on a 10-meter two-lane track:

| Metric                      | Result    |
|-----------------------------|-----------|
| Average lane deviation      | 2.7 cm    |
| Lane change time            | 1.1 s     |
| Track completion time       | 7.3 s     |
| Obstacle avoidance success  | 100%      |

---

## Hardware Stack

| Component         | Part                              |
|-------------------|-----------------------------------|
| Chassis           | 1:4 scale Ackermann car           |
| Processor         | Raspberry Pi 4B (ROS Noetic)      |
| Motor interface   | Arduino Uno / ESP32               |
| Drive motors      | DC motors + IR encoder            |
| Motor driver      | Cytron Dual Channel MDD10A        |
| Steering          | Stepper motor                     |
| IMU               | 6-DOF IMU                         |

---

## Software Stack

- **ROS Noetic** on Ubuntu 20.04
- **Gazebo** (audibot_gazebo) for simulation
- **Python 3** ROS nodes
- **Arduino / FreeRTOS** for embedded actuator control

---

## Resources

- [Final Report](/Milestone_05_Team_1/Autonomous_Systems_Project_Report_Team_1.pdf)
- [Poster](/Poster.pdf)
- [Overleaf](https://www.overleaf.com/8317881217nwnzqvwkvkwh#5e357f)

---

## License

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This project is licensed under the MIT License.
See the [LICENSE](./LICENSE) file for full details.

All contributors are credited via the
[GitHub contributors list](../../graphs/contributors).
