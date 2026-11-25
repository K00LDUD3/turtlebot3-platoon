# Ego-Centric Multi-Robot Platooning with LiDAR Target Lock Pursuit

**Team: Keyboard Warriors** — CS F407 Artificial Intelligence (AY 2026–27 Semester I), BITS Pilani, K.K. Birla Goa Campus  
**Members:** Tanush Dinesh Srivatsa (2023B5A70982G), Parikshith Moleyar (2023B1A70654G), Eshan Saikia (2023A7PS1024G)

**Project Report:** [Doc.pdf](Doc.pdf)

A decentralized, communication-free multi-robot platooning system for TurtleBot3 Burger in ROS 2 Jazzy and Gazebo Harmonic. Each follower runs identical reactive nodes using only its own 2D LiDAR (`/scan`) and wheel odometry (`/odom`) to maintain a `0.30 m` headway behind the robot directly ahead—with zero inter-robot wireless communication or global tracking.

---

## Environment
- **OS:** Ubuntu 24.04 LTS
- **ROS 2:** Jazzy
- **Simulator:** Gazebo Harmonic (`gz sim`)
- **Robot:** TurtleBot3 Burger (outer width `17.8 cm`, `360`-ray `5 Hz` LiDAR with `1 cm` Gaussian noise)

---

## Repository Structure
All custom development is inside `src/turtlebot3_simulations/turtlebot3_gazebo/` (paths below are relative to that package), and the final report is at the repository root (`Doc.pdf`):

### 1D Distance-Lock (Midsem)
- `scripts/lidar_filter.py`: Filters 360° LiDAR to a `±15°` forward cone and publishes the minimum valid distance.
- `scripts/pd_controller.py`: 1D PD controller (`Kp=4.0, Ki=0.0, Kd=0.4`, `d*=0.30 m`, `±0.20 m/s` saturation) with CSV telemetry logging.
- `scripts/moving_wall.py`: Physics-based prismatic joint wall motion (`0.60–1.40 m`, `|v| ≤ 0.15 m/s`, `|a| ≤ 0.15 m/s²`).
- `scripts/velocity_test.py`: Open-loop step velocity test for kinematic calibration.

### 2D Multi-Robot Platooning (Final)
- `scripts/twin_lidar_target.py`: Calibrated (`5`-scan startup lock) spatial clusterer (`cluster_jump=0.055 m`, `max_cluster_width=0.22 m`) with local odom-frame constant-velocity gating (`track_gate=0.14 m`) to maintain target lock amidst nearby pillars and decoys.
- `scripts/twin_follower_controller.py`: Coupled longitudinal PD gap controller (`d*=0.30 m`) + **local breadcrumb Pure Pursuit** steering (`waypoint_spacing=0.05 m`, `lookahead_distance=0.16 m`) to eliminate cascading corner-cutting. Logs per-robot CSV telemetry to `~/turtlebot3_ws/logs/`.
- `scripts/twin_leader_motion.py`: Scripted S-curve leader motion (`v=0.12 m/s`, `w=±0.20 rad/s`).
- `launch/platoon.launch.py`: Parameterized multi-follower launch (`num_followers:=1, 2, 3, ...`) that dynamically generates namespaced follower SDFs/bridges and launches each follower's independent target tracker and controller.
- `launch/twin_robots.launch.py`: Two-robot launch supporting a `world` argument (`empty_world.world`, `obstacle_course.world`, `cluttered_course.world`).
- `worlds/obstacle_course.world`: S-curve course with two `1.5×` robot-width (`27 cm`) cylinder gates (`4.6 cm` wheel clearance per side).
- `worlds/cluttered_course.world`: Same `1.5×` S-curve gates plus `16` surrounding junk/decoy obstacles (`10` turret-sized decoys, `4` boxes, `2` walls) to stress-test target lock.

---

## Build & Setup
```bash
source /opt/ros/jazzy/setup.bash
cd ~/turtlebot3_ws
colcon build --symlink-install --packages-select turtlebot3_gazebo
source install/setup.bash
```

---

## Reproducing Reported Results

### 1. Multi-Follower Platoon on `1.5×` S-Curve Gates (`n = 1` or `n = 2`)
**Terminal 1 — Gazebo + Leader + Followers (wait for `TARGET LOCKED` logs):**
```bash
source /opt/ros/jazzy/setup.bash && source ~/turtlebot3_ws/install/setup.bash
ros2 launch turtlebot3_gazebo platoon.launch.py num_followers:=2 world:=obstacle_course.world
```
*(Use `num_followers:=1` for the single-follower test, or `num_followers:=2` for the 2-follower platoon.)*

**Terminal 2 — Leader S-Curve Motion:**
```bash
source /opt/ros/jazzy/setup.bash && source ~/turtlebot3_ws/install/setup.bash
ros2 run turtlebot3_gazebo twin_leader_motion.py \
  --ros-args -p cmd_topic:=/leader/cmd_vel -p linear_speed:=0.12 -p angular_speed:=0.20 -p loop:=false
```

### 2. Cluttered 16-Decoy Target-Lock Test (`n = 2`)
**Terminal 1:**
```bash
source /opt/ros/jazzy/setup.bash && source ~/turtlebot3_ws/install/setup.bash
ros2 launch turtlebot3_gazebo platoon.launch.py num_followers:=2 world:=cluttered_course.world
```
**Terminal 2:** Run the same `twin_leader_motion.py` command as above once both followers log `TARGET LOCKED`.

> **Tip (CPU throttling):** On laptops, you can wrap each command in `systemd-run --user --scope -p CPUQuota=70% bash -c "..."` to cap CPU usage.

---

## Summary of Results (`d* = 0.30 m`, post-settling `t > 3 s`)

| Scenario | MAE (cm) ↓ | RMSE (cm) ↓ | Within ±5 cm ↑ |
| :--- | :---: | :---: | :---: |
| 1D Moving Wall (Test 2) | 4.43 | 7.51 | 88.0% |
| `1.5×` Gates (`n=1`): Follower 1 | 1.68 | 2.20 | 98.7% |
| `1.5×` Gates (`n=2`): Follower 1 | 1.69 | 2.26 | 98.2% |
| `1.5×` Gates (`n=2`): Follower 2 | 2.06 | 2.73 | 93.2% |
| Cluttered 16-Decoy (`n=2`): Follower 1 | 2.79 | 3.22 | 93.8% |
| Cluttered 16-Decoy (`n=2`): Follower 2 | 3.01 | 3.39 | 90.1% |

All runs completed with **zero collisions** and **zero false target locks**. Full details, equations, and analysis are in [Doc.pdf](Doc.pdf).
