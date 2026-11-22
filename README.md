# TurtleBot3 Robot Platoon

Autonomous platooning system for TurtleBot3 using ROS 2 and local ego-centric LiDAR perception. The architecture strips away multi-agent wireless orchestration; each robot acts as an independent reactive agent tracking the object directly in front of it.

## Environment
- **OS:** Ubuntu 24.04 LTS
- **ROS 2:** Jazzy
- **Simulator:** Gazebo Harmonic (`gz sim`)
- **Robot:** TurtleBot3 Burger

## Repository Structure
All custom development is contained within the `turtlebot3_gazebo` package. 
Paths below are relative to: `src/turtlebot3_simulations/turtlebot3_gazebo/`

### Milestone 2: Single-Robot Distance-Lock
A virtual TurtleBot speeds up and brakes to maintain a 0.30m gap from a dynamic wall using a classical PD controller.
- `scripts/lidar_filter.py`: Filters 360° LiDAR to a ±15° forward cone and extracts the minimum valid distance.
- `scripts/pd_controller.py`: 1D PD controller mapping distance error to linear velocity (`/cmd_vel`).
- `scripts/moving_wall.py`: Generates smooth, physics-based prismatic joint motion for the dynamic wall.
- `scripts/velocity_test.py`: Diagnostic open-loop script for kinematic calibration.
- `worlds/empty_world.world`: Gazebo world containing the dynamic wall model.
- `params/turtlebot3_burger_bridge.yaml`: ROS 2 ↔ Gazebo topic bridge configuration.

### Milestone 3: Twin-Robot Coupling
A follower robot tracks the leader's distance and angular offset using only local LiDAR. Includes edge-tracking logic to prevent inner-corner cutting during turns.
- `scripts/twin_lidar_target.py`: Extracts distance and bearing angle to the leader (supports center/left/right edge tracking).
- `scripts/twin_follower_controller.py`: Coupled 2D controller managing linear distance-locking and angular steering.
- `scripts/twin_leader_motion.py`: Scripted velocity profile (straight lines and arcs) for the leader robot.
- `launch/twin_robots.launch.py`: Spawns both leader and follower simultaneously with isolated namespaces.
- `params/turtlebot3_burger_leader_bridge.yaml`: Bridge config isolating the leader's topics.
- `params/turtlebot3_burger_follower_bridge.yaml`: Bridge config isolating the follower's topics.
- `models/turtlebot3_burger_leader/`: Namespaced SDF model for the leader.
- `models/turtlebot3_burger_follower/`: Namespaced SDF model for the follower.

## Build & Source
```bash
cd ~/turtlebot3_ws
colcon build --packages-select turtlebot3_gazebo
source install/setup.bash
