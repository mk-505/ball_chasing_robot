# 🤖 Simulated Ball-Chasing Differential Drive Robot (ROS)

This project demonstrates a simulated differential drive robot in a custom Gazebo world that autonomously chases a white ball using camera input. It combines robotics simulation, computer vision, and service-based control using **ROS (Robot Operating System)**.

## 📦 ROS Packages

The project consists of two core ROS packages:

### 1. `my_robot`

- Defines the **URDF model** of the differential drive robot.
- Sets up a **custom Gazebo world** that includes:
  - Realistic environment
  - The robot and the ball
- Integrates:
  - Gazebo plugins for differential drive and camera
  - RViz for robot and sensor visualization

### 2. `ball_chaser`

- Implements the robot behavior logic:
  - `drive_bot`: ROS node exposing a `command_robot` service that sets robot velocity
  - `process_image`: Subscribes to robot camera image stream, detects the ball position, and requests the appropriate velocity command from `command_robot`

## 🧠 System Architecture

```
[ Camera Image Stream ] ---> [ process_image Node ]
                                 |
                                 v
                          [ command_robot Service ]
                                 |
                                 v
                          [ drive_bot Node ] ---> [ /cmd_vel ]
```

## 🛠 Technologies Used

- **ROS Noetic** (or ROS1)
- **Gazebo** for physics-based simulation
- **URDF** for robot modeling
- **RViz** for sensor and robot visualization
- **ROS Services & Nodes** for modular control

## 🚀 How to Launch

1. **Clone the repository** into your ROS workspace:

```bash
cd ~/catkin_ws/src
git clone https://github.com/yourusername/ball-chasing-robot.git
cd ~/catkin_ws && catkin_make
source devel/setup.bash
```

2. **Launch the simulation**:

```bash
roslaunch my_robot world.launch
```

3. **Launch the ball chaser**:

```bash
roslaunch ball_chaser ball_chaser.launch
```

The robot will start detecting and moving toward the white ball using its camera feed.

## 🔍 Visualization

- **RViz** is included to visualize:
  - Robot's camera sensor
  - Joint states and transforms
  - World geometry

## 🏗 Robot Design

- **Differential Drive** using Gazebo plugin
- **Camera Sensor** for visual input
- **URDF Modeling** includes:
  - Links and joints
  - Inertial and visual elements
  - Sensor mounting

## 📸 Ball Detection Logic

- The `process_image` node scans the raw image feed for white pixels.
- Based on the location (left, center, right), it sends velocity commands to:
  - Turn left
  - Drive forward
  - Turn right

## 📬 Contact

Built with 🛠 and ❤️ by Manroop Kalsi
