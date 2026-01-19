# Tower of Hanoi with xArm6

This is a course project for the postgraduate level course of [Smart Robotics](https://unimore.coursecatalogue.cineca.it/corsi/2022/10300/insegnamenti/2023/25796/2021/10003) taught at [DIEF, UniMoRe](https://inginf.unimore.it/laurea-magistrale-ing-inf/).

![Demo](demo.gif)

Quickstart:

- Setup ROS2 Humble
- Clone and build: https://github.com/xArm-Developer/xarm_ros2/tree/humble and https://github.com/IFRA-Cranfield/IFRA_LinkAttacher/tree/humble in ~/ros2_ws 
- In ~/.bashrc add: "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash" and also this repository's path

```bash

# 0. Build the packages
colcon build --symlink-install

# 1. Start Gazebo, RViz and all the nodes
ros2 launch hanoi_tower demo.launch.py

# 2. Trigger service to start the demo
ros2 service call /hanoi/start std_srvs/srv/Trigger

# Solver state
ros2 topic echo /hanoi/solver_state

# Current move
ros2 topic echo /hanoi/current_move

# Cube poses
ros2 topic echo /hanoi/cube_small_pose
ros2 topic echo /hanoi/cube_medium_pose
ros2 topic echo /hanoi/cube_large_pose
```