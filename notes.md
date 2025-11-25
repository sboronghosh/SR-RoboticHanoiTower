1. Install ROS2 Foxy
2. https://github.com/xArm-Developer/xarm_ros2/tree/foxy > Using XARM supports ROS2 FOXY and Ubuntu 20.04 
    ```
        boron@themachine:/media/boron/data/SR-RoboticHanoiTower/src$ git submodule add -b "$ROS_DISTRO" https://github.com/xArm-Developer/xarm_ros2.git xarm_ros2
        boron@themachine:/media/boron/data/SR-RoboticHanoiTower/src$ git submodule update --init --recursive
        boron@themachine:/media/boron/data/SR-RoboticHanoiTower/src/xarm_ros2$ rosdep update --include-eol-distros
        boron@themachine:/media/boron/data/SR-RoboticHanoiTower/src/xarm_ros2$ rosdep install --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
        boron@themachine:/media/boron/data/SR-RoboticHanoiTower/$ colcon build --symlink-install
    ```
3. Open Gazebo: boron@themachine:/media/boron/data/SR-RoboticHanoiTower$ ros2
   launch xarm_gazebo xarm6_beside_table_gazebo.launch.py add_gripper:=true
4. Open RViz: boron@themachine:/media/boron/data/SR-RoboticHanoiTower$ ros2
   launch xarm_description xarm6_rviz_display.launch.py add_gripper:=true
