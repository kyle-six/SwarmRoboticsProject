# SwarmRoboticsProject

# Installation 
1) Make sure you have Ubuntu 20.04 and ROS2 (FOXY VERSION) installed
2) Clone this repo somewhere!
3) cd ~/SwarmRoboticsProject/
4) colcon build --symlink-install
   - this command builds the project and creates 3 directories: /log, /install, and /build (do NOT push these to the repo)
   - --symlink-install makes it so that all the python files are editable 'in-place' so you don't need to rebuild for each change
5) source install/setup.bash
   - overlays our ros2 packages

# Useage
As of right now there are 3 things implemented

1) ros2 launch my_robot_description simulation.launch.py
  - just loads rviz2 with the basic URDF I created for now

2) ros2 launch my_robot_hardware hardware.launch.py
  - Sets up the motor_controller node to drive the steppers from the RPI, OR creates fake output if running on a laptop
  - Listens for the \cmd_vel topic

3) ros2 run my_robot_hardware cmd_vel_publisher
  - Starts a publisher on the \cmd_vel topic, that takes user keyboard input and sends velocity values
