## Simulation

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e use_mock_hardware:=true robot_ip:=0.0.0.0

ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true

ros2 run ur5e_manoeuvring bounding_box_node --ros-args   -p board_x:=-0.075   -p board_y:=0.20   -p board_z:=0.0   -p front_dist:=0.50   -p back_dist:=0.50   -p right_dist:=1.00   -p left_dist:=0.40

ros2 run ur5e_manoeuvring chessboard_marker_node --ros-args -p origin_x:=-0.075 -p origin_y:=0.20 -p origin_z:=0.00 -p square_size:=0.05 -p rotation_steps:=2

ros2 run ur5e_manoeuvring checkerboard_pose_node --ros-args   -p origin_x:=-0.075   -p origin_y:=0.20   -p origin_z:=0.00   -p square_size:=0.05   -p rotation_steps:=2   -p hover_height:=0.20   -p descent_height:=0.10   -p velocity_scaling:=0.08   -p acceleration_scaling:=0.05   -p lift_height:=0.20

ros2 topic pub --once /checkerboard_target std_msgs/msg/String "{data: '0 7'}"  #lift #home

## Hardware

ros2 launch ur_robot_driver ur_control.launch.py \
ur_type:=ur5e \
robot_ip:=<UR5E_IP_ADDRESS>

ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true

ros2 launch robotiq_hande_driver gripper_controller_preview.launch.py use_fake_hardware:=false tty_port:=/dev/ttyUSB0

ros2 run ur5e_manoeuvring gripper_command_node   --ros-args   -p arm_model:=ur5e   -p open_position:=0.025   -p closed_position:=0.019

ros2 run ur5e_manoeuvring bounding_box_node --ros-args   -p board_x:=-0.075   -p board_y:=0.20   -p board_z:=0.0   -p front_dist:=0.50   -p back_dist:=0.50   -p right_dist:=1.00   -p left_dist:=0.40

ros2 run ur5e_manoeuvring chessboard_marker_node --ros-args -p origin_x:=-0.075 -p origin_y:=0.20 -p origin_z:=0.00 -p square_size:=0.05 -p rotation_steps:=2

ros2 run ur5e_manoeuvring checkerboard_pose_node --ros-args   -p origin_x:=-0.075   -p origin_y:=0.20   -p origin_z:=0.00   -p square_size:=0.05   -p rotation_steps:=2   -p hover_height:=0.20   -p descent_height:=0.10   -p velocity_scaling:=0.08   -p acceleration_scaling:=0.05   -p lift_height:=0.20

## Commands
ros2 topic pub --once /checkerboard_target std_msgs/msg/String "{data: '0 7'}"

ros2 topic pub --once /gripper_command std_msgs/msg/String \
"{data: 'open'}"

# board should be 7.5cm laterally and 20cm longitudinally from the arm