## Simulation

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e use_mock_hardware:=true robot_ip:=0.0.0.0

ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true

ros2 run ur5e_manoeuvring bounding_box_node --ros-args   -p board_x:=-0.075   -p board_y:=0.20   -p board_z:=0.0   -p front_dist:=0.50   -p back_dist:=0.50   -p right_dist:=1.00   -p left_dist:=0.40

ros2 run ur5e_manoeuvring chessboard_marker_node --ros-args -p origin_x:=-0.075 -p origin_y:=0.20 -p origin_z:=0.00 -p square_size:=0.05 -p rotation_steps:=2

ros2 run ur5e_manoeuvring checkerboard_pose_node --ros-args   -p origin_x:=-0.075   -p origin_y:=0.20   -p origin_z:=0.00   -p square_size:=0.05   -p rotation_steps:=2   -p hover_height:=0.20   -p descent_height:=0.10   -p velocity_scaling:=0.08   -p acceleration_scaling:=0.05   -p lift_height:=0.20

## Hardware

ros2 launch ur_robot_driver ur_control.launch.py \
ur_type:=ur5e \
robot_ip:=192.168.56.101

ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true

ros2 launch robotiq_hande_driver gripper_controller_preview.launch.py use_fake_hardware:=false tty_port:=/dev/ttyUSB0

ros2 run ur5e_manoeuvring gripper_command_node   --ros-args   -p arm_model:=ur5e   -p open_position:=0.025   -p closed_position:=0.019
# or 0.013 for closed when changed screw config

ros2 run ur5e_manoeuvring bounding_box_node --ros-args   -p board_x:=-0.075   -p board_y:=0.20   -p board_z:=0.0   -p front_dist:=0.50   -p back_dist:=0.50   -p right_dist:=1.00   -p left_dist:=0.40

ros2 run ur5e_manoeuvring chessboard_marker_node --ros-args -p origin_x:=-0.075 -p origin_y:=0.20 -p origin_z:=0.00 -p square_size:=0.05 -p rotation_steps:=2

ros2 run ur5e_manoeuvring checkerboard_pose_node --ros-args   -p origin_x:=-0.075   -p origin_y:=0.20   -p origin_z:=0.00   -p square_size:=0.05   -p rotation_steps:=2   -p hover_height:=0.25   -p descent_height:=0.08   -p velocity_scaling:=0.08   -p acceleration_scaling:=0.05   -p lift_height:=0.08

## Commands
ros2 topic pub --once /checkerboard_target std_msgs/msg/String "{data: '0 7'}"

ros2 topic pub --once /gripper_command std_msgs/msg/String \
"{data: 'open'}"

or

ros2 topic pub --once /robot_move std_msgs/msg/String "{data: '{\"from\":[0,7], \"to\":[7,0]}'}"
ros2 topic pub --once /gripper_done std_msgs/msg/Bool "{data: true}"

# board should be 7.5cm laterally and 20cm longitudinally from the arm


## State machine
ros2 run game_state_machine robot_controller
ros2 run game_state_machine game_controller
ros2 run perception checkers_perception --ros-args --params-file src/perception/config/checkers_perception.yaml
ros2 launch realsense2_camera rs_launch.py
ros2 run rqt_image_view rqt_image_view

