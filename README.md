# MTRX4701_A4

## UR5e simulation quick start

Use these in separate terminals from the workspace root:

Terminal 0 (one-time build or after code changes):

	source /opt/ros/jazzy/setup.bash
	colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

Terminal 1 (UR driver with mock hardware):

	source /opt/ros/jazzy/setup.bash
	source install/setup.bash
	ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.10 use_mock_hardware:=true launch_rviz:=true

Terminal 2 (MoveIt interface):

	source /opt/ros/jazzy/setup.bash
	source install/setup.bash
	ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true

Terminal 3 (workspace bounding-box visualizer/script):

	source /opt/ros/jazzy/setup.bash
	source install/setup.bash
	python3 bounding_box.py


# To launch realsense camera and publish it as a ros topic
ros2 launch realsense2_camera rs_launch.py

# To run the perception node with a realsense bag
ros2 run perception checkers_perception --ros-args -p input_mode:=bag -p bag_path:=/home/eashan-garg/checkers1.bag

# To run the perception node with live camera feed
ros2 run perception checkers_perception --ros-args \
-p input_mode:=ros \
-p image_topic:=/camera/camera/color/image_raw

# To open the camera visualiser to record a bag
realsense-viewer

# To see the camera feed in ros2
ros2 run rqt_image_view rqt_image_view

# To run the game state machine
ros2 run game_engine game_controller

# To echo the game status
ros2 topic echo /game/status

# To publish an arm movement complete message for testing purposes
ros2 topic pub /game/robot_done std_msgs/msg/Bool "{data: true}" --once


Camera image
→ detect 4 ArUco tags
→ warp board to 800x800
→ run chessboard detector for 7x7 inner corners
→ if chessboard not found: publish blocked=True
→ if found: classify green/purple pieces
→ if same board for 8 frames: publish blocked=False and publish board state
