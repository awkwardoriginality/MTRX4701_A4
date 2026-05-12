# MTRX4701_A4


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
