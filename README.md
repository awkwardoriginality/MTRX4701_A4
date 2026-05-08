# MTRX4701_A4


# To launch realsense camera and publish it as a ros topic
ros2 launch realsense2_camera rs_launch.py

# To run the perception node
ros2 run perception checkers_perception --ros-args -p input_mode:=bag -p bag_path:=/home/eashan-garg/checkers.bag

# To open the camera visualiser
realsense-viewer

# To see the camera feed in ros2
ros2 run rqt_image_view rqt_image_view


Camera image
→ detect 4 ArUco tags
→ warp board to 800x800
→ run chessboard detector for 7x7 inner corners
→ if chessboard not found: publish blocked=True
→ if found: classify green/purple pieces
→ if same board for 8 frames: publish blocked=False and publish board state


game controller sequence

1. save the initial state / previous state of board by waiting until blocked = false
2. Tell the user ready to play. Make your move.

State machine 1: if blocked = true. Wait
                 if blocked = false and previous state = current state. Wait
                 if blocked = false and previous state != current state. Continue

3. Robot makes its move
