# MTRX4701_A4


# To launch realsense camera and publish it as a ros topic
ros2 launch realsense2_camera rs_launch.py

ros2 run perception checkers_perception --ros-args -p input_mode:=bag -p bag_path:=/home/eashan-garg/checkers.bag

realsense-viewer

ros2 run rqt_image_view rqt_image_view