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

# To run the perception node using the config yaml. Change ros to bag for sim
ros2 run perception checkers_perception --ros-args --params-file src/perception/config/checkers_perception.yaml

# To open the camera visualiser to record a bag
realsense-viewer

# To see the camera feed in ros2
ros2 run rqt_image_view rqt_image_view

# To run the game state machine (includes CheckersGame legality checking & AI)
ros2 run game_state_machine game_controller

# To echo the game status
ros2 topic echo /game/status

# To publish an arm movement complete message for testing purposes
ros2 topic pub /game/robot_done std_msgs/msg/Bool "{data: true}" --once


----------------------------------------------------------------------

Camera image
→ detect 4 ArUco tags
→ warp board to 800x800
→ run chessboard detector for 7x7 inner corners
→ if chessboard not found: publish blocked=True
→ if found: classify green/purple pieces
→ if same board for 8 frames: publish blocked=False and publish board state


# To add the realsense package which interfaces with the camera
mkdir -p ~/realsense_ws/src
cd ~/realsense_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd ~/realsense_ws
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash

----------------------------------------------------------------------

### Robotic control in SIMULATION

#### To launch the UR5e Controller
ros2 launch ur_robot_driver ur_control.launch.py \
ur_type:=ur5e \
use_mock_hardware:=true \
robot_ip:=0.0.0.0

#### To launch moveit
ros2 launch ur_moveit_config ur_moveit.launch.py \
ur_type:=ur5e \
launch_rviz:=true

#### To attach gripper arm
rosdep install -i --from-path src/robotiq_hande_description src/robotiq_hande_driver --rosdistro jazzy -y

ros2 launch ur5e_manoeuvring gripper_attached.launch.py

In RViz, add second RobotModel:
Description Topic: /gripper/robot_description

#### To add a checker board
ros2 run ur5e_manoeuvring chessboard_marker_node --ros-args \
-p origin_x:=0.60 \
-p origin_y:=-0.25 \
-p origin_z:=0.00 \
-p square_size:=0.05 \
-p rotation_steps:=3

origin_x         # board bottom-left x position relative to base_link (metres)
origin_y         # board bottom-left y position relative to base_link (metres)
origin_z         # board height relative to base_link (metres)
square_size      # checkerboard square size (metres) -> 0.05 = 5 cm
rotation_steps   # board rotation: 0=0°, 1=90°, 2=180°, 3=270°

#### To add a bounding box
ros2 run ur5e_manoeuvring bounding_box_node

#### To run cartesian goal sending node
sudo apt install ros-jazzy-tf-transformations

ros2 run ur5e_manoeuvring checkerboard_pose_node --ros-args \
  -p origin_x:=-0.60 \
  -p origin_y:=0.25 \
  -p origin_z:=0.00 \
  -p square_size:=0.05 \
  -p rotation_steps:=3 \
  -p hover_height:=0.10 \
  -p descent_height:=0.05 \
  -p velocity_scaling:=0.08 \
  -p acceleration_scaling:=0.05 \
  -p lift_height:=0.20

  ros2 topic pub --once /checkerboard_target std_msgs/msg/String "{data: '2 3'}"


----------------------------------------------------------------------

### Robotic Control in HARDWARE

ros2 launch ur_robot_driver ur_control.launch.py \
ur_type:=ur5e \
robot_ip:=<UR5E_IP_ADDRESS>

ros2 launch ur_moveit_config ur_moveit.launch.py \
ur_type:=ur5e \
launch_rviz:=true

ros2 launch ur5e_manoeuvring gripper_attached.launch.py \
use_fake_hardware:=false \
tty_port:=/dev/ttyUSB0

ros2 launch robotiq_hande_driver gripper_controller_preview.launch.py use_fake_hardware:=false tty_port:=/dev/ttyUSB0

ros2 run ur5e_manoeuvring bounding_box_node

ros2 run ur5e_manoeuvring gripper_command_node --ros-args \
  -p arm_model:=ur5e

ros2 run ur5e_manoeuvring gripper_command_node --ros-args \
  -p arm_model:=ur5

ros2 topic pub --once /gripper_command std_msgs/msg/String \
"{data: 'open'}"

ros2 topic pub --once /gripper_command std_msgs/msg/String \
"{data: 'close'}"

----------------------------------------------------------------------

## Commands to test arm directly
ros2 action send_goal \
/scaled_joint_trajectory_controller/follow_joint_trajectory \
control_msgs/action/FollowJointTrajectory \
"{
  trajectory: {
    joint_names: [
      shoulder_pan_joint,
      shoulder_lift_joint,
      elbow_joint,
      wrist_1_joint,
      wrist_2_joint,
      wrist_3_joint
    ],
    points: [
      {
        positions: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],
        time_from_start: {sec: 3}
      }
    ]
  }
}"

ros2 action send_goal \
/gripper/gripper_action_controller/gripper_cmd \
control_msgs/action/ParallelGripperCommand \
"command:
  position: [0.0]
"

## In hardware
ros2 action send_goal \
/gripper_action_controller/gripper_cmd \
control_msgs/action/ParallelGripperCommand \
"command:
  position: [0.0]
"
## In sim
ros2 action send_goal \
/gripper/gripper_action_controller/gripper_cmd \
control_msgs/action/ParallelGripperCommand \
"command:
  position: [0.025]
"

# Command to send a Cartesian goal to the UR5e node
ros2 topic pub --once /ur5e_cartesian_goal std_msgs/msg/String \
"{data: '{\"x\":0.35,\"y\":0.0,\"z\":0.35,\"roll\":3.14,\"pitch\":0.0,\"yaw\":0.0,\"gripper\":0.025,\"time\":4.0}'}"


x        = target x position relative to base_link (metres)
y        = target y position relative to base_link (metres)
z        = target z position relative to base_link (metres)

roll     = end-effector roll angle (radians)
pitch    = end-effector pitch angle (radians)
yaw      = end-effector yaw angle (radians)

gripper  = gripper opening amount (metres)
             0.025 = open
             0.0   = closed

time     = movement duration (seconds)
