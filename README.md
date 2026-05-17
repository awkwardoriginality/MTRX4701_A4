# MTRX4701_A4 - UR5e Checkers Robot

This repository contains the English checkers engine, robot manipulation workflow, and board perception stack for the UR5e checkers project.

## Architecture

The integration branch keeps responsibilities separated:

- `src/checkers_bot`: game engine, state machine, manipulation commands, and ROS orchestration
- `src/perception`: camera-driven board perception, ArUco warping, stability filtering, and canonical board publishing

The shared ROS contract is:

- `/checkers/board_state`: canonical flat 64 board as `std_msgs/UInt8MultiArray`
- `/checkers/board_state_report`: structured JSON report with stability, blocked state, and confidence
- `/checkers/board_blocked`: coarse board visibility signal
- `/checkers/game_status`: structured JSON game-state snapshot
- `/checkers/robot_instruction`: human-readable instruction stream
- `/checkers/manipulation_goal`: action-like robot command goal
- `/checkers/manipulation_feedback`: action-like robot command feedback
- `/checkers/manipulation_result`: action-like robot command result

## Standalone Simulation

The pure Python simulation remains the fastest way to work on the checkers engine and GUI:

```bash
pip install -r requirements.txt
python3 src/checkers_bot/play_checkers.py
```

## ROS2 Workflow

Build the workspace:

```bash
colcon build --packages-select checkers_bot perception
source install/setup.bash
```

Launch the integrated stack:

```bash
ros2 launch checkers_bot checkers_bot.launch.py
```

Run only perception with the packaged configuration:

```bash
ros2 run perception checkers_perception --ros-args --params-file src/perception/config/checkers_perception.yaml
```

Use the standalone GUI as a no-camera fake perception source:

```bash
python3 src/checkers_bot/play_checkers.py --ros-bridge
```

In `--ros-bridge` mode:

- the GUI publishes `/checkers/board_state`, `/checkers/board_state_report`, and `/checkers/board_blocked`
- the ROS `game_manager` reacts as if perception had seen the board
- the GUI mirrors robot moves back from `/checkers/internal_board`
- local GUI AI is disabled so ROS remains the authority for robot turns

Inspect the main coordination topics:

```bash
ros2 topic echo /checkers/game_status
ros2 topic echo /checkers/board_state_report
ros2 topic echo /checkers/robot_instruction
```

---

## UR5e Simulation Quick Start

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

---

## Camera / Perception Setup

### Add the RealSense package

```bash
mkdir -p ~/realsense_ws/src
cd ~/realsense_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd ~/realsense_ws
rosdep install -i --from-path src --rosdistro jazzy -y
colcon build --symlink-install
source install/setup.bash
```

### Launch the camera

```bash
ros2 launch realsense2_camera rs_launch.py
```

Open the camera visualiser to record a bag:

```bash
realsense-viewer
```

See the camera feed in ROS2:

```bash
ros2 run rqt_image_view rqt_image_view
```

### Camera pipeline

```
Camera image
→ detect 4 ArUco tags
→ warp board to 800x800
→ run chessboard detector for 7x7 inner corners
→ if chessboard not found: publish blocked=True
→ if found: classify green/purple pieces
→ if same board for 8 frames: publish blocked=False and publish board state
```

---

## Robotic Control — Simulation

### Launch the UR5e controller

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  use_mock_hardware:=true \
  robot_ip:=0.0.0.0
```

### Launch MoveIt

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  launch_rviz:=true
```

### Attach the gripper

```bash
rosdep install -i --from-path src/robotiq_hande_description src/robotiq_hande_driver --rosdistro jazzy -y
ros2 launch ur5e_manoeuvring gripper_attached.launch.py
```

In RViz, add a second RobotModel with Description Topic: `/gripper/robot_description`

### Add a checkerboard marker

```bash
ros2 run ur5e_manoeuvring chessboard_marker_node --ros-args \
  -p origin_x:=0.30 \
  -p origin_y:=-0.20 \
  -p origin_z:=0.00 \
  -p square_size:=0.05 \
  -p rotation_steps:=1
```

| Parameter | Description |
|---|---|
| `origin_x` | Board bottom-left x relative to `base_link` (m) |
| `origin_y` | Board bottom-left y relative to `base_link` (m) |
| `origin_z` | Board height relative to `base_link` (m) |
| `square_size` | Square size (m), e.g. `0.05` = 5 cm |
| `rotation_steps` | Board rotation: 0=0°, 1=90°, 2=180°, 3=270° |

### Add a bounding box

```bash
ros2 run ur5e_manoeuvring bounding_box_node
```

### Run the Cartesian goal node

```bash
sudo apt install ros-jazzy-tf-transformations
ros2 run ur5e_manoeuvring ur5e_cartesian_node
```

---

## Robotic Control — Hardware

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=<UR5E_IP_ADDRESS>

ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  launch_rviz:=true

ros2 launch ur5e_manoeuvring gripper_attached.launch.py \
  use_fake_hardware:=false \
  tty_port:=/dev/ttyUSB0

ros2 run ur5e_manoeuvring bounding_box_node
```

---

## Direct Arm Commands

Send a joint trajectory:

```bash
ros2 action send_goal \
  /scaled_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
  "{
    trajectory: {
      joint_names: [
        shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
        wrist_1_joint, wrist_2_joint, wrist_3_joint
      ],
      points: [
        { positions: [0.0, -1.57, 0.0, -1.57, 0.0, 0.0], time_from_start: {sec: 3} }
      ]
    }
  }"
```

Open / close the gripper:

```bash
# Open
ros2 action send_goal /gripper/gripper_action_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand "command: {position: [0.025]}"

# Close
ros2 action send_goal /gripper/gripper_action_controller/gripper_cmd \
  control_msgs/action/ParallelGripperCommand "command: {position: [0.0]}"
```

Send a Cartesian goal:

```bash
ros2 topic pub --once /ur5e_cartesian_goal std_msgs/msg/String \
  "{data: '{\"x\":0.35,\"y\":0.0,\"z\":0.35,\"roll\":3.14,\"pitch\":0.0,\"yaw\":0.0,\"gripper\":0.025,\"time\":4.0}'}"
```

| Field | Description |
|---|---|
| `x`, `y`, `z` | Target position relative to `base_link` (m) |
| `roll`, `pitch`, `yaw` | End-effector orientation (rad) |
| `gripper` | Opening: `0.025` = open, `0.0` = closed (m) |
| `time` | Movement duration (s) |

---

## Perception Notes

The perception package:

- warps the board using the four ArUco corner markers
- checks that the board surface is visible before publishing
- classifies pieces into canonical game-engine piece codes
- publishes a stability count before the game manager accepts a board

The default colour mapping in `src/perception/config/checkers_perception.yaml` is:

- green → black pieces
- purple → white pieces

If your physical board uses different colours, update the HSV thresholds and piece-code mapping in that YAML file.

---

## Test Suite

The integration branch has a focused automated suite for:

- core game-engine behaviour
- perception piece classification and board-orientation mapping
- state-machine sequencing
- ROS-facing topic payload contracts and blocked-board gating

```bash
pytest src/checkers_bot/test -v
```

For narrower checks:

```bash
pytest src/checkers_bot/test/test_game_engine.py -v
pytest src/checkers_bot/test/test_perception.py -v
pytest src/checkers_bot/test/test_state_machine_integration.py -v
pytest src/checkers_bot/test/test_ros_topic_contracts.py -v
```

---

## Repository Structure

- `src/checkers_bot/checkers_bot/game_engine/`: pure Python checkers logic
- `src/checkers_bot/checkers_bot/state_machine/`: safe orchestration of human turn, AI turn, and robot execution
- `src/checkers_bot/checkers_bot/nodes/`: game manager and manipulation ROS nodes
- `src/checkers_bot/checkers_bot/protocol.py`: shared JSON payload schema for perception, game status, and manipulation
- `src/perception/perception/checkers_perception_node.py`: perception adapter publishing canonical board topics
- `src/checkers_bot/play_checkers.py`: standalone interactive simulation entry point

## Notes

- Generated `log/` artefacts are intentionally excluded from version control.
- This branch is designed for safe integration work between the perception and game-control stacks before hardware deployment.
