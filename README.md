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

Inspect the main coordination topics:

```bash
ros2 topic echo /checkers/game_status
ros2 topic echo /checkers/board_state_report
ros2 topic echo /checkers/robot_instruction
```

## Perception Notes

The perception package:

- warps the board using the four ArUco corner markers
- checks that the board surface is visible before publishing
- classifies pieces into canonical game-engine piece codes
- publishes a stability count before the game manager accepts a board

The default colour mapping in `src/perception/config/checkers_perception.yaml` is:

- green -> black pieces
- purple -> white pieces

If your physical board uses different colours, update the HSV thresholds and piece-code mapping in that YAML file.

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
