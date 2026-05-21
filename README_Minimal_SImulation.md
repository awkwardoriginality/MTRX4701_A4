# Robotic Control — Simulation (Minimum Setup)

## Prerequisites

```bash
rosdep install -i --from-path src/robotiq_hande_description src/robotiq_hande_driver --rosdistro jazzy -y
sudo apt install ros-jazzy-tf-transformations
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Terminal 1 — UR driver (mock hardware)

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  use_mock_hardware:=true \
  robot_ip:=0.0.0.0
```

## Terminal 2 — MoveIt

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  launch_rviz:=true
```
## Terminal 3 — Cartesian goal node

```bash
ros2 run ur5e_manoeuvring ur5e_cartesian_node
```
---

## Terminal 4 — Cartesian bridge

```bash
ros2 run checkers_bot cartesian_bridge
```

## Terminal 5 — Gripper (required for PICK / PLACE)

```bash
ros2 launch ur5e_manoeuvring gripper_attached.launch.py
```

> **Note:** The gripper launch is only needed for commands that open or close the gripper (PICK, PLACE).
> MOVE_HOME, BOB, and FINGER_WAG work without it.

## Optional — Bounding box

```bash
ros2 run ur5e_manoeuvring bounding_box_node
```

## Test commands

Send a manipulation goal to the bridge via:

```bash
ros2 topic pub --once /checkers/manipulation_goal std_msgs/msg/String \
  "{data: '<json>'}"
```

| Command | JSON |
|---|---|
| Move home | `{"command_id":"test-1","command_type":"MOVE_HOME","square":0,"row":0,"col":0,"is_king_stack":false,"metadata":{}}` |
| Bob over (3,2) | `{"command_id":"test-2","command_type":"BOB","square":0,"row":3,"col":2,"is_king_stack":false,"metadata":{}}` |
| Finger wag | `{"command_id":"test-3","command_type":"FINGER_WAG","square":0,"row":0,"col":0,"is_king_stack":false,"metadata":{}}` |
| Pick (0,1) | `{"command_id":"test-4","command_type":"PICK","square":1,"row":0,"col":1,"is_king_stack":false,"metadata":{}}` |
| Place (1,5) | `{"command_id":"test-5","command_type":"PLACE","square":5,"row":1,"col":5,"is_king_stack":false,"metadata":{}}` |

Monitor progress:

```bash
ros2 topic echo /ur5e_motion_status
```

Expected sequence: `RECEIVED_GOAL` → `IK_REQUEST_SENT` → `IK_SUCCESS` → `SENDING_ARM_GOAL` → `ARM_GOAL_ACCEPTED` → `ARM_MOVE_DONE`
