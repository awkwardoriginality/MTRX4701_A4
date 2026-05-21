# Robotic Control — Hardware (A→B Move Test)

## IMPORTANT: BEFORE RUNNING ANY COMMAND MAKE SURE BOUNDING BOX AND FLOOR PLANE IS RUNNINg

If the arm that is being used is a UR5, simply change the ur_type to ur5 in commands for UR driver and MoveIt.

## Build

```bash
source /opt/ros/jazzy/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

---

## Terminal 1 - UR driver (real hardware)

```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=<UR5E_IP_ADDRESS> \
  launch_rviz:=true
```

## Terminal 2 - MoveIt

```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  ur_type:=ur5e \
  launch_rviz:=true
```

## Terminal 3 - Gripper

```bash
ros2 launch ur5e_manoeuvring gripper_attached.launch.py
```

## Terminal 4 - Cartesian goal node

```bash
ros2 run ur5e_manoeuvring ur5e_cartesian_node
```

## Terminal 5 - Cartesian bridge

```bash
ros2 run checkers_bot cartesian_bridge
```

## Terminal 6 - Workspace collision boundaries

```bash
ros2 run ur5e_manoeuvring bounding_box_node
```

---

## Test an A→B move

Pick a piece at row 0, col 1:

```bash
ros2 topic pub --once /checkers/manipulation_goal std_msgs/msg/String \
  "{data: '{\"command_id\":\"hw-1\",\"command_type\":\"PICK\",\"square\":1,\"row\":0,\"col\":1,\"is_king_stack\":false,\"metadata\":{}}'}"
```

Place it at row 1, col 5:

```bash
ros2 topic pub --once /checkers/manipulation_goal std_msgs/msg/String \
  "{data: '{\"command_id\":\"hw-2\",\"command_type\":\"PLACE\",\"square\":5,\"row\":1,\"col\":5,\"is_king_stack\":false,\"metadata\":{}}'}"
```

Watch the status stream to confirm each step completes before sending the next command:

```bash
ros2 topic echo /ur5e_motion_status
```

Expected sequence: `RECEIVED_GOAL` → `IK_REQUEST_SENT` → `IK_SUCCESS` → `SENDING_ARM_GOAL` → `ARM_GOAL_ACCEPTED` → `ARM_MOVE_DONE`
