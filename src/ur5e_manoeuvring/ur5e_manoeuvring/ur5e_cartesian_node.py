#!/usr/bin/env python3
"""
ur5e_cartesian_node — Executes Cartesian end-effector goals on the UR5e.

Planning hierarchy (in order of preference):
  1. GetCartesianPath  — straight-line, collision-checked path for short moves.
  2. MoveGroup/OMPL   — full collision-aware sampling-based planner for large
                         transit moves where a straight Cartesian line fails.

Only GetCartesianPath requires a straight Cartesian line; MoveGroup (used as
fallback) can plan arbitrary collision-free joint-space paths, so it correctly
handles the initial move from the upright start position without the arm ever
sweeping through the floor mid-arc.
"""

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped

from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import MoveGroup as MoveGroupAction
from moveit_msgs.msg import (
    CollisionObject, PlanningScene, RobotState,
    MotionPlanRequest, Constraints,
    PositionConstraint, OrientationConstraint,
    BoundingVolume, WorkspaceParameters,
    MoveItErrorCodes,
)
from shape_msgs.msg import SolidPrimitive

from trajectory_msgs.msg import JointTrajectory
from control_msgs.action import FollowJointTrajectory, GripperCommand

from tf_transformations import quaternion_from_euler

_FLOOR_Z_MIN = 0.02   # hard-reject any goal below this (metres, tool0 frame)


class UR5eCartesianNode(Node):
    def __init__(self):
        super().__init__("ur5e_cartesian_node")

        self.joint_names = [
            "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
            "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint",
        ]

        self.latest_joint_state = None
        self.pending_gripper    = 0.025
        self.pending_move_time  = 3.0
        self._pending_pose: dict = {}

        self.create_subscription(JointState, "/joint_states",
                                 self.joint_state_callback, 10)
        self.create_subscription(String, "/ur5e_cartesian_goal",
                                 self.goal_callback, 10)

        self.status_pub = self.create_publisher(String, "/ur5e_motion_status", 10)

        # ── Planners ────────────────────────────────────────────────────────
        # Primary: straight-line Cartesian path (good for short approach moves).
        self.cartesian_path_client = self.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )

        # Fallback: full OMPL motion planning via MoveGroup.
        # MoveGroup checks the entire path for collisions at every sample —
        # the only planner that reliably prevents mid-arc floor penetration.
        self.move_group_client = ActionClient(
            self, MoveGroupAction, "/move_action"
        )

        # Arm trajectory executor (used by the Cartesian path route only;
        # MoveGroup executes its own trajectory internally).
        self.arm_client = ActionClient(
            self, FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )

        self.gripper_client = ActionClient(
            self, GripperCommand,
            "/gripper/gripper_action_controller/gripper_cmd",
        )

        self._scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self._floor_timer = self.create_timer(2.0, self._add_floor_once)

        self.get_logger().info("UR5e Cartesian node ready.")
        self.get_logger().info("Publish JSON goals to /ur5e_cartesian_goal")

    # ── Scene setup ──────────────────────────────────────────────────────────

    def _add_floor_once(self):
        self._floor_timer.cancel()
        box = SolidPrimitive(type=SolidPrimitive.BOX, dimensions=[3.0, 3.0, 0.01])
        box_pose = Pose()
        box_pose.position.z = -0.005   # top face at z = 0
        box_pose.orientation.w = 1.0

        floor = CollisionObject()
        floor.header.frame_id = "base_link"
        floor.header.stamp = self.get_clock().now().to_msg()
        floor.id = "floor"
        floor.operation = CollisionObject.ADD
        floor.primitives = [box]
        floor.primitive_poses = [box_pose]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [floor]
        self._scene_pub.publish(scene)
        self.get_logger().info("Floor collision object published to planning scene.")

    # ── Inbound goal ─────────────────────────────────────────────────────────

    def joint_state_callback(self, msg):
        self.latest_joint_state = msg

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def goal_callback(self, msg):
        try:
            data = json.loads(msg.data)
            x     = float(data["x"])
            y     = float(data["y"])
            z     = float(data["z"])
            roll  = float(data.get("roll",  math.pi))
            pitch = float(data.get("pitch", 0.0))
            yaw   = float(data.get("yaw",   0.0))

            self.pending_gripper   = float(data.get("gripper", 0.025))
            self.pending_move_time = float(data.get("time",    3.0))

            if z < _FLOOR_Z_MIN:
                self.publish_status(
                    f"GOAL_BELOW_FLOOR: z={z:.3f} m (min={_FLOOR_Z_MIN} m)"
                )
                self.publish_status("IK_FAILED")
                return

            self._pending_pose = dict(x=x, y=y, z=z,
                                      roll=roll, pitch=pitch, yaw=yaw)
            self.publish_status("RECEIVED_GOAL")

            req = self._make_cartesian_path_request(x, y, z, roll, pitch, yaw)
            if req is None:
                return

            future = self.cartesian_path_client.call_async(req)
            future.add_done_callback(self._cartesian_path_cb)
            self.publish_status("IK_REQUEST_SENT")

        except Exception as e:
            self.publish_status(f"GOAL_ERROR: {e}")

    # ── Primary: GetCartesianPath ─────────────────────────────────────────────

    def _make_cartesian_path_request(self, x, y, z, roll, pitch, yaw):
        if self.latest_joint_state is None:
            self.publish_status("NO_JOINT_STATE_YET")
            self.publish_status("IK_REQUEST_FAILED")
            return None

        if not self.cartesian_path_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn(
                "/compute_cartesian_path unavailable — falling back to MoveGroup"
            )
            self._send_movegroup_request()
            return None

        robot_state = RobotState()
        robot_state.joint_state = self.latest_joint_state

        req = GetCartesianPath.Request()
        req.header.frame_id  = "base_link"
        req.header.stamp     = self.get_clock().now().to_msg()
        req.start_state      = robot_state
        req.group_name       = "ur_manipulator"
        req.link_name        = "tool0"
        req.waypoints        = [self._build_pose(x, y, z, roll, pitch, yaw)]
        req.max_step         = 0.01        # 1 cm resolution
        req.jump_threshold   = 0.0
        req.avoid_collisions = True
        return req

    def _cartesian_path_cb(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().warn(
                f"GetCartesianPath call failed ({e}) — falling back to MoveGroup"
            )
            self._send_movegroup_request()
            return

        if response is None or response.error_code.val != 1:
            code = response.error_code.val if response else "None"
            self.get_logger().warn(
                f"GetCartesianPath error {code} — falling back to MoveGroup"
            )
            self._send_movegroup_request()
            return

        if response.fraction < 0.99:
            # Straight-line Cartesian path infeasible — typical for large transit
            # moves from arbitrary starting positions.  MoveGroup's OMPL planner
            # can find a valid path without a straight-line constraint.
            self.get_logger().info(
                f"Cartesian path only {response.fraction:.0%} feasible — "
                "falling back to MoveGroup for collision-safe planning"
            )
            self._send_movegroup_request()
            return

        # Path is fully collision-checked — execute directly.
        trajectory = response.solution.joint_trajectory
        points = trajectory.points
        if points and points[-1].time_from_start.sec == 0 and \
                points[-1].time_from_start.nanosec == 0:
            n = len(points)
            for i, pt in enumerate(points):
                t = self.pending_move_time * (i + 1) / n
                pt.time_from_start.sec      = int(t)
                pt.time_from_start.nanosec  = int((t % 1) * 1e9)

        self.get_logger().info(
            f"Cartesian path OK ({len(points)} pts, 100%) — executing"
        )
        self.publish_status("IK_SUCCESS")
        self._send_arm_trajectory(trajectory)
        self.send_gripper_goal(self.pending_gripper)

    # ── Fallback: MoveGroup / OMPL ───────────────────────────────────────────
    #
    # MoveGroup plans a collision-free path in joint space using OMPL.  Unlike
    # GetCartesianPath (straight Cartesian only) or IK + FollowJointTrajectory
    # (path never checked), MoveGroup verifies every sample along the path
    # against the planning scene — including the floor slab.  This is the only
    # planner that reliably prevents the arm from sweeping through the floor
    # on large transit moves from arbitrary start configurations.

    def _send_movegroup_request(self):
        p = self._pending_pose
        if not p:
            self.publish_status("IK_REQUEST_FAILED")
            return

        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.publish_status("IK_SERVICE_NOT_AVAILABLE")
            return

        q = quaternion_from_euler(p["roll"], p["pitch"], p["yaw"])

        # Position constraint: a small tolerance box around the target position.
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.005, 0.005, 0.005]   # 5 mm goal tolerance

        box_pose = Pose()
        box_pose.position.x = p["x"]
        box_pose.position.y = p["y"]
        box_pose.position.z = p["z"]
        box_pose.orientation.w = 1.0

        bvol = BoundingVolume()
        bvol.primitives      = [box]
        bvol.primitive_poses = [box_pose]

        pos_c = PositionConstraint()
        pos_c.header.frame_id = "base_link"
        pos_c.header.stamp    = self.get_clock().now().to_msg()
        pos_c.link_name       = "tool0"
        pos_c.constraint_region = bvol
        pos_c.weight          = 1.0

        # Orientation constraint: gripper must point down (with small tolerance).
        orient_c = OrientationConstraint()
        orient_c.header.frame_id = "base_link"
        orient_c.header.stamp    = self.get_clock().now().to_msg()
        orient_c.link_name       = "tool0"
        orient_c.orientation.x   = q[0]
        orient_c.orientation.y   = q[1]
        orient_c.orientation.z   = q[2]
        orient_c.orientation.w   = q[3]
        orient_c.absolute_x_axis_tolerance = 0.05
        orient_c.absolute_y_axis_tolerance = 0.05
        orient_c.absolute_z_axis_tolerance = 0.05
        orient_c.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints    = [pos_c]
        goal_constraints.orientation_constraints = [orient_c]

        # Workspace limits keep the OMPL sampler efficient.
        ws = WorkspaceParameters()
        ws.header.frame_id = "base_link"
        ws.min_corner.x = -1.5; ws.min_corner.y = -1.5; ws.min_corner.z = -0.05
        ws.max_corner.x =  1.5; ws.max_corner.y =  1.5; ws.max_corner.z =  2.0

        plan_req = MotionPlanRequest()
        plan_req.group_name                   = "ur_manipulator"
        plan_req.num_planning_attempts        = 5
        plan_req.allowed_planning_time        = 5.0
        plan_req.max_velocity_scaling_factor  = 0.3
        plan_req.max_acceleration_scaling_factor = 0.3
        plan_req.goal_constraints             = [goal_constraints]
        plan_req.workspace_parameters         = ws

        if self.latest_joint_state is not None:
            start = RobotState()
            start.joint_state = self.latest_joint_state
            plan_req.start_state = start

        from moveit_msgs.msg import PlanningOptions
        options = PlanningOptions()
        options.plan_only = False   # plan AND execute

        goal = MoveGroupAction.Goal()
        goal.request         = plan_req
        goal.planning_options = options

        self.get_logger().info(
            f"MoveGroup: planning to ({p['x']:.3f}, {p['y']:.3f}, {p['z']:.3f}) …"
        )
        self.publish_status("SENDING_ARM_GOAL")
        future = self.move_group_client.send_goal_async(goal)
        future.add_done_callback(self._movegroup_goal_cb)

    def _movegroup_goal_cb(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.publish_status(f"ARM_GOAL_SEND_FAILED: {e}")
            return

        if not goal_handle.accepted:
            self.publish_status("ARM_GOAL_REJECTED")
            return

        self.publish_status("ARM_GOAL_ACCEPTED")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._movegroup_result_cb)

    def _movegroup_result_cb(self, future):
        try:
            result = future.result().result
        except Exception as e:
            self.publish_status(f"ARM_RESULT_FAILED: {e}")
            return

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("MoveGroup execution complete.")
            self.send_gripper_goal(self.pending_gripper)
            self.publish_status("ARM_MOVE_DONE")
        else:
            self.get_logger().error(
                f"MoveGroup failed: error_code={result.error_code.val}"
            )
            self.publish_status(f"ARM_MOVE_FAILED: {result.error_code.val}")

    # ── Arm trajectory sender (Cartesian path route) ──────────────────────────

    def _send_arm_trajectory(self, trajectory: JointTrajectory):
        if not self.arm_client.wait_for_server(timeout_sec=2.0):
            self.publish_status("ARM_ACTION_NOT_AVAILABLE")
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self.publish_status("SENDING_ARM_GOAL")
        future = self.arm_client.send_goal_async(goal)
        future.add_done_callback(self.arm_goal_response_callback)

    def arm_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.publish_status(f"ARM_GOAL_SEND_FAILED: {e}")
            return
        if not goal_handle.accepted:
            self.publish_status("ARM_GOAL_REJECTED")
            return
        self.publish_status("ARM_GOAL_ACCEPTED")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.arm_result_callback)

    def arm_result_callback(self, future):
        try:
            result = future.result().result
        except Exception as e:
            self.publish_status(f"ARM_RESULT_FAILED: {e}")
            return
        if result.error_code == 0:
            self.publish_status("ARM_MOVE_DONE")
        else:
            self.publish_status(f"ARM_MOVE_FAILED: {result.error_code}")

    # ── Gripper ──────────────────────────────────────────────────────────────

    def send_gripper_goal(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.publish_status("GRIPPER_ACTION_NOT_AVAILABLE")
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        self.publish_status("SENDING_GRIPPER_GOAL")
        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(self.gripper_goal_response_callback)

    def gripper_goal_response_callback(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.publish_status(f"GRIPPER_GOAL_SEND_FAILED: {e}")
            return
        if not goal_handle.accepted:
            self.publish_status("GRIPPER_GOAL_REJECTED")
            return
        self.publish_status("GRIPPER_GOAL_ACCEPTED")

    # ── Helper ───────────────────────────────────────────────────────────────

    def _build_pose(self, x, y, z, roll, pitch, yaw) -> Pose:
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        q = quaternion_from_euler(roll, pitch, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = UR5eCartesianNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
