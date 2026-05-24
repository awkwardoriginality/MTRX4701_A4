#!/usr/bin/env python3

import json
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest,
    Constraints,
    PositionConstraint,
    OrientationConstraint,
    JointConstraint,
    BoundingVolume,
    PlanningOptions,
)
from shape_msgs.msg import SolidPrimitive


class CheckerboardPoseNode(Node):
    def __init__(self):
        super().__init__("checkerboard_pose_node")

        self.declare_parameter("frame_id", "base_link")
        self.declare_parameter("origin_x", -0.60)
        self.declare_parameter("origin_y", 0.25)
        self.declare_parameter("origin_z", 0.00)
        self.declare_parameter("square_size", 0.05)
        self.declare_parameter("rotation_steps", 3)

        self.declare_parameter("hover_height", 0.10)
        self.declare_parameter("descent_height", 0.05)
        self.declare_parameter("lift_height", 0.20)

        self.declare_parameter("planning_group", "ur_manipulator")
        self.declare_parameter("eef_link", "tool0")
        self.declare_parameter("velocity_scaling", 0.15)
        self.declare_parameter("acceleration_scaling", 0.15)

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.home_joint_deg = [
            -180.00,
            -80.00,
            -142.00,
            -227.00,
            -90.00,
            90.00,
        ]

        self.home_waypoint_joint_deg = [
            -90.00,
            -80.00,
            -142.00,
            -227.00,
            -90.00,
            90.00,
        ]

        self.board_waypoint_joint_deg = [
            -90.0,
            -113.0,
            -102.0,
            -234.0,
            -90.0,
            90.0,
        ]

        self.move_client = ActionClient(self, MoveGroup, "/move_action")

        self.motion_done_pub = self.create_publisher(
            Bool,
            "/arm_motion_done",
            10,
        )

        self.retry_count = 0
        self.max_retries = 5

        self.last_goal_type = None
        self.last_pose_target = None
        self.last_joint_target = None

        self.pending_board_target = None

        self.hover_target = None
        self.descent_target = None
        self.current_stage = None
        self.current_xyz = None

        self.sub = self.create_subscription(
            String,
            "/checkerboard_target",
            self.target_callback,
            10,
        )

        self.startup_timer = self.create_timer(1.0, self.startup_home_once)

        self.get_logger().info(
            "Ready. Publish 'row col', JSON {'row':r,'col':c}, 'lift', or 'home'"
        )

    def publish_motion_done(self, success):
        msg = Bool()
        msg.data = bool(success)
        self.motion_done_pub.publish(msg)

        self.get_logger().info(
            f"Published /arm_motion_done = {success}"
        )

    def startup_home_once(self):
        self.startup_timer.cancel()
        self.get_logger().info("Startup: moving directly to HOME")
        self.send_joint_goal(self.home_joint_deg, "startup_home")

    def rotate_square(self, row, col, rotation):
        rotation = rotation % 4
        if rotation == 0:
            return row, col
        if rotation == 1:
            return col, 7 - row
        if rotation == 2:
            return 7 - row, 7 - col
        if rotation == 3:
            return 7 - col, row
        return row, col

    def rpy_to_quat(self, roll, pitch, yaw):
        cr = math.cos(roll / 2.0)
        sr = math.sin(roll / 2.0)
        cp = math.cos(pitch / 2.0)
        sp = math.sin(pitch / 2.0)
        cy = math.cos(yaw / 2.0)
        sy = math.sin(yaw / 2.0)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return qx, qy, qz, qw

    def parse_target(self, msg):
        text = msg.data.strip()

        try:
            data = json.loads(text)
            return int(data["row"]), int(data["col"])
        except Exception:
            parts = text.replace(",", " ").split()
            if len(parts) != 2:
                raise ValueError("Use 'row col', JSON {'row': r, 'col': c}, 'lift', or 'home'")
            return int(parts[0]), int(parts[1])

    def target_callback(self, msg):
        text = msg.data.strip().lower()

        if text in ["home", "upright", "back", "return"]:
            self.get_logger().info("Home command: moving HOME WAYPOINT -> HOME")
            self.reset_motion_state()
            self.send_joint_goal(self.home_waypoint_joint_deg, "waypoint_before_home")
            return

        if text in ["lift", "raise", "up"]:
            self.send_lift_goal()
            return

        try:
            row, col = self.parse_target(msg)
        except Exception as e:
            self.get_logger().error(str(e))
            return

        if row < 0 or row > 7 or col < 0 or col > 7:
            self.get_logger().error("row and col must be from 0 to 7")
            return

        self.pending_board_target = (row, col)
        self.retry_count = 0

        self.get_logger().info(
            f"Board command row={row}, col={col}: moving BOARD WAYPOINT -> square"
        )

        self.send_joint_goal(self.board_waypoint_joint_deg, "waypoint_before_board")

    def prepare_board_motion(self, row, col):
        ox = float(self.get_parameter("origin_x").value)
        oy = float(self.get_parameter("origin_y").value)
        oz = float(self.get_parameter("origin_z").value)
        square_size = float(self.get_parameter("square_size").value)
        rotation = int(self.get_parameter("rotation_steps").value)
        hover_height = float(self.get_parameter("hover_height").value)
        descent_height = float(self.get_parameter("descent_height").value)

        r_rot, c_rot = self.rotate_square(row, col, rotation)

        target_x = ox + c_rot * square_size + square_size / 2.0
        target_y = oy + r_rot * square_size + square_size / 2.0

        hover_z = oz + hover_height
        descend_z = oz + hover_height - descent_height

        if descend_z < oz:
            self.get_logger().error("descent_height is too large")
            return

        self.hover_target = (target_x, target_y, hover_z)
        self.descent_target = (target_x, target_y, descend_z)

        self.get_logger().info(
            f"Hover target:   x={target_x:.3f}, y={target_y:.3f}, z={hover_z:.3f}"
        )
        self.get_logger().info(
            f"Descend target: x={target_x:.3f}, y={target_y:.3f}, z={descend_z:.3f}"
        )

        self.current_stage = "hover"
        self.send_moveit_pose_goal(target_x, target_y, hover_z)

    def send_lift_goal(self):
        if self.current_xyz is None:
            self.get_logger().error("No current pose stored yet. Move to a square first.")
            return

        x, y, z = self.current_xyz
        lift_height = float(self.get_parameter("lift_height").value)
        lifted_z = z + lift_height

        self.get_logger().info(
            f"Lifting vertically by {lift_height:.3f} m to z={lifted_z:.3f}"
        )

        self.retry_count = 0
        self.current_stage = "lift"
        self.send_moveit_pose_goal(x, y, lifted_z)

    def reset_motion_state(self):
        self.retry_count = 0
        self.last_goal_type = None
        self.last_pose_target = None
        self.last_joint_target = None
        self.pending_board_target = None
        self.hover_target = None
        self.descent_target = None
        self.current_stage = None

    def send_joint_goal(self, joint_deg, stage_name):
        constraints = Constraints()
        constraints.name = stage_name

        joint_rad = [math.radians(v) for v in joint_deg]

        for name, pos in zip(self.joint_names, joint_rad):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

        self.current_stage = stage_name
        self.last_goal_type = "joint"
        self.last_joint_target = joint_deg

        self.send_moveit_request(constraints)

    def send_moveit_pose_goal(self, x, y, z):
        self.current_xyz = (x, y, z)
        self.last_goal_type = "pose"
        self.last_pose_target = (x, y, z)

        frame_id = self.get_parameter("frame_id").value
        eef_link = self.get_parameter("eef_link").value

        qx, qy, qz, qw = self.rpy_to_quat(0.0, math.pi, 0.0)

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.003]

        bv = BoundingVolume()
        bv.primitives = [sphere]
        bv.primitive_poses = [pose.pose]

        pc = PositionConstraint()
        pc.header.frame_id = frame_id
        pc.link_name = eef_link
        pc.constraint_region = bv
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header.frame_id = frame_id
        oc.link_name = eef_link
        oc.orientation = pose.pose.orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 3.14
        oc.weight = 1.0

        constraints = Constraints()
        constraints.name = "checkerboard_pose_goal"
        constraints.position_constraints = [pc]
        constraints.orientation_constraints = [oc]

        self.send_moveit_request(constraints)

    def send_moveit_request(self, constraints):
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveIt /move_action server not available")
            return

        request = MotionPlanRequest()
        request.group_name = self.get_parameter("planning_group").value
        request.num_planning_attempts = 20
        request.allowed_planning_time = 8.0
        request.max_velocity_scaling_factor = float(
            self.get_parameter("velocity_scaling").value
        )
        request.max_acceleration_scaling_factor = float(
            self.get_parameter("acceleration_scaling").value
        )
        request.goal_constraints = [constraints]

        options = PlanningOptions()
        options.plan_only = False
        options.look_around = False
        options.replan = True
        options.replan_attempts = 5

        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options = options

        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("MoveIt goal rejected")
            return

        self.get_logger().info("MoveIt goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        error_code = result.error_code.val

        if error_code == 1:
            self.get_logger().info(f"Move completed successfully: {self.current_stage}")
            self.retry_count = 0

            if self.current_stage == "waypoint_before_home":
                self.get_logger().info("Home waypoint reached. Moving to HOME.")
                self.send_joint_goal(self.home_joint_deg, "home")
                return

            if self.current_stage == "waypoint_before_board":
                if self.pending_board_target is not None:
                    row, col = self.pending_board_target
                    self.pending_board_target = None
                    self.prepare_board_motion(row, col)
                return

            if self.current_stage == "hover" and self.descent_target is not None:
                x, y, z = self.descent_target
                self.current_stage = "descent"

                self.get_logger().info(
                    f"Now descending vertically to: x={x:.3f}, y={y:.3f}, z={z:.3f}"
                )

                self.send_moveit_pose_goal(x, y, z)
                return

            self.publish_motion_done(True)
            return

        self.get_logger().error(f"MoveIt failed, error code: {error_code}")

        if self.retry_count < self.max_retries:
            self.retry_count += 1
            self.get_logger().warn(
                f"Retrying {self.current_stage} ({self.retry_count}/{self.max_retries})"
            )

            if self.last_goal_type == "joint" and self.last_joint_target is not None:
                self.send_joint_goal(self.last_joint_target, self.current_stage)
                return

            if self.last_goal_type == "pose" and self.last_pose_target is not None:
                x, y, z = self.last_pose_target
                self.send_moveit_pose_goal(x, y, z)
                return

        self.get_logger().error("Maximum retries reached")
        self.retry_count = 0
        self.publish_motion_done(False)


def main(args=None):
    rclpy.init(args=args)
    node = CheckerboardPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()