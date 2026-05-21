#!/usr/bin/env python3

import json
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
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

        self.declare_parameter("planning_group", "ur_manipulator")
        self.declare_parameter("eef_link", "tool0")
        self.declare_parameter("velocity_scaling", 0.25)
        self.declare_parameter("acceleration_scaling", 0.25)

        self.move_client = ActionClient(self, MoveGroup, "/move_action")

        self.retry_count = 0
        self.max_retries = 5
        self.last_target = None

        self.sub = self.create_subscription(
            String,
            "/checkerboard_target",
            self.target_callback,
            10,
        )

        self.get_logger().info(
            "Checkerboard pose node ready. Publish 'row col' or 'home' to /checkerboard_target"
        )

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
                raise ValueError("Use either 'row col', JSON {'row': r, 'col': c}, or 'home'")
            return int(parts[0]), int(parts[1])

    def target_callback(self, msg):
        text = msg.data.strip().lower()

        if text in ["home", "upright", "back", "return"]:
            self.get_logger().info("Returning to upright/home pose")
            self.last_target = None
            self.retry_count = 0
            self.send_home_goal()
            return

        try:
            row, col = self.parse_target(msg)
        except Exception as e:
            self.get_logger().error(str(e))
            return

        if row < 0 or row > 7 or col < 0 or col > 7:
            self.get_logger().error("row and col must be from 0 to 7")
            return

        ox = float(self.get_parameter("origin_x").value)
        oy = float(self.get_parameter("origin_y").value)
        oz = float(self.get_parameter("origin_z").value)
        square_size = float(self.get_parameter("square_size").value)
        rotation = int(self.get_parameter("rotation_steps").value)
        hover_height = float(self.get_parameter("hover_height").value)

        r_rot, c_rot = self.rotate_square(row, col, rotation)

        target_x = ox + c_rot * square_size + square_size / 2.0
        target_y = oy + r_rot * square_size + square_size / 2.0
        target_z = oz + hover_height

        self.last_target = (target_x, target_y, target_z)
        self.retry_count = 0

        self.get_logger().info(
            f"Target square row={row}, col={col} -> "
            f"x={target_x:.3f}, y={target_y:.3f}, z={target_z:.3f}"
        )

        self.send_moveit_pose_goal(target_x, target_y, target_z)

    def send_moveit_pose_goal(self, x, y, z):
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveIt /move_action server not available")
            return

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
        sphere.dimensions = [0.01]

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

        # Keep strict vertical orientation
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 3.14
        oc.weight = 1.0

        constraints = Constraints()
        constraints.name = "checkerboard_hover_goal"
        constraints.position_constraints = [pc]
        constraints.orientation_constraints = [oc]

        self.send_moveit_request(constraints)

    def send_home_goal(self):
        joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        joint_positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

        constraints = Constraints()
        constraints.name = "home_upright_goal"

        for name, pos in zip(joint_names, joint_positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)

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
            self.get_logger().info("Move completed successfully")
            self.retry_count = 0
            return

        self.get_logger().error(f"MoveIt failed, error code: {error_code}")

        if self.retry_count < self.max_retries and self.last_target is not None:
            self.retry_count += 1

            self.get_logger().warn(
                f"Retrying same target ({self.retry_count}/{self.max_retries})"
            )

            x, y, z = self.last_target
            self.send_moveit_pose_goal(x, y, z)
            return

        self.get_logger().error("Maximum retries reached")
        self.retry_count = 0


def main(args=None):
    rclpy.init(args=args)
    node = CheckerboardPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()