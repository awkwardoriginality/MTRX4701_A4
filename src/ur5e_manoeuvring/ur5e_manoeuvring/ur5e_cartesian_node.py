#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import RobotState

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from control_msgs.action import ParallelGripperCommand

from tf_transformations import quaternion_from_euler


class UR5eCartesianNode(Node):
    def __init__(self):
        super().__init__("ur5e_cartesian_node")

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

        self.latest_joint_state = None
        self.pending_gripper = 0.025
        self.pending_move_time = 3.0

        self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

        self.create_subscription(
            String,
            "/ur5e_cartesian_goal",
            self.goal_callback,
            10,
        )

        self.status_pub = self.create_publisher(
            String,
            "/ur5e_motion_status",
            10,
        )

        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")

        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/scaled_joint_trajectory_controller/follow_joint_trajectory",
        )

        self.gripper_client = ActionClient(
            self,
            ParallelGripperCommand,
            "/gripper/gripper_action_controller/gripper_cmd",
        )

        self.get_logger().info("UR5e Cartesian node ready.")
        self.get_logger().info("Publish JSON goals to /ur5e_cartesian_goal")

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

            x = float(data["x"])
            y = float(data["y"])
            z = float(data["z"])

            roll = float(data.get("roll", math.pi))
            pitch = float(data.get("pitch", 0.0))
            yaw = float(data.get("yaw", 0.0))

            self.pending_gripper = float(data.get("gripper", 0.025))
            self.pending_move_time = float(data.get("time", 3.0))

            self.publish_status("RECEIVED_GOAL")

            request = self.make_ik_request(x, y, z, roll, pitch, yaw)

            if request is None:
                self.publish_status("IK_REQUEST_FAILED")
                return

            future = self.ik_client.call_async(request)
            future.add_done_callback(self.ik_response_callback)

            self.publish_status("IK_REQUEST_SENT")

        except Exception as e:
            self.publish_status(f"GOAL_ERROR: {e}")

    def make_ik_request(self, x, y, z, roll, pitch, yaw):
        if self.latest_joint_state is None:
            self.publish_status("NO_JOINT_STATE_YET")
            return None

        if not self.ik_client.wait_for_service(timeout_sec=2.0):
            self.publish_status("IK_SERVICE_NOT_AVAILABLE")
            return None

        pose = PoseStamped()
        pose.header.frame_id = "base_link"
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        q = quaternion_from_euler(roll, pitch, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        robot_state = RobotState()
        robot_state.joint_state = self.latest_joint_state

        request = GetPositionIK.Request()
        request.ik_request.group_name = "ur_manipulator"
        request.ik_request.robot_state = robot_state
        request.ik_request.pose_stamped = pose
        request.ik_request.ik_link_name = "tool0"
        request.ik_request.timeout = Duration(seconds=2.0).to_msg()
        request.ik_request.avoid_collisions = False

        return request

    def ik_response_callback(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.publish_status(f"IK_SERVICE_CALL_FAILED: {e}")
            return

        if response is None:
            self.publish_status("IK_RESPONSE_NONE")
            return

        if response.error_code.val != 1:
            self.publish_status(f"IK_ERROR_CODE: {response.error_code.val}")
            self.publish_status("IK_FAILED")
            return

        joint_state = response.solution.joint_state
        joint_map = {}

        for name, pos in zip(joint_state.name, joint_state.position):
            joint_map[name] = pos

        positions = []

        for joint in self.joint_names:
            if joint not in joint_map:
                self.publish_status(f"MISSING_IK_JOINT: {joint}")
                return
            positions.append(joint_map[joint])

        self.publish_status("IK_SUCCESS")

        self.send_arm_goal(positions, self.pending_move_time)
        self.send_gripper_goal(self.pending_gripper)

    def send_arm_goal(self, positions, move_time):
        if not self.arm_client.wait_for_server(timeout_sec=2.0):
            self.publish_status("ARM_ACTION_NOT_AVAILABLE")
            return

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = self.joint_names

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(move_time)
        point.time_from_start.nanosec = int((move_time - int(move_time)) * 1e9)

        goal.trajectory.points.append(point)

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

    def send_gripper_goal(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=2.0):
            self.publish_status("GRIPPER_ACTION_NOT_AVAILABLE")
            return

        goal = ParallelGripperCommand.Goal()
        goal.command.position = [float(position)]

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


def main(args=None):
    rclpy.init(args=args)
    node = UR5eCartesianNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()