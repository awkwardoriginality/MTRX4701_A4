#!/usr/bin/env python3

import json

from click import command
import rclpy
import subprocess
from rclpy.node import Node
import random
import os

from std_msgs.msg import String, Bool


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__("robot_controller_node")

        self.arm_pub = self.create_publisher(
            String,
            "/checkerboard_target",
            10,
        )

        self.gripper_pub = self.create_publisher(
            String,
            "/gripper_command",
            10,
        )

        self.robot_done_pub = self.create_publisher(
            Bool,
            "/game/robot_done",
            10,
        )

        self.command_sub = self.create_subscription(
            String,
            "/robot_move",
            self.robot_move_callback,
            10,
        )

        self.arm_done_sub = self.create_subscription(
            Bool,
            "/arm_motion_done",
            self.arm_done_callback,
            10,
        )

        self.gripper_done_sub = self.create_subscription(
            Bool,
            "/gripper_done",
            self.gripper_done_callback,
            10,
        )

        self.busy = False
        self.sequence = []
        self.step_index = 0
        self.waiting_for = None

        self.get_logger().info("Robot controller ready")
        self.get_logger().info(
            "Publish move as: {'from':[0,7], 'to':[1,7]}"
        )

    def robot_move_callback(self, msg):
        if self.busy:
            self.get_logger().warn("Robot is already running a sequence")
            return

        try:
            data = json.loads(msg.data)

            from_row = int(data["from"][0])
            from_col = int(data["from"][1])
            to_row = int(data["to"][0])
            to_col = int(data["to"][1])
            path = data.get("path", [])
            captured = data.get("captured", [])

        except Exception as e:
            self.get_logger().error(
                f"Bad command. Use JSON like {{'from':[0,7], 'to':[1,7]}}. Error: {e}"
            )
            return

        self.sequence = [
            ("arm", f"{from_row} {from_col}"),
            ("gripper", "close"),
            ("arm", "lift"),
        ]

        for mid in path:
            mid_row, mid_col = int(mid[0]), int(mid[1])
            self.sequence += [
                ("arm", f"{mid_row} {mid_col}"),
                ("arm", "lift"),
            ]

        self.sequence += [
            ("arm", f"{to_row} {to_col}"),
            ("gripper", "open"),
            ("arm", "lift"),
        ]

        for i, cap in enumerate(captured):
            cap_row, cap_col = int(cap[0]), int(cap[1])

            self.sequence += [
                ("arm", f"{cap_row} {cap_col}"),
            ]

            if i == 0:
                self.sequence.append(("wav", "single_take"))
            else:
                self.sequence.append(("wav", "multiple_take"))

            self.sequence += [
                ("gripper", "close"),
                ("arm", "lift"),
                ("arm", "discard"),
                ("wav", "discard"),
                ("gripper", "open"),
            ]

        self.sequence.append(("arm", "home"))

        self.busy = True
        self.step_index = 0
        self.waiting_for = None

        self.get_logger().info(
            f"Starting move from [{from_row}, {from_col}] to [{to_row}, {to_col}]"
        )

        self.run_next_step()

    def run_next_step(self):
        if self.step_index >= len(self.sequence):
            self.get_logger().info("Full robot sequence complete")
            self.busy = False
            self.waiting_for = None
            self.publish_robot_done(True)
            return

        command_type, command = self.sequence[self.step_index]

        self.get_logger().info(
            f"Step {self.step_index + 1}/{len(self.sequence)}: {command_type} -> {command}"
        )

        if command_type == "arm":
            msg = String()
            msg.data = command
            self.arm_pub.publish(msg)
            self.waiting_for = "arm"

        elif command_type == "gripper":
            msg = String()
            msg.data = command
            self.gripper_pub.publish(msg)
            self.waiting_for = "gripper"
        
        elif command_type == "wav":
            if command == "single_take":
                self.play_wav("/home/eashan-garg/glados/ill_take_that.wav")

            elif command == "multiple_take":
                self.play_wav("/home/eashan-garg/glados/and_that_one_too.wav")

            elif command == "discard":
                self.play_wav("/home/eashan-garg/glados/trash.wav")

            self.step_index += 1
            self.run_next_step()
            return

    def arm_done_callback(self, msg):
        if not self.busy:
            return

        if self.waiting_for != "arm":
            return

        if not msg.data:
            self.get_logger().error("Arm motion failed. Sequence stopped.")
            self.busy = False
            self.waiting_for = None
            self.publish_robot_done(False)
            return

        self.get_logger().info("Arm motion done")

        self.step_index += 1
        self.waiting_for = None
        self.run_next_step()

    def gripper_done_callback(self, msg):
        if not self.busy:
            return

        if self.waiting_for != "gripper":
            return
        if not msg.data:
            self.get_logger().warn("Ignoring /gripper_done = False")
            return

        self.get_logger().info("Gripper command done")

        self.step_index += 1
        self.waiting_for = None
        self.run_next_step()

    def publish_robot_done(self, success):
        msg = Bool()
        msg.data = bool(success)
        self.robot_done_pub.publish(msg)

        self.get_logger().info(
            f"Published /game/robot_done = {success}"
        )
    
    def play_wav(self, wav_path):
        subprocess.Popen(["aplay", wav_path])


def main(args=None):
    rclpy.init(args=args)

    node = RobotControllerNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()