#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String
from control_msgs.action import ParallelGripperCommand
from ur_msgs.srv import SetIO


class GripperCommandNode(Node):
    def __init__(self):
        super().__init__("gripper_command_node")

        # arm_model:
        #   ur5e -> Robotiq/parallel gripper action
        #   ur5  -> UR digital IO pneumatic gripper
        self.declare_parameter("arm_model", "ur5e")

        # UR5e gripper positions
        self.declare_parameter("open_position", 0.025)
        self.declare_parameter("closed_position", 0.0)

        # UR5 IO pins
        self.declare_parameter("open_pin", 0)
        self.declare_parameter("close_pin", 1)

        self.arm_model = (
            self.get_parameter("arm_model").value.lower()
        )

        self.sub = self.create_subscription(
            String,
            "/gripper_command",
            self.command_callback,
            10,
        )

        self.gripper_action_client = None
        self.io_client = None

        if self.arm_model == "ur5e":
            self.gripper_action_client = ActionClient(
                self,
                ParallelGripperCommand,
                "/gripper_action_controller/gripper_cmd",
            )

            self.get_logger().info(
                "Using UR5e gripper action interface"
            )

        elif self.arm_model == "ur5":
            self.io_client = self.create_client(
                SetIO,
                "/io_and_status_controller/set_io",
            )

            self.get_logger().info(
                "Using UR5 digital IO interface"
            )

        else:
            self.get_logger().error(
                "arm_model must be either 'ur5e' or 'ur5'"
            )

        self.get_logger().info(
            "Publish 'open' or 'close' to /gripper_command"
        )

    def command_callback(self, msg):
        command = msg.data.strip().lower()

        if command in ["open", "opened"]:
            self.open_gripper()

        elif command in ["close", "closed", "shut"]:
            self.close_gripper()

        else:
            self.get_logger().error(
                "Command must be 'open' or 'close'"
            )

    # ============================================================
    # OPEN
    # ============================================================

    def open_gripper(self):
        if self.arm_model == "ur5e":

            position = float(
                self.get_parameter("open_position").value
            )

            self.send_gripper_action(position)

        elif self.arm_model == "ur5":

            pin = int(
                self.get_parameter("open_pin").value
            )

            self.send_io(pin)

    # ============================================================
    # CLOSE
    # ============================================================

    def close_gripper(self):
        if self.arm_model == "ur5e":

            position = float(
                self.get_parameter("closed_position").value
            )

            self.send_gripper_action(position)

        elif self.arm_model == "ur5":

            pin = int(
                self.get_parameter("close_pin").value
            )

            self.send_io(pin)

    # ============================================================
    # UR5e ACTION
    # ============================================================

    def send_gripper_action(self, position):

        if not self.gripper_action_client.wait_for_server(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                "Gripper action server not available"
            )
            return

        goal = ParallelGripperCommand.Goal()

        goal.command.position = position
        goal.command.max_effort = 50.0

        self.get_logger().info(
            f"Sending UR5e gripper position: {position:.3f}"
        )

        future = self.gripper_action_client.send_goal_async(
            goal
        )

        future.add_done_callback(
            self.gripper_goal_response_callback
        )

    def gripper_goal_response_callback(self, future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(
                "Gripper goal rejected"
            )
            return

        self.get_logger().info(
            "Gripper goal accepted"
        )

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.gripper_result_callback
        )

    def gripper_result_callback(self, future):

        self.get_logger().info(
            "Gripper command finished"
        )

    # ============================================================
    # UR5 DIGITAL IO
    # ============================================================

    def send_io(self, active_pin):

        if not self.io_client.wait_for_service(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                "SetIO service not available"
            )
            return

        open_pin = int(
            self.get_parameter("open_pin").value
        )

        close_pin = int(
            self.get_parameter("close_pin").value
        )

        # Reset both pins first
        for pin in [open_pin, close_pin]:

            req = SetIO.Request()

            req.fun = 1
            req.pin = pin
            req.state = 0.0

            self.io_client.call_async(req)

        # Activate requested pin
        req = SetIO.Request()

        req.fun = 1
        req.pin = active_pin
        req.state = 1.0

        self.get_logger().info(
            f"Setting UR5 IO pin {active_pin} HIGH"
        )

        self.io_client.call_async(req)


def main(args=None):

    rclpy.init(args=args)

    node = GripperCommandNode()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()