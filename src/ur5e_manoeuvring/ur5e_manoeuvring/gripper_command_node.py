#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String, Bool
from control_msgs.action import ParallelGripperCommand
from ur_msgs.srv import SetIO


class GripperCommandNode(Node):
    def __init__(self):
        super().__init__("gripper_command_node")

        self.declare_parameter("arm_model", "ur5e")

        self.declare_parameter("open_position", 0.025)
        self.declare_parameter("closed_position", 0.019)

        self.declare_parameter("open_pin", 0)
        self.declare_parameter("close_pin", 1)

        self.arm_model = self.get_parameter("arm_model").value.lower()

        self.sub = self.create_subscription(
            String,
            "/gripper_command",
            self.command_callback,
            10,
        )

        self.done_pub = self.create_publisher(
            Bool,
            "/gripper_done",
            10,
        )

        self.gripper_action_client = None
        self.io_client = None

        if self.arm_model == "ur5e":
            self.gripper_action_client = ActionClient(
                self,
                ParallelGripperCommand,
                "/gripper/gripper_action_controller/gripper_cmd",
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

        # Open gripper on startup
        self.startup_timer = self.create_timer(
            2.0,
            self.startup_open_gripper
        )

        self.get_logger().info(
            "Publish 'open' or 'close' to /gripper_command"
        )

        self.get_logger().info(
            "Publishing gripper completion on /gripper_done"
        )

    def publish_done(self, success=True):
        msg = Bool()
        msg.data = bool(success)
        self.done_pub.publish(msg)

        if success:
            self.get_logger().info("Published /gripper_done = True")
        else:
            self.get_logger().warn("Published /gripper_done = False")

    def command_callback(self, msg):
        command = msg.data.strip().lower()

        self.publish_done(False)

        if command in ["open", "opened"]:
            self.open_gripper()

        elif command in ["close", "closed", "shut"]:
            self.close_gripper()

        else:
            self.get_logger().error(
                "Command must be 'open' or 'close'"
            )
            self.publish_done(False)

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

    def send_gripper_action(self, position):
        if not self.gripper_action_client.wait_for_server(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                "Gripper action server not available"
            )
            self.publish_done(False)
            return

        goal = ParallelGripperCommand.Goal()

        goal.command.name = ["finger_joint"]
        goal.command.position = [float(position)]
        goal.command.effort = [50.0]

        self.get_logger().info(
            f"Sending UR5e gripper position: {position:.3f}"
        )

        future = self.gripper_action_client.send_goal_async(goal)

        future.add_done_callback(
            self.gripper_goal_response_callback
        )

    def gripper_goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Gripper goal rejected")
            self.publish_done(False)
            return

        self.get_logger().info("Gripper goal accepted")

        result_future = goal_handle.get_result_async()

        result_future.add_done_callback(
            self.gripper_result_callback
        )

    def gripper_result_callback(self, future):
        try:
            result = future.result()
            status = result.status

            if status == 4:
                self.get_logger().info(
                    "Gripper command finished successfully"
                )
                self.publish_done(True)
            else:
                self.get_logger().error(
                    f"Gripper command failed with status: {status}"
                )
                self.publish_done(False)

        except Exception as e:
            self.get_logger().error(
                f"Gripper result error: {e}"
            )
            self.publish_done(False)

    def send_io(self, active_pin):
        if not self.io_client.wait_for_service(
            timeout_sec=5.0
        ):
            self.get_logger().error(
                "SetIO service not available"
            )
            self.publish_done(False)
            return

        open_pin = int(
            self.get_parameter("open_pin").value
        )

        close_pin = int(
            self.get_parameter("close_pin").value
        )

        for pin in [open_pin, close_pin]:
            req = SetIO.Request()
            req.fun = 1
            req.pin = pin
            req.state = 0.0
            self.io_client.call_async(req)

        req = SetIO.Request()
        req.fun = 1
        req.pin = active_pin
        req.state = 1.0

        self.get_logger().info(
            f"Setting UR5 IO pin {active_pin} HIGH"
        )

        future = self.io_client.call_async(req)
        future.add_done_callback(self.io_done_callback)

    def io_done_callback(self, future):
        try:
            response = future.result()

            if response.success:
                self.get_logger().info(
                    "UR5 IO gripper command finished"
                )
                self.publish_done(True)
            else:
                self.get_logger().error(
                    "UR5 IO gripper command failed"
                )
                self.publish_done(False)

        except Exception as e:
            self.get_logger().error(
                f"UR5 IO service error: {e}"
            )
            self.publish_done(False)

    def startup_open_gripper(self):
        self.startup_timer.cancel()

        self.get_logger().info(
            "Startup: opening gripper"
        )

        self.open_gripper()


def main(args=None):
    rclpy.init(args=args)

    node = GripperCommandNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()