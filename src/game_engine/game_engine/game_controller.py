#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, Bool, String


class GameController(Node):
    def __init__(self):
        super().__init__("game_controller")

        self.board_sub = self.create_subscription(
            Int32MultiArray,
            "/checkers/board_state",
            self.board_callback,
            10
        )

        self.blocked_sub = self.create_subscription(
            Bool,
            "/checkers/board_blocked",
            self.blocked_callback,
            10
        )

        self.robot_done_sub = self.create_subscription(
            Bool,
            "/game/robot_done",
            self.robot_done_callback,
            10
        )

        self.robot_start_pub = self.create_publisher(
            Bool,
            "/game/robot_start",
            10
        )

        self.status_pub = self.create_publisher(
            String,
            "/game/status",
            10
        )

        self.current_board = None

        self.board_before_human = None
        self.board_after_human = None
        self.board_after_robot = None

        self.blocked = True
        self.robot_done = False

        self.robot_retry_count = 0
        self.max_robot_retries = 2

        self.state = "WAIT_INITIAL_CLEAR"

        self.timer = self.create_timer(0.2, self.control_loop)

        self.get_logger().info("Game controller started")

    def board_callback(self, msg):
        if len(msg.data) != 64:
            self.get_logger().warn("Invalid board state length")
            return

        self.current_board = list(msg.data)

    def blocked_callback(self, msg):
        self.blocked = msg.data

    def robot_done_callback(self, msg):
        if msg.data:
            self.robot_done = True
            self.get_logger().info("Robot done signal received")

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def publish_robot_start(self):
        msg = Bool()
        msg.data = True
        self.robot_start_pub.publish(msg)
        self.get_logger().info("Published robot start command")

    def boards_equal(self, a, b):
        return a == b

    def control_loop(self):
        if self.current_board is None:
            return

        # --------------------------------------------------
        # 1. WAIT_INITIAL_CLEAR
        #
        # Wait until:
        # - board is visible
        # - no arm/human blocking view
        #
        # Save the initial board state.
        # --------------------------------------------------
        if self.state == "WAIT_INITIAL_CLEAR":

            if self.blocked:
                return

            self.board_before_human = self.current_board.copy()

            self.publish_status("Ready to play. Make your move.")

            self.state = "WAIT_HUMAN_MOVE"
            return

        # --------------------------------------------------
        # 2. WAIT_HUMAN_MOVE
        #
        # Wait for:
        # - board not blocked
        # - current board != previous board
        #
        # This means the human made a move.
        # --------------------------------------------------
        if self.state == "WAIT_HUMAN_MOVE":

            if self.blocked:
                return

            if self.boards_equal(
                self.board_before_human,
                self.current_board
            ):
                return

            self.board_after_human = self.current_board.copy()

            self.publish_status(
                "Human move detected. Robot turn starting."
            )

            self.robot_done = False
            self.robot_retry_count = 0

            self.state = "ROBOT_MOVE_START"
            return

        # --------------------------------------------------
        # 3. ROBOT_MOVE_START
        #
        # Tell the robot node to begin moving.
        #
        # Publishes:
        # /game/robot_start = True
        # --------------------------------------------------
        if self.state == "ROBOT_MOVE_START":

            self.robot_done = False

            self.publish_robot_start()

            self.publish_status("Robot moving.")

            self.state = "WAIT_ROBOT_DONE"
            return

        # --------------------------------------------------
        # 4. WAIT_ROBOT_DONE
        #
        # Wait for robot node to publish:
        #
        # /game/robot_done = True
        #
        # This means robot says motion is complete.
        # --------------------------------------------------
        if self.state == "WAIT_ROBOT_DONE":

            if not self.robot_done:
                return

            self.publish_status(
                "Robot reports move complete. "
                "Waiting for board to clear."
            )

            self.state = "WAIT_BOARD_CLEAR_AFTER_ROBOT"
            return

        # --------------------------------------------------
        # 5. WAIT_BOARD_CLEAR_AFTER_ROBOT
        #
        # Robot may still physically block the board.
        #
        # Wait until:
        # blocked == False
        # --------------------------------------------------
        if self.state == "WAIT_BOARD_CLEAR_AFTER_ROBOT":

            if self.blocked:
                return

            self.state = "VERIFY_ROBOT_BOARD_CHANGE"
            return

        # --------------------------------------------------
        # 6. VERIFY_ROBOT_BOARD_CHANGE
        #
        # Verify robot ACTUALLY changed the board.
        #
        # Compare:
        # board_after_human
        # vs
        # current_board
        #
        # If same:
        # robot failed
        #
        # If different:
        # robot succeeded
        # --------------------------------------------------
        if self.state == "VERIFY_ROBOT_BOARD_CHANGE":

            if self.blocked:
                return

            self.board_after_robot = self.current_board.copy()

            if self.boards_equal(
                self.board_after_human,
                self.board_after_robot
            ):

                self.publish_status(
                    "ERROR: Robot move was not detected "
                    "on the board."
                )

                self.state = "ROBOT_MOVE_FAILED"
                return

            self.publish_status(
                "Robot move verified. "
                "Ready to play. Make your move."
            )

            self.robot_retry_count = 0

            self.board_before_human = self.board_after_robot.copy()

            self.board_after_human = None
            self.board_after_robot = None

            self.robot_done = False

            self.state = "WAIT_HUMAN_MOVE"
            return

        # --------------------------------------------------
        # 7. ROBOT_MOVE_FAILED
        #
        # Robot claimed move completed,
        # but board did not change.
        #
        # Retry robot move.
        # --------------------------------------------------
        if self.state == "ROBOT_MOVE_FAILED":

            if self.robot_retry_count < self.max_robot_retries:

                self.robot_retry_count += 1

                self.publish_status(
                    f"Robot move failed. Retrying "
                    f"{self.robot_retry_count}/"
                    f"{self.max_robot_retries}."
                )

                self.robot_done = False
                self.board_after_robot = None

                self.state = "ROBOT_MOVE_START"
                return

            self.publish_status(
                "ERROR: Robot move failed after "
                "maximum retries. Manual reset required."
            )

            self.state = "MANUAL_RESET_REQUIRED"
            return

        # --------------------------------------------------
        # 8. MANUAL_RESET_REQUIRED
        #
        # System halted.
        #
        # Requires manual intervention/reset.
        # --------------------------------------------------
        if self.state == "MANUAL_RESET_REQUIRED":
            return


def main(args=None):
    rclpy.init(args=args)

    node = GameController()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()