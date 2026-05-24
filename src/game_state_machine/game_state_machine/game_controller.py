#!/usr/bin/env python3

import json
import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32MultiArray, Bool, String
from .game_engine.board import (
    Board, CB_WHITE, WHITE_MAN, BLACK_MAN, INTERNAL_TO_ROWCOL
)
from .game_engine.search import Search


class GameController(Node):
    def __init__(self):
        super().__init__("game_controller")

        self.declare_parameter("empty_value", 0)
        self.declare_parameter("purple_value", 2)

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

        self.robot_move_pub = self.create_publisher(
            String,
            "/robot_move",
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

        self.blocked = False
        self.robot_done = False

        self.robot_retry_count = 0
        self.max_robot_retries = 2

        self.last_robot_move = None

        self.search = Search(max_time=3.0)

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
        if self.state != "WAIT_ROBOT_DONE":
            return

        if msg.data:
            self.robot_done = True
            self.get_logger().info("Robot done signal received")
        else:
            self.robot_done = False
            self.publish_status("ERROR: Robot controller reported failure.")
            self.state = "ROBOT_MOVE_FAILED"

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)
        self.get_logger().info(text)

    def boards_equal(self, a, b):
        return a == b

    def index_to_row_col(self, index):
        row = index // 8
        col = index % 8
        return row, col

    def row_col_to_index(self, row, col):
        return row * 8 + col

    def in_bounds(self, row, col):
        return 0 <= row < 8 and 0 <= col < 8

    def find_best_purple_move(self, board):
        # The perception board uses the opposite column parity to the engine's
        # dark squares, so we mirror columns (7 - col) on the way in and out.
        engine_flat = [0] * 64
        for idx, v in enumerate(board):
            if v == 0:
                continue
            row = idx // 8
            col = 7 - (idx % 8)
            if v == 1:
                engine_flat[row * 8 + col] = BLACK_MAN
            elif v == 2:
                engine_flat[row * 8 + col] = WHITE_MAN

        engine_board = Board.from_flat64(engine_flat)
        stats = self.search.find_best_move(engine_board, CB_WHITE)

        if stats.best_move is None:
            return None

        move = stats.best_move
        from_row, from_col_eng = INTERNAL_TO_ROWCOL[move.from_sq]
        to_row, to_col_eng = INTERNAL_TO_ROWCOL[move.to_sq]

        captured = []
        for sq in move.captured_squares:
            r, c_eng = INTERNAL_TO_ROWCOL[sq]
            captured.append((r, 7 - c_eng))

        path = []
        for sq in move.path_squares[1:-1]:
            r, c_eng = INTERNAL_TO_ROWCOL[sq]
            path.append((r, 7 - c_eng))

        return (from_row, 7 - from_col_eng), (to_row, 7 - to_col_eng), captured, path

    def publish_robot_move(self):
        move = self.find_best_purple_move(self.board_after_human)

        if move is None:
            self.publish_status(
                "ERROR: No valid move found for robot."
            )
            self.state = "MANUAL_RESET_REQUIRED"
            return

        from_square, to_square, captured, path = move

        self.last_robot_move = move

        msg = String()
        msg.data = json.dumps({
            "from": [from_square[0], from_square[1]],
            "to": [to_square[0], to_square[1]],
            "path": [[r, c] for r, c in path],
            "captured": [[r, c] for r, c in captured],
        })

        self.robot_move_pub.publish(msg)

        self.get_logger().info(
            f"Published robot move: {msg.data}"
        )

    def control_loop(self):
        if self.current_board is None:
            return

        if self.state == "WAIT_INITIAL_CLEAR":

            if self.blocked:
                return

            self.board_before_human = self.current_board.copy()

            self.publish_status("Ready to play. Make your move.")

            self.state = "WAIT_HUMAN_MOVE"
            return

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

        if self.state == "ROBOT_MOVE_START":

            self.robot_done = False

            previous_state = self.state

            self.publish_robot_move()

            if self.state != previous_state:
                return

            self.publish_status("Robot moving.")

            self.state = "WAIT_ROBOT_DONE"
            return

        if self.state == "WAIT_ROBOT_DONE":

            if not self.robot_done:
                return

            self.publish_status(
                "Robot reports move complete. "
                "Waiting for board to clear."
            )

            self.state = "WAIT_BOARD_CLEAR_AFTER_ROBOT"
            return

        if self.state == "WAIT_BOARD_CLEAR_AFTER_ROBOT":

            if self.blocked:
                return

            self.state = "VERIFY_ROBOT_BOARD_CHANGE"
            return

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