"""
game_manager_node.py — Central ROS2 node orchestrating the checkers game.

This node:
    1. Subscribes to /checkers/board_state (from perception)
    2. Runs the game state machine
    3. Publishes manipulation commands to /checkers/manipulation_cmd
    4. Subscribes to /checkers/manipulation_done (from manipulation node)
    5. Publishes game status to /checkers/game_status

The game manager is the single source of truth for the game state.
All game logic, move validation, and AI planning run here.
"""

from __future__ import annotations
import logging
import json

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Bool, UInt8MultiArray
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

from ..game_engine.board import Board, CB_BLACK, CB_WHITE
from ..state_machine.game_states import (
    GameStateMachine, GameContext, GameState, ManipulationCommand,
)
from ..protocol import (
    BoardStateReport, GameStatus, ManipulationGoal,
    ManipulationResult, RobotInstruction,
)

logger = logging.getLogger(__name__)


class GameManagerNode:
    """ROS2 node managing the checkers game state machine.

    When running with ROS2, this wraps the state machine in a ROS2 node
    with proper topics and timers. Without ROS2 (for testing), it can
    be used standalone.
    """

    def __init__(
        self,
        search_time: float = 5.0,
        human_colour: int = CB_BLACK,
        use_ros2: bool = True,
    ):
        self.use_ros2 = use_ros2 and HAS_ROS2
        self.search_time = search_time
        self.human_colour = human_colour

        # Pending manipulation commands
        self._pending_commands: list[ManipulationCommand] = []
        self._stable_board_data: list[int] | None = None
        self._stable_board_count = 0
        self.required_stable_frames = 2
        self._command_counter = 0
        self._board_blocked = False
        self._last_internal_board: list[int] | None = None

        if self.use_ros2:
            self._init_ros2()
        else:
            logger.info("Running without ROS2 — standalone mode")
            self._configure_game(search_time, human_colour)

    def _configure_game(self, search_time: float, human_colour: int):
        """Create the state machine and context using the effective configuration."""
        self.search_time = search_time
        self.human_colour = human_colour

        self.state_machine = GameStateMachine(
            search_time=search_time,
            human_colour=human_colour,
        )

        self.ctx = GameContext()
        self.ctx.human_colour = human_colour
        self.ctx.robot_colour = CB_WHITE if human_colour == CB_BLACK else CB_BLACK

        self.state_machine.set_callbacks(
            on_state_change=self._on_state_change,
            on_manipulation_cmd=self._on_manipulation_cmd,
        )

    def _init_ros2(self):
        """Initialize ROS2 node, publishers, subscribers, timers."""
        if not HAS_ROS2:
            return

        rclpy.init()
        self.node = Node('game_manager')
        self.node.declare_parameter('search_time', self.search_time)
        self.node.declare_parameter('human_colour', self.human_colour)

        effective_search_time = float(self.node.get_parameter('search_time').value)
        effective_human_colour = int(self.node.get_parameter('human_colour').value)
        self._configure_game(effective_search_time, effective_human_colour)

        # ─── Publishers ──────────────────────────────────────────────────
        # Game status (state machine state + board summary)
        self.status_pub = self.node.create_publisher(
            String, '/checkers/game_status', 10
        )

        # Debug / dashboard instruction stream
        self.instruction_pub = self.node.create_publisher(
            String, '/checkers/robot_instruction', 10
        )

        # Manipulation command stream
        self.manip_goal_pub = self.node.create_publisher(
            String, '/checkers/manipulation_goal', 10
        )
        self.manip_legacy_pub = self.node.create_publisher(
            String, '/checkers/manipulation_cmd', 10
        )

        # Board state (internal, for debugging)
        self.board_pub = self.node.create_publisher(
            UInt8MultiArray, '/checkers/internal_board', 10
        )

        # ─── Subscribers ─────────────────────────────────────────────────
        # Perceived board state from perception node
        self.node.create_subscription(
            UInt8MultiArray,
            '/checkers/board_state',
            self._board_state_callback,
            10,
        )
        self.node.create_subscription(
            Bool,
            '/checkers/board_blocked',
            self._board_blocked_callback,
            10,
        )
        self.node.create_subscription(
            String,
            '/checkers/board_state_report',
            self._board_state_report_callback,
            10,
        )

        # Manipulation done signal
        self.node.create_subscription(
            Bool,
            '/checkers/manipulation_done',
            self._manipulation_done_callback,
            10,
        )
        self.node.create_subscription(
            String,
            '/checkers/manipulation_result',
            self._manipulation_result_callback,
            10,
        )

        # ─── Timer ───────────────────────────────────────────────────────
        # Run state machine at 10 Hz
        self.timer = self.node.create_timer(0.1, self._timer_callback)

        logger.info("Game manager ROS2 node initialized")

    def _board_state_callback(self, msg):
        """Handle perceived board state from perception."""
        if self._board_blocked:
            return
        flat = list(msg.data)
        if len(flat) == 64:
            self._accept_stable_board(flat)

    def _board_blocked_callback(self, msg):
        """Handle a coarse board visibility signal for compatibility."""
        self._board_blocked = bool(msg.data)
        if self._board_blocked:
            self._stable_board_count = 0
            self.ctx.perceived_board = None

    def _board_state_report_callback(self, msg):
        """Handle structured board-state reports from perception."""
        try:
            report = BoardStateReport.from_json(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            logger.warning(f"Invalid board state report received: {exc}")
            return

        self.ingest_board_report(report)

    def ingest_board_report(self, report: BoardStateReport):
        """Feed a structured board-state report into the game manager."""
        if len(report.flat64) != 64:
            logger.warning("Ignoring board state report with invalid flat64 length: %s", len(report.flat64))
            return

        self._board_blocked = report.board_blocked
        if report.board_blocked or report.hand_present:
            self._stable_board_count = 0
            self._stable_board_data = None
            self.ctx.perceived_board = None
            return

        self._accept_stable_board(report.flat64, stable_count=report.stable_count)

    def _accept_stable_board(self, flat: list[int], stable_count: int | None = None):
        """Only expose perceived boards to the state machine once they are stable."""
        if len(flat) != 64:
            logger.warning("Ignoring board update with invalid flat64 length: %s", len(flat))
            return

        if self._stable_board_data == flat:
            self._stable_board_count += 1
        else:
            self._stable_board_data = flat[:]
            self._stable_board_count = 1

        effective_stable_count = stable_count or self._stable_board_count
        if effective_stable_count >= self.required_stable_frames:
            self.ctx.perceived_board = Board.from_flat64(flat)

    def _manipulation_done_callback(self, msg):
        """Handle manipulation completion signal."""
        if msg.data:
            self.state_machine.notify_manipulation_done()

    def _manipulation_result_callback(self, msg):
        """Handle structured manipulation results."""
        try:
            result = ManipulationResult.from_json(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            logger.warning(f"Invalid manipulation result received: {exc}")
            return

        self.process_manipulation_result(result)

    def process_manipulation_result(self, result: ManipulationResult):
        """Handle a structured manipulation result in both ROS and standalone tests."""
        if not result.success:
            self.ctx.error_message = result.detail or f"{result.command_type} failed"
            logger.error(f"Manipulation command failed: {self.ctx.error_message}")
            self.state_machine.notify_manipulation_failed(self.ctx, self.ctx.error_message)
            return

        self.state_machine.notify_manipulation_done()

    def _timer_callback(self):
        """Periodic callback — step the state machine."""
        self.state_machine.step(self.ctx)

        internal_flat = self.ctx.board.to_flat64()
        if internal_flat != self._last_internal_board:
            self._last_internal_board = internal_flat[:]
            board_msg = UInt8MultiArray()
            board_msg.data = internal_flat
            self.board_pub.publish(board_msg)

        # Publish status
        if HAS_ROS2:
            status = String()
            status.data = GameStatus(
                state=self.state_machine.state.value,
                move_number=self.ctx.move_number,
                current_turn=_colour_name(self.ctx.current_turn),
                board_summary=self.state_machine.rules.get_board_summary(self.ctx.board),
                legal_move=self.ctx.current_move.to_notation() if self.ctx.current_move else None,
                winner=self.ctx.winner,
                error_message=self.ctx.error_message or None,
            ).to_json()
            self.status_pub.publish(status)

    def _on_state_change(self, old_state: GameState, new_state: GameState):
        """Callback when the state machine transitions."""
        logger.info(f"[GameManager] {old_state.value} → {new_state.value}")
        if self.use_ros2 and HAS_ROS2:
            msg = String()
            msg.data = RobotInstruction(
                state=new_state.value,
                summary=f"{old_state.value} -> {new_state.value}",
                move_number=self.ctx.move_number,
                current_turn=_colour_name(self.ctx.current_turn),
            ).to_json()
            self.instruction_pub.publish(msg)

    def _on_manipulation_cmd(self, cmd: ManipulationCommand):
        """Callback when the state machine generates a manipulation command."""
        self._pending_commands.append(cmd)

        if self.use_ros2 and HAS_ROS2:
            self._command_counter += 1
            goal = ManipulationGoal(
                command_id=f"cmd-{self._command_counter}",
                command_type=cmd.command_type.value,
                square=cmd.target_square,
                row=cmd.target_row,
                col=cmd.target_col,
                is_king_stack=cmd.is_king_stack,
                metadata=cmd.metadata,
            )

            goal_msg = String()
            goal_msg.data = goal.to_json()
            self.manip_goal_pub.publish(goal_msg)

            legacy_msg = String()
            legacy_msg.data = json.dumps({
                'command_id': goal.command_id,
                'type': goal.command_type,
                'square': goal.square,
                'row': goal.row,
                'col': goal.col,
                'is_king_stack': goal.is_king_stack,
                'metadata': goal.metadata,
            })
            self.manip_legacy_pub.publish(legacy_msg)

            instruction_msg = String()
            instruction_msg.data = RobotInstruction(
                state=self.state_machine.state.value,
                summary=f"Dispatch {cmd.command_type.value}",
                move_number=self.ctx.move_number,
                current_turn=_colour_name(self.ctx.current_turn),
                active_command=cmd.command_type.value,
                metadata={
                    'command_id': goal.command_id,
                    'square': goal.square,
                    'row': goal.row,
                    'col': goal.col,
                },
            ).to_json()
            self.instruction_pub.publish(instruction_msg)

    def get_pending_commands(self) -> list[ManipulationCommand]:
        """Get and clear pending manipulation commands (for standalone mode)."""
        cmds = self._pending_commands[:]
        self._pending_commands.clear()
        return cmds

    def update_perceived_board(self, board: Board):
        """Update the perceived board state (for standalone mode)."""
        self.ctx.perceived_board = board

    def step(self) -> GameState:
        """Step the state machine (for standalone mode)."""
        return self.state_machine.step(self.ctx)

    def run(self):
        """Run the ROS2 node (blocking)."""
        if self.use_ros2 and HAS_ROS2:
            rclpy.spin(self.node)
        else:
            logger.warning("Cannot run spin loop — ROS2 not available")

    def shutdown(self):
        """Shutdown the node."""
        if self.use_ros2 and HAS_ROS2:
            self.node.destroy_node()
            rclpy.shutdown()


def _colour_name(colour: int) -> str:
    return "black" if colour == CB_BLACK else "white"


def main():
    """Entry point for the game manager node."""
    logging.basicConfig(level=logging.INFO)
    manager = GameManagerNode(search_time=5.0, human_colour=CB_BLACK)
    try:
        manager.run()
    except KeyboardInterrupt:
        pass
    finally:
        manager.shutdown()


if __name__ == '__main__':
    main()
