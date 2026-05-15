"""
game_states.py — State machine for the checkers-playing UR5e.

Implements all game states, transitions, and orchestration logic for a
complete game of English checkers between the UR5e robot and a human player.

States:
    INIT → WAIT_HUMAN_MOVE → VALIDATE_HUMAN_MOVE → PLAN_ROBOT_MOVE
    → EXECUTE_MOVE → CHECK_KING_PROMOTION → WAIT_HUMAN_MOVE → ...
    With branches for: ILLEGAL_MOVE_RESPONSE, EXECUTE_CAPTURE,
    PROMOTE_TO_KING, GAME_OVER.
"""

from __future__ import annotations
import enum
import logging
from dataclasses import dataclass, field
from typing import List, Optional, Callable

from ..game_engine.board import (
    Board, Move, CB_BLACK, CB_WHITE,
    INTERNAL_TO_ROWCOL, INTERNAL_TO_XY, XY_TO_INTERNAL,
)
from ..game_engine.rules import Rules
from ..game_engine.search import Search, SearchStats

logger = logging.getLogger(__name__)


class GameState(enum.Enum):
    """All possible states in the checkers game state machine."""

    INIT = "INIT"
    WAIT_HUMAN_MOVE = "WAIT_HUMAN_MOVE"
    VALIDATE_HUMAN_MOVE = "VALIDATE_HUMAN_MOVE"
    ILLEGAL_MOVE_RESPONSE = "ILLEGAL_MOVE_RESPONSE"
    UNDO_ILLEGAL = "UNDO_ILLEGAL"
    PLAN_ROBOT_MOVE = "PLAN_ROBOT_MOVE"
    EXECUTE_MOVE = "EXECUTE_MOVE"
    EXECUTE_SIMPLE = "EXECUTE_SIMPLE"
    EXECUTE_CAPTURE = "EXECUTE_CAPTURE"
    PICK_PIECE = "PICK_PIECE"
    BOB_OVER_SQUARE = "BOB_OVER_SQUARE"
    CAPTURE_PIECE = "CAPTURE_PIECE"
    PLACE_PIECE = "PLACE_PIECE"
    DISCARD_CAPTURED = "DISCARD_CAPTURED"
    CHECK_KING_PROMOTION = "CHECK_KING_PROMOTION"
    PROMOTE_TO_KING = "PROMOTE_TO_KING"
    RETURN_HOME = "RETURN_HOME"
    GAME_OVER = "GAME_OVER"
    ERROR = "ERROR"


@dataclass
class ManipulationCommand:
    """A command sent to the manipulation node."""

    class Type(enum.Enum):
        PICK = "PICK"
        PLACE = "PLACE"
        BOB = "BOB"
        GRASP = "GRASP"
        RELEASE = "RELEASE"
        MOVE_TO = "MOVE_TO"
        MOVE_HOME = "MOVE_HOME"
        FINGER_WAG = "FINGER_WAG"

    command_type: 'ManipulationCommand.Type'
    target_square: Optional[int] = None  # internal square number
    target_row: int = 0
    target_col: int = 0
    is_king_stack: bool = False  # True if placing on top of another piece
    metadata: dict = field(default_factory=dict)


@dataclass
class GameContext:
    """All mutable state for the game state machine."""

    # Board state
    board: Board = field(default_factory=Board)
    perceived_board: Optional[Board] = None
    previous_board: Optional[Board] = None

    # Game state
    human_colour: int = CB_BLACK      # Human plays black by default
    robot_colour: int = CB_WHITE      # Robot plays white by default
    current_turn: int = CB_BLACK      # Black moves first in English checkers
    move_number: int = 0

    # Current move being executed
    current_move: Optional[Move] = None
    robot_move: Optional[Move] = None
    search_stats: Optional[SearchStats] = None

    # Capture execution state
    capture_path_index: int = 0
    captured_pieces_to_discard: List[int] = field(default_factory=list)
    discard_index: int = 0
    total_discarded: int = 0

    # Game result
    game_over: bool = False
    winner: Optional[str] = None

    # Error tracking
    illegal_move_count: int = 0
    error_message: str = ""

    # Move history
    move_history: List[str] = field(default_factory=list)

    # Sequential manipulation dispatch
    command_queue: List[ManipulationCommand] = field(default_factory=list)
    command_continuation: Optional['GameState'] = None
    next_state_after_home: Optional['GameState'] = None


class GameStateMachine:
    """State machine orchestrating the checkers game.

    The state machine is event-driven: each state transition is triggered
    by calling `step()` with the current context. The machine may generate
    ManipulationCommands that need to be executed by the manipulation node.

    Typical flow:
        1. Create GameStateMachine and GameContext
        2. Call step() repeatedly
        3. Check context.manipulation_commands for robot actions
        4. Provide perception updates via context.perceived_board

    Usage with ROS2:
        The game_manager_node runs a timer callback that calls step(),
        checks for generated manipulation commands, sends them to the
        manipulation node, and waits for completion before stepping again.
    """

    def __init__(
        self,
        search_time: float = 5.0,
        max_search_depth: int = 99,
        human_colour: int = CB_BLACK,
    ):
        self.rules = Rules()
        self.search = Search(max_time=search_time, max_depth=max_search_depth)
        self.state = GameState.INIT
        self.human_colour = human_colour
        self.robot_colour = CB_WHITE if human_colour == CB_BLACK else CB_BLACK

        # Callbacks for integration with ROS2 nodes
        self._on_state_change: Optional[Callable] = None
        self._on_manipulation_cmd: Optional[Callable] = None
        self._manipulation_done: bool = True

    def set_callbacks(
        self,
        on_state_change: Optional[Callable[[GameState, GameState], None]] = None,
        on_manipulation_cmd: Optional[Callable[[ManipulationCommand], None]] = None,
    ):
        """Set callbacks for ROS2 integration."""
        self._on_state_change = on_state_change
        self._on_manipulation_cmd = on_manipulation_cmd

    def notify_manipulation_done(self):
        """Called by the manipulation node when a command completes."""
        self._manipulation_done = True

    def _transition(self, new_state: GameState, _ctx: GameContext):
        """Transition to a new state."""
        old_state = self.state
        self.state = new_state
        logger.info(f"State transition: {old_state.value} → {new_state.value}")
        if self._on_state_change:
            self._on_state_change(old_state, new_state)

    def _emit_manipulation(self, cmd: ManipulationCommand):
        """Send a manipulation command."""
        self._manipulation_done = False
        if self._on_manipulation_cmd:
            self._on_manipulation_cmd(cmd)

    def _queue_manipulations(
        self,
        ctx: GameContext,
        commands: List[ManipulationCommand],
        continuation: Optional[GameState] = None,
    ):
        """Queue a manipulation batch to be dispatched one command at a time."""
        ctx.command_queue = list(commands)
        ctx.command_continuation = continuation

    def _dispatch_pending_manipulation(self, ctx: GameContext) -> bool:
        """Dispatch the next queued command or finish the queued sequence."""
        if ctx.command_queue:
            self._emit_manipulation(ctx.command_queue.pop(0))
            return True

        if ctx.command_continuation is not None:
            next_state = ctx.command_continuation
            ctx.command_continuation = None
            self._transition(next_state, ctx)
            return True

        return False

    def _prepare_human_turn_handoff(self, ctx: GameContext):
        """Prepare the state transition that happens after the robot returns home."""
        ctx.current_turn = ctx.human_colour
        ctx.current_move = None
        ctx.robot_move = None
        ctx.search_stats = None
        ctx.perceived_board = None

        game_over, winner = self.rules.is_game_over(ctx.board, ctx.current_turn)
        if game_over:
            ctx.game_over = True
            ctx.winner = winner
            ctx.next_state_after_home = GameState.GAME_OVER
        else:
            ctx.game_over = False
            ctx.winner = None
            ctx.next_state_after_home = GameState.WAIT_HUMAN_MOVE

    def _reconstruct_capture_path(self, move: Move) -> List[int]:
        """Recover landing squares for multi-hop captures when the engine stores only endpoints."""
        if not move.is_capture:
            return move.path_squares if move.path_squares else [move.from_sq, move.to_sq]

        if move.path_squares and len(move.path_squares) > 2:
            return move.path_squares

        if not move.captured_squares:
            return [move.from_sq, move.to_sq]

        path = [move.from_sq]
        current_sq = move.from_sq

        for cap_sq in move.captured_squares:
            current_xy = INTERNAL_TO_XY.get(current_sq)
            captured_xy = INTERNAL_TO_XY.get(cap_sq)
            if current_xy is None or captured_xy is None:
                return [move.from_sq, move.to_sq]

            dx = captured_xy[0] - current_xy[0]
            dy = captured_xy[1] - current_xy[1]
            landing_sq = XY_TO_INTERNAL.get((captured_xy[0] + dx, captured_xy[1] + dy))
            if landing_sq is None:
                return [move.from_sq, move.to_sq]

            path.append(landing_sq)
            current_sq = landing_sq

        if path[-1] != move.to_sq:
            path.append(move.to_sq)

        return path

    def step(self, ctx: GameContext) -> GameState:
        """Execute one step of the state machine.

        Call this repeatedly from the game manager node's timer callback.

        Args:
            ctx: Mutable game context.

        Returns:
            The current state after this step.
        """
        if not self._manipulation_done:
            return self.state  # Wait for manipulation to complete

        if self._dispatch_pending_manipulation(ctx):
            return self.state

        handler = {
            GameState.INIT: self._handle_init,
            GameState.WAIT_HUMAN_MOVE: self._handle_wait_human_move,
            GameState.VALIDATE_HUMAN_MOVE: self._handle_validate_human_move,
            GameState.ILLEGAL_MOVE_RESPONSE: self._handle_illegal_move_response,
            GameState.UNDO_ILLEGAL: self._handle_undo_illegal,
            GameState.PLAN_ROBOT_MOVE: self._handle_plan_robot_move,
            GameState.EXECUTE_MOVE: self._handle_execute_move,
            GameState.EXECUTE_SIMPLE: self._handle_execute_simple,
            GameState.EXECUTE_CAPTURE: self._handle_execute_capture,
            GameState.PICK_PIECE: self._handle_pick_piece,
            GameState.BOB_OVER_SQUARE: self._handle_bob_over_square,
            GameState.CAPTURE_PIECE: self._handle_capture_piece,
            GameState.PLACE_PIECE: self._handle_place_piece,
            GameState.DISCARD_CAPTURED: self._handle_discard_captured,
            GameState.CHECK_KING_PROMOTION: self._handle_check_king_promotion,
            GameState.PROMOTE_TO_KING: self._handle_promote_to_king,
            GameState.RETURN_HOME: self._handle_return_home,
            GameState.GAME_OVER: self._handle_game_over,
            GameState.ERROR: self._handle_error,
        }.get(self.state)

        if handler:
            handler(ctx)
        else:
            logger.error(f"No handler for state {self.state}")
            self._transition(GameState.ERROR, ctx)

        if self._manipulation_done:
            self._dispatch_pending_manipulation(ctx)

        return self.state

    # ─── State Handlers ──────────────────────────────────────────────────────

    def _handle_init(self, ctx: GameContext):
        """Initialize the game."""
        ctx.board = Board()  # Starting position
        ctx.human_colour = self.human_colour
        ctx.robot_colour = self.robot_colour
        ctx.current_turn = CB_BLACK  # Black always moves first
        ctx.move_number = 0
        ctx.game_over = False
        ctx.winner = None
        ctx.total_discarded = 0
        ctx.move_history = []

        logger.info(
            f"Game initialized. Human={_colour_name(ctx.human_colour)}, "
            f"Robot={_colour_name(ctx.robot_colour)}"
        )
        logger.info(f"Board:\n{ctx.board}")

        # Determine who moves first
        if ctx.current_turn == ctx.human_colour:
            self._transition(GameState.WAIT_HUMAN_MOVE, ctx)
        else:
            self._transition(GameState.PLAN_ROBOT_MOVE, ctx)

    def _handle_wait_human_move(self, ctx: GameContext):
        """Wait for the human to make a move (detected by perception)."""
        if ctx.perceived_board is None:
            return  # No perception update yet

        # Check if the board has changed
        if self.rules.detect_board_change(ctx.board, ctx.perceived_board):
            ctx.previous_board = ctx.board.copy()
            self._transition(GameState.VALIDATE_HUMAN_MOVE, ctx)

    def _handle_validate_human_move(self, ctx: GameContext):
        """Validate the human's move against the rules."""
        is_legal, move = self.rules.validate_human_move(
            ctx.previous_board, ctx.perceived_board, ctx.human_colour
        )

        if is_legal and move is not None:
            # Legal move — apply it
            ctx.board = ctx.perceived_board.copy()
            ctx.current_move = move
            ctx.move_number += 1
            ctx.move_history.append(
                f"{ctx.move_number}. {_colour_name(ctx.human_colour)}: "
                f"{self.rules.format_move(move)}"
            )
            logger.info(
                f"Human played: {self.rules.format_move(move)}"
            )

            # Switch turn and check game over
            ctx.current_turn = ctx.robot_colour
            ctx.perceived_board = None
            game_over, winner = self.rules.is_game_over(ctx.board, ctx.current_turn)

            if game_over:
                ctx.game_over = True
                ctx.winner = winner
                self._transition(GameState.GAME_OVER, ctx)
            else:
                self._transition(GameState.PLAN_ROBOT_MOVE, ctx)
        else:
            # Illegal move!
            logger.warning("Illegal move detected!")
            ctx.illegal_move_count += 1
            self._transition(GameState.ILLEGAL_MOVE_RESPONSE, ctx)

    def _handle_illegal_move_response(self, ctx: GameContext):
        """Queue an illegal-move response, then return the arm to a safe home pose."""
        logger.info("Executing illegal-move response...")
        ctx.perceived_board = None
        self._queue_manipulations(
            ctx,
            [
                ManipulationCommand(
                    command_type=ManipulationCommand.Type.FINGER_WAG,
                ),
                ManipulationCommand(
                    command_type=ManipulationCommand.Type.MOVE_HOME,
                ),
            ],
            continuation=GameState.WAIT_HUMAN_MOVE,
        )

    def _handle_undo_illegal(self, ctx: GameContext):
        """Legacy compatibility state: immediately resume waiting for a valid human move."""
        ctx.perceived_board = None
        self._transition(GameState.WAIT_HUMAN_MOVE, ctx)

    def _handle_plan_robot_move(self, ctx: GameContext):
        """Use the AI engine to plan the robot's move."""
        logger.info("Planning robot move...")
        stats = self.search.find_best_move(ctx.board, ctx.robot_colour)
        ctx.search_stats = stats

        if stats.best_move is None:
            # No legal moves — robot loses
            ctx.game_over = True
            ctx.winner = _colour_name(ctx.human_colour)
            self._transition(GameState.GAME_OVER, ctx)
            return

        ctx.robot_move = stats.best_move
        logger.info(
            f"AI selected: {self.rules.format_move(stats.best_move)} "
            f"(eval={stats.eval_score}, depth={stats.depth_reached}, "
            f"time={stats.time_elapsed:.2f}s, nodes={stats.nodes})"
        )

        self._transition(GameState.EXECUTE_MOVE, ctx)

    def _handle_execute_move(self, ctx: GameContext):
        """Dispatch to simple or capture execution pipeline."""
        move = ctx.robot_move
        if move is None:
            self._transition(GameState.ERROR, ctx)
            return

        if move.is_capture:
            # Set up capture execution state
            ctx.capture_path_index = 0
            ctx.captured_pieces_to_discard = move.captured_squares[:]
            ctx.discard_index = 0
            self._transition(GameState.EXECUTE_CAPTURE, ctx)
        else:
            self._transition(GameState.EXECUTE_SIMPLE, ctx)

    def _handle_execute_simple(self, ctx: GameContext):
        """Execute a simple (non-capture) move."""
        move = ctx.robot_move
        if move is None:
            self._transition(GameState.ERROR, ctx)
            return

        from_row, from_col = INTERNAL_TO_ROWCOL[move.from_sq]
        to_row, to_col = INTERNAL_TO_ROWCOL[move.to_sq]

        # Apply the move to internal board
        ctx.board.apply_move_inplace(move)
        ctx.move_number += 1
        ctx.move_history.append(
            f"{ctx.move_number}. {_colour_name(ctx.robot_colour)}: "
            f"{self.rules.format_move(move)}"
        )

        self._queue_manipulations(
            ctx,
            [
                ManipulationCommand(
                    command_type=ManipulationCommand.Type.PICK,
                    target_square=move.from_sq,
                    target_row=from_row,
                    target_col=from_col,
                ),
                ManipulationCommand(
                    command_type=ManipulationCommand.Type.PLACE,
                    target_square=move.to_sq,
                    target_row=to_row,
                    target_col=to_col,
                ),
            ],
            continuation=GameState.CHECK_KING_PROMOTION,
        )

    def _handle_execute_capture(self, ctx: GameContext):
        """Queue the full capture sequence so each manipulation completes before the next begins."""
        move = ctx.robot_move
        if move is None:
            self._transition(GameState.ERROR, ctx)
            return

        from_row, from_col = INTERNAL_TO_ROWCOL[move.from_sq]
        commands = [
            ManipulationCommand(
                command_type=ManipulationCommand.Type.PICK,
                target_square=move.from_sq,
                target_row=from_row,
                target_col=from_col,
            )
        ]

        capture_path = self._reconstruct_capture_path(move)
        for sq in capture_path[1:-1]:
            row, col = INTERNAL_TO_ROWCOL[sq]
            commands.append(
                ManipulationCommand(
                    command_type=ManipulationCommand.Type.BOB,
                    target_square=sq,
                    target_row=row,
                    target_col=col,
                )
            )

        to_row, to_col = INTERNAL_TO_ROWCOL[move.to_sq]
        commands.append(
            ManipulationCommand(
                command_type=ManipulationCommand.Type.PLACE,
                target_square=move.to_sq,
                target_row=to_row,
                target_col=to_col,
            )
        )

        for capture_index, cap_sq in enumerate(move.captured_squares):
            cap_row, cap_col = INTERNAL_TO_ROWCOL[cap_sq]
            commands.extend([
                ManipulationCommand(
                    command_type=ManipulationCommand.Type.PICK,
                    target_square=cap_sq,
                    target_row=cap_row,
                    target_col=cap_col,
                ),
                ManipulationCommand(
                    command_type=ManipulationCommand.Type.PLACE,
                    target_square=-1,
                    target_row=-1,
                    target_col=-1,
                    metadata={'discard_index': ctx.total_discarded + capture_index},
                ),
            ])

        ctx.total_discarded += len(move.captured_squares)
        ctx.board.apply_move_inplace(move)
        ctx.move_number += 1
        ctx.move_history.append(
            f"{ctx.move_number}. {_colour_name(ctx.robot_colour)}: "
            f"{self.rules.format_move(move)}"
        )

        self._queue_manipulations(
            ctx,
            commands,
            continuation=GameState.CHECK_KING_PROMOTION,
        )

    def _handle_pick_piece(self, ctx: GameContext):
        """Legacy compatibility state: capture picking is now queued in one batch."""
        self._transition(GameState.BOB_OVER_SQUARE, ctx)

    def _handle_bob_over_square(self, ctx: GameContext):
        """Bob down and up over intermediate squares during multi-jump."""
        move = ctx.robot_move
        if move is None or ctx.capture_path_index >= len(move.path_squares):
            self._transition(GameState.PLACE_PIECE, ctx)
            return

        from ..game_engine.board import INTERNAL_TO_ROWCOL

        sq = move.path_squares[ctx.capture_path_index]
        row, col = INTERNAL_TO_ROWCOL[sq]

        self._emit_manipulation(ManipulationCommand(
            command_type=ManipulationCommand.Type.BOB,
            target_square=sq,
            target_row=row,
            target_col=col,
        ))

        ctx.capture_path_index += 1

        # Check if this is the last path square (destination)
        if ctx.capture_path_index >= len(move.path_squares):
            self._transition(GameState.PLACE_PIECE, ctx)
        else:
            # More path squares to bob over
            pass  # Stay in BOB_OVER_SQUARE state

    def _handle_capture_piece(self, ctx: GameContext):
        """Stub — captures are collected and discarded after placing."""
        self._transition(GameState.BOB_OVER_SQUARE, ctx)

    def _handle_place_piece(self, ctx: GameContext):
        """Legacy compatibility state: capture execution is queued in one batch."""
        self._transition(GameState.CHECK_KING_PROMOTION, ctx)

    def _handle_discard_captured(self, ctx: GameContext):
        """Legacy compatibility state: capture discards are queued in one batch."""
        self._transition(GameState.CHECK_KING_PROMOTION, ctx)

    def _handle_check_king_promotion(self, ctx: GameContext):
        """Check if the piece that just moved should be promoted to king."""
        move = ctx.robot_move
        if move is not None and move.is_promotion:
            self._transition(GameState.PROMOTE_TO_KING, ctx)
            return

        self._prepare_human_turn_handoff(ctx)
        self._transition(GameState.RETURN_HOME, ctx)

    def _handle_promote_to_king(self, ctx: GameContext):
        """Physically crown the promoted piece.

        Either stack a second checker on top or flip the piece.
        """
        move = ctx.robot_move
        if move is None:
            self._transition(GameState.ERROR, ctx)
            return

        to_row, to_col = INTERNAL_TO_ROWCOL[move.to_sq]

        logger.info(
            f"King promotion at square {move.to_sq} "
            f"(row={to_row}, col={to_col})"
        )

        self._prepare_human_turn_handoff(ctx)
        self._queue_manipulations(
            ctx,
            [
                ManipulationCommand(
                    command_type=ManipulationCommand.Type.PLACE,
                    target_square=move.to_sq,
                    target_row=to_row,
                    target_col=to_col,
                    is_king_stack=True,
                    metadata={'colour': ctx.robot_colour},
                )
            ],
            continuation=GameState.RETURN_HOME,
        )

    def _handle_return_home(self, ctx: GameContext):
        """Move the arm back to its safe default pose before waiting for the next event."""
        next_state = ctx.next_state_after_home or GameState.WAIT_HUMAN_MOVE
        ctx.next_state_after_home = None
        self._queue_manipulations(
            ctx,
            [ManipulationCommand(command_type=ManipulationCommand.Type.MOVE_HOME)],
            continuation=next_state,
        )

    def _handle_game_over(self, ctx: GameContext):
        """Handle game over."""
        logger.info(f"Game Over! Winner: {ctx.winner}")
        logger.info(f"Move history ({ctx.move_number} moves):")
        for entry in ctx.move_history:
            logger.info(f"  {entry}")

    def _handle_error(self, ctx: GameContext):
        """Handle error state."""
        logger.error(f"Error: {ctx.error_message}")


def _colour_name(colour: int) -> str:
    """Convert colour constant to human-readable name."""
    return "black" if colour == CB_BLACK else "white"
