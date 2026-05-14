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
from typing import List, Optional, Callable, Any

from ..game_engine.board import Board, Move, CB_BLACK, CB_WHITE
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

    def _transition(self, new_state: GameState, ctx: GameContext):
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
            GameState.GAME_OVER: self._handle_game_over,
            GameState.ERROR: self._handle_error,
        }.get(self.state)

        if handler:
            handler(ctx)
        else:
            logger.error(f"No handler for state {self.state}")
            self._transition(GameState.ERROR, ctx)

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
        """Wag finger at the human for an illegal move."""
        logger.info("Executing finger wag gesture...")
        self._emit_manipulation(ManipulationCommand(
            command_type=ManipulationCommand.Type.FINGER_WAG,
        ))
        self._transition(GameState.UNDO_ILLEGAL, ctx)

    def _handle_undo_illegal(self, ctx: GameContext):
        """Physically undo the illegal move on the board.

        Compare perceived board with the expected board and move pieces back.
        """
        if ctx.perceived_board is None or ctx.previous_board is None:
            self._transition(GameState.ERROR, ctx)
            return

        # Find differences and generate commands to restore
        diffs = ctx.previous_board.diff(ctx.perceived_board)
        logger.info(f"Restoring {len(diffs)} square(s) to undo illegal move")

        # For each difference, we need to move pieces back
        # This is simplified — a full implementation would plan the exact
        # pick-place sequence based on which pieces moved where
        for sq, expected_piece, actual_piece in diffs:
            from ..game_engine.board import INTERNAL_TO_ROWCOL
            row, col = INTERNAL_TO_ROWCOL[sq]

            if actual_piece != 16 and expected_piece == 16:
                # Piece appeared where it shouldn't be — remove it
                self._emit_manipulation(ManipulationCommand(
                    command_type=ManipulationCommand.Type.PICK,
                    target_square=sq,
                    target_row=row,
                    target_col=col,
                ))
            elif actual_piece == 16 and expected_piece != 16:
                # Piece missing — it will be placed back after picking
                self._emit_manipulation(ManipulationCommand(
                    command_type=ManipulationCommand.Type.PLACE,
                    target_square=sq,
                    target_row=row,
                    target_col=col,
                ))

        # After undoing, go back to waiting
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

        from ..game_engine.board import INTERNAL_TO_ROWCOL

        # Pick from source
        from_row, from_col = INTERNAL_TO_ROWCOL[move.from_sq]
        self._emit_manipulation(ManipulationCommand(
            command_type=ManipulationCommand.Type.PICK,
            target_square=move.from_sq,
            target_row=from_row,
            target_col=from_col,
        ))

        # Place at destination
        to_row, to_col = INTERNAL_TO_ROWCOL[move.to_sq]
        self._emit_manipulation(ManipulationCommand(
            command_type=ManipulationCommand.Type.PLACE,
            target_square=move.to_sq,
            target_row=to_row,
            target_col=to_col,
        ))

        # Apply the move to internal board
        ctx.board.apply_move_inplace(move)
        ctx.move_number += 1
        ctx.move_history.append(
            f"{ctx.move_number}. {_colour_name(ctx.robot_colour)}: "
            f"{self.rules.format_move(move)}"
        )

        self._transition(GameState.CHECK_KING_PROMOTION, ctx)

    def _handle_execute_capture(self, ctx: GameContext):
        """Begin executing a capture move — pick up the moving piece."""
        move = ctx.robot_move
        if move is None:
            self._transition(GameState.ERROR, ctx)
            return

        from ..game_engine.board import INTERNAL_TO_ROWCOL

        # Pick up the moving piece
        from_row, from_col = INTERNAL_TO_ROWCOL[move.from_sq]
        self._emit_manipulation(ManipulationCommand(
            command_type=ManipulationCommand.Type.PICK,
            target_square=move.from_sq,
            target_row=from_row,
            target_col=from_col,
        ))

        ctx.capture_path_index = 1  # Start from path[1] (path[0] is 'from')
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
        """Place the moving piece at its destination."""
        move = ctx.robot_move
        if move is None:
            self._transition(GameState.ERROR, ctx)
            return

        from ..game_engine.board import INTERNAL_TO_ROWCOL

        to_row, to_col = INTERNAL_TO_ROWCOL[move.to_sq]
        self._emit_manipulation(ManipulationCommand(
            command_type=ManipulationCommand.Type.PLACE,
            target_square=move.to_sq,
            target_row=to_row,
            target_col=to_col,
        ))

        # Now discard captured pieces
        if ctx.captured_pieces_to_discard:
            ctx.discard_index = 0
            self._transition(GameState.DISCARD_CAPTURED, ctx)
        else:
            # Apply move and check promotion
            ctx.board.apply_move_inplace(move)
            ctx.move_number += 1
            ctx.move_history.append(
                f"{ctx.move_number}. {_colour_name(ctx.robot_colour)}: "
                f"{self.rules.format_move(move)}"
            )
            self._transition(GameState.CHECK_KING_PROMOTION, ctx)

    def _handle_discard_captured(self, ctx: GameContext):
        """Pick up captured pieces and move them to the discard pile."""
        if ctx.discard_index >= len(ctx.captured_pieces_to_discard):
            # All captured pieces discarded — apply move and continue
            move = ctx.robot_move
            if move is not None:
                ctx.board.apply_move_inplace(move)
                ctx.move_number += 1
                ctx.move_history.append(
                    f"{ctx.move_number}. {_colour_name(ctx.robot_colour)}: "
                    f"{self.rules.format_move(move)}"
                )
            self._transition(GameState.CHECK_KING_PROMOTION, ctx)
            return

        from ..game_engine.board import INTERNAL_TO_ROWCOL

        cap_sq = ctx.captured_pieces_to_discard[ctx.discard_index]
        cap_row, cap_col = INTERNAL_TO_ROWCOL[cap_sq]

        # Pick up the captured piece
        self._emit_manipulation(ManipulationCommand(
            command_type=ManipulationCommand.Type.PICK,
            target_square=cap_sq,
            target_row=cap_row,
            target_col=cap_col,
        ))

        # Place in discard pile
        self._emit_manipulation(ManipulationCommand(
            command_type=ManipulationCommand.Type.PLACE,
            target_square=-1,  # -1 = discard pile
            target_row=-1,
            target_col=-1,
            metadata={'discard_index': ctx.total_discarded + ctx.discard_index},
        ))

        ctx.discard_index += 1
        ctx.total_discarded += 1

    def _handle_check_king_promotion(self, ctx: GameContext):
        """Check if the piece that just moved should be promoted to king."""
        move = ctx.robot_move
        if move is not None and move.is_promotion:
            self._transition(GameState.PROMOTE_TO_KING, ctx)
            return

        # Switch turn and check game over
        ctx.current_turn = ctx.human_colour
        game_over, winner = self.rules.is_game_over(ctx.board, ctx.current_turn)

        if game_over:
            ctx.game_over = True
            ctx.winner = winner
            self._transition(GameState.GAME_OVER, ctx)
        else:
            ctx.perceived_board = None  # Reset perception
            self._transition(GameState.WAIT_HUMAN_MOVE, ctx)

    def _handle_promote_to_king(self, ctx: GameContext):
        """Physically crown the promoted piece.

        Either stack a second checker on top or flip the piece.
        """
        move = ctx.robot_move
        if move is None:
            self._transition(GameState.ERROR, ctx)
            return

        from ..game_engine.board import INTERNAL_TO_ROWCOL

        to_row, to_col = INTERNAL_TO_ROWCOL[move.to_sq]

        logger.info(
            f"King promotion at square {move.to_sq} "
            f"(row={to_row}, col={to_col})"
        )

        # Place a second piece on top for king promotion
        self._emit_manipulation(ManipulationCommand(
            command_type=ManipulationCommand.Type.PLACE,
            target_square=move.to_sq,
            target_row=to_row,
            target_col=to_col,
            is_king_stack=True,
            metadata={'colour': ctx.robot_colour},
        ))

        # Switch turn and check game over
        ctx.current_turn = ctx.human_colour
        game_over, winner = self.rules.is_game_over(ctx.board, ctx.current_turn)

        if game_over:
            ctx.game_over = True
            ctx.winner = winner
            self._transition(GameState.GAME_OVER, ctx)
        else:
            ctx.perceived_board = None
            self._transition(GameState.WAIT_HUMAN_MOVE, ctx)

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
