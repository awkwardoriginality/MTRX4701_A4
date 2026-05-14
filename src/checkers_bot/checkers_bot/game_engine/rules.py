"""
rules.py — English checkers rules engine.

Provides high-level game logic:
    - Legality checking (validates human moves)
    - Board state comparison (perception → internal)
    - Game-over detection
    - King promotion detection
"""

from __future__ import annotations
from typing import List, Optional, Tuple

from .board import (
    Board, Move, PLAYABLE_SQUARES, INTERNAL_TO_STANDARD,
    CB_BLACK, CB_WHITE, CB_MAN, CB_KING,
    BLACK_MAN, BLACK_KING, WHITE_MAN, WHITE_KING,
    FREE, OCCUPIED,
)
from .move_generator import MoveGenerator


class Rules:
    """English checkers rules engine for game management."""

    def __init__(self):
        self.move_gen = MoveGenerator()

    def get_legal_moves(self, board: Board, colour: int) -> List[Move]:
        """Get all legal moves for the given colour."""
        return self.move_gen.get_legal_moves(board, colour)

    def has_legal_moves(self, board: Board, colour: int) -> bool:
        """Check if the given colour has any legal moves."""
        moves = self.move_gen.get_legal_moves(board, colour)
        return len(moves) > 0

    def is_game_over(self, board: Board, colour_to_move: int) -> Tuple[bool, Optional[str]]:
        """Check if the game is over.

        Returns:
            (is_over, winner) where winner is 'black', 'white', 'draw', or None.
        """
        # A player loses if they have no legal moves on their turn
        if not self.has_legal_moves(board, colour_to_move):
            if colour_to_move == CB_BLACK:
                return True, 'white'
            else:
                return True, 'black'

        # Check for no pieces remaining
        bm, bk = board.count_pieces(CB_BLACK)
        wm, wk = board.count_pieces(CB_WHITE)

        if bm + bk == 0:
            return True, 'white'
        if wm + wk == 0:
            return True, 'black'

        return False, None

    def validate_human_move(
        self,
        old_board: Board,
        new_board: Board,
        human_colour: int,
    ) -> Tuple[bool, Optional[Move]]:
        """Validate a human move by comparing board states.

        Enumerates all legal moves for the human's colour and checks
        if any of them produce the observed new board state.

        Args:
            old_board: Board state before the human moved.
            new_board: Board state after the human moved (from perception).
            human_colour: The human's colour (CB_BLACK or CB_WHITE).

        Returns:
            (is_legal, matching_move) — True and the matching move if legal,
            False and None if the move is illegal.
        """
        legal_moves = self.get_legal_moves(old_board, human_colour)

        for move in legal_moves:
            expected = old_board.apply_move(move)
            if expected.boards_match(new_board):
                return True, move

        return False, None

    def detect_board_change(self, old_board: Board, new_board: Board) -> bool:
        """Check if the board state has changed (any piece moved)."""
        return not old_board.boards_match(new_board)

    def detect_promotion(self, move: Move) -> bool:
        """Check if a move results in king promotion."""
        return move.is_promotion

    def get_capture_details(self, move: Move) -> List[Tuple[int, int, int]]:
        """Get details of captured pieces for manipulation planning.

        Returns:
            List of (internal_square, row, col) for each captured piece.
        """
        from .board import INTERNAL_TO_ROWCOL
        details = []
        for cap_sq in move.captured_squares:
            row, col = INTERNAL_TO_ROWCOL[cap_sq]
            details.append((cap_sq, row, col))
        return details

    def get_move_path_for_manipulation(
        self, move: Move
    ) -> List[Tuple[int, int, int]]:
        """Get the complete path of a move for robot manipulation.

        For simple moves: [(from_sq, row, col), (to_sq, row, col)]
        For captures: includes all intermediate squares the piece passes through.

        Returns:
            List of (internal_square, row, col) for the move path.
        """
        from .board import INTERNAL_TO_ROWCOL
        path = []
        for sq in move.path_squares:
            row, col = INTERNAL_TO_ROWCOL[sq]
            path.append((sq, row, col))
        return path

    def format_move(self, move: Move) -> str:
        """Format a move for display."""
        notation = move.to_notation()
        details = []
        if move.is_capture:
            details.append(f"{len(move.captured_squares)} capture(s)")
        if move.is_promotion:
            details.append("promotion")
        extra = f" ({', '.join(details)})" if details else ""
        return f"{notation}{extra}"

    def get_board_summary(self, board: Board) -> str:
        """Get a text summary of the current board state."""
        bm, bk = board.count_pieces(CB_BLACK)
        wm, wk = board.count_pieces(CB_WHITE)
        return (
            f"Black: {bm} men, {bk} kings | "
            f"White: {wm} men, {wk} kings | "
            f"Total: {bm + bk + wm + wk} pieces"
        )
