"""
search.py — Alpha-beta search with iterative deepening.

Ported from SimpleCh (Martin Fierz), simplech.c lines 488–727.

Features:
    - Iterative deepening
    - Alpha-beta pruning
    - Quiescence search (extends by 1 ply when captures exist at leaf)
    - Time management (configurable search time)
    - Forced move detection (returns immediately if only one legal move)
"""

from __future__ import annotations
import time
from typing import List, Optional, Tuple

from .board import Board, Move, CB_BLACK, CB_WHITE, FREE
from .move_generator import MoveGenerator
from .evaluation import evaluate

BLACK = CB_BLACK
WHITE = CB_WHITE

# Sentinel values
WIN_SCORE = 5000
LOSS_SCORE = -5000
MAXDEPTH = 99


def change_colour(colour: int) -> int:
    """Return the opponent's colour."""
    return colour ^ (CB_BLACK | CB_WHITE)


class SearchStats:
    """Statistics for a search run."""

    def __init__(self):
        self.nodes: int = 0
        self.depth_reached: int = 0
        self.time_elapsed: float = 0.0
        self.eval_score: int = 0
        self.best_move: Optional[Move] = None
        self.forced: bool = False

    def __repr__(self) -> str:
        mv = self.best_move.to_notation() if self.best_move else "None"
        return (
            f"SearchStats(move={mv}, depth={self.depth_reached}, "
            f"eval={self.eval_score}, nodes={self.nodes}, "
            f"time={self.time_elapsed:.3f}s, forced={self.forced})"
        )


class Search:
    """Iterative-deepening alpha-beta search for English checkers.

    Usage:
        search = Search(max_time=5.0)
        stats = search.find_best_move(board, colour)
        best_move = stats.best_move
    """

    def __init__(self, max_time: float = 5.0, max_depth: int = MAXDEPTH):
        """
        Args:
            max_time: Maximum search time in seconds.
            max_depth: Maximum search depth (default: 99).
        """
        self.max_time = max_time
        self.max_depth = max_depth
        self.move_gen = MoveGenerator()

        # Internal state
        self._start_time: float = 0.0
        self._absolute_max_time: float = 0.0
        self._new_iter_max_time: float = 0.0
        self._abort: bool = False
        self._nodes: int = 0

    def find_best_move(self, board: Board, colour: int) -> SearchStats:
        """Find the best move for the given colour.

        Ported from simplech.c checkers() (lines 488–577).

        Args:
            board: Current board state.
            colour: Side to move (CB_BLACK or CB_WHITE).

        Returns:
            SearchStats with the best move and search information.
        """
        stats = SearchStats()
        self._start_time = time.monotonic()
        self._abort = False
        self._nodes = 0

        # Time management (from simplech.c lines 392–394)
        self._new_iter_max_time = 0.59 * self.max_time
        self._absolute_max_time = 3.0 * self.max_time

        b = board.b[:]  # working copy

        # Check for forced moves
        captures = self.move_gen.generate_capture_list(b, colour)
        if len(captures) == 1:
            # Forced capture — no need to search
            captures[0].decode()
            stats.best_move = captures[0]
            stats.forced = True
            stats.eval_score = 0
            stats.depth_reached = 0
            stats.time_elapsed = time.monotonic() - self._start_time
            return stats

        if not captures:
            moves = self.move_gen.generate_move_list(b, colour)
            if len(moves) == 1:
                # Only one move available
                moves[0].decode()
                stats.best_move = moves[0]
                stats.forced = True
                stats.eval_score = 0
                stats.depth_reached = 0
                stats.time_elapsed = time.monotonic() - self._start_time
                return stats

            if len(moves) == 0:
                # No legal moves — we lose
                stats.best_move = None
                stats.eval_score = LOSS_SCORE if colour == BLACK else WIN_SCORE
                stats.time_elapsed = time.monotonic() - self._start_time
                return stats

        # Iterative deepening
        best = None
        last_best = None
        eval_score = 0

        for depth in range(1, self.max_depth + 1):
            elapsed = time.monotonic() - self._start_time
            if elapsed >= self._new_iter_max_time:
                break

            last_best = best
            eval_score, best = self._first_alphabeta(
                b, depth, -10000, 10000, colour
            )

            stats.depth_reached = depth
            stats.eval_score = eval_score
            stats.nodes = self._nodes

            if self._abort:
                # Use the result from the previous iteration
                if last_best is not None:
                    best = last_best
                break

            if abs(eval_score) >= WIN_SCORE:
                break

        if best is not None:
            best.decode()
        stats.best_move = best
        stats.time_elapsed = time.monotonic() - self._start_time
        return stats

    def _first_alphabeta(
        self,
        b: List[int],
        depth: int,
        alpha: int,
        beta: int,
        colour: int,
    ) -> Tuple[int, Optional[Move]]:
        """Root-level alpha-beta search that returns both value and best move.

        Ported from simplech.c firstalphabeta() (lines 579–649).
        """
        self._nodes += 1

        if self._abort:
            return 0, None

        # Check for captures
        has_capture = self.move_gen.test_capture(b, colour)

        if depth == 0:
            if not has_capture:
                return evaluate(b, colour), None
            else:
                depth = 1  # Quiescence: extend search if captures available

        # Generate moves
        if has_capture:
            moves = self.move_gen.generate_capture_list(b, colour)
        else:
            moves = self.move_gen.generate_move_list(b, colour)

        if not moves:
            # No legal moves — we lose
            if colour == BLACK:
                return LOSS_SCORE, None
            else:
                return WIN_SCORE, None

        best_move = moves[0]  # Default to first move

        for move in moves:
            # Apply move
            self._domove(b, move)
            value = self._alphabeta(b, depth - 1, alpha, beta, change_colour(colour))
            self._undomove(b, move)

            if colour == BLACK:
                if value >= beta:
                    return value, move
                if value > alpha:
                    alpha = value
                    best_move = move
            else:  # WHITE
                if value <= alpha:
                    return value, move
                if value < beta:
                    beta = value
                    best_move = move

        if colour == BLACK:
            return alpha, best_move
        return beta, best_move

    def _alphabeta(
        self,
        b: List[int],
        depth: int,
        alpha: int,
        beta: int,
        colour: int,
    ) -> int:
        """Internal alpha-beta search.

        Ported from simplech.c alphabeta() (lines 651–727).
        """
        self._nodes += 1

        # Time check every 1024 nodes
        if (self._nodes & 0x3FF) == 0:
            elapsed = time.monotonic() - self._start_time
            if elapsed >= self._absolute_max_time:
                self._abort = True

        if self._abort:
            return 0

        # Check for captures
        has_capture = self.move_gen.test_capture(b, colour)

        if depth == 0:
            if not has_capture:
                return evaluate(b, colour)
            else:
                depth = 1  # Quiescence extension

        # Generate moves
        if has_capture:
            moves = self.move_gen.generate_capture_list(b, colour)
        else:
            moves = self.move_gen.generate_move_list(b, colour)

        if not moves:
            if colour == BLACK:
                return LOSS_SCORE
            else:
                return WIN_SCORE

        for move in moves:
            self._domove(b, move)
            value = self._alphabeta(b, depth - 1, alpha, beta, change_colour(colour))
            self._undomove(b, move)

            if colour == BLACK:
                if value >= beta:
                    return value
                if value > alpha:
                    alpha = value
            else:
                if value <= alpha:
                    return value
                if value < beta:
                    beta = value

        if colour == BLACK:
            return alpha
        return beta

    @staticmethod
    def _domove(b: List[int], move: Move):
        """Apply a move to the board array in-place.

        Ported from simplech.c domove() (lines 729–742).
        """
        for i in range(move.n):
            sq = move.m[i] & 0xFF
            after = (move.m[i] >> 16) & 0xFF
            b[sq] = after

    @staticmethod
    def _undomove(b: List[int], move: Move):
        """Undo a move on the board array in-place.

        Ported from simplech.c undomove() (lines 744–754).
        """
        for i in range(move.n - 1, -1, -1):
            sq = move.m[i] & 0xFF
            before = (move.m[i] >> 8) & 0xFF
            b[sq] = before
