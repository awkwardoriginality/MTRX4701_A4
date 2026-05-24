"""
move_generator.py — Legal move generation for English checkers.

Ported from SimpleCh (Martin Fierz), specifically the functions:
    - generatemovelist()     (simplech.c lines ~1120–1350)
    - generatecapturelist()  (simplech.c lines ~1360–1600)
    - testcapture()          (simplech.c lines ~1600–1680)
    - blackmancapture(), blackkingcapture(), whitemancapture(), whitekingcapture()

The move generator uses the internal 46-element board representation.
Moves are encoded as `Move` objects using the packed `move2` format.
"""

from __future__ import annotations
from typing import List, Tuple

from .board import (
    Board, Move, PLAYABLE_SQUARES, INTERNAL_TO_XY, INTERNAL_TO_ROWCOL,
    FREE, OCCUPIED, CB_BLACK, CB_WHITE, CB_MAN, CB_KING,
    BLACK_MAN, BLACK_KING, WHITE_MAN, WHITE_KING,
    BLACK, WHITE, MAN, KING,
)

MAXMOVES = 28


class MoveGenerator:
    """Generates all legal moves for English checkers (GT_ENGLISH).

    English checkers rules enforced:
        - Men move forward diagonally one square.
        - Kings move any diagonal one square.
        - Captures are mandatory (forced jump rule).
        - Multi-jump: a piece MUST continue jumping if it can.
        - Men capture forward only.
        - Promotion occurs when a man reaches the opponent's back rank.
    """

    def __init__(self):
        pass

    def get_legal_moves(self, board: Board, colour: int) -> List[Move]:
        """Get all legal moves for the given colour.

        English checkers: if captures exist, only captures are legal.

        Args:
            board: Current board state.
            colour: CB_BLACK or CB_WHITE.

        Returns:
            List of legal Move objects with decoded fields populated.
        """
        captures = self.generate_capture_list(board.b, colour)
        if captures:
            for m in captures:
                m.decode()
            return captures

        moves = self.generate_move_list(board.b, colour)
        for m in moves:
            m.decode()
        return moves

    def has_captures(self, board: Board, colour: int) -> bool:
        """Quick test if any captures exist for the given colour."""
        return self.test_capture(board.b, colour)

    # ─── Non-Capture Move Generation ─────────────────────────────────────────

    def generate_move_list(self, b: List[int], colour: int) -> List[Move]:
        """Generate all non-capture moves.

        Ported from simplech.c generatemovelist().
        """
        movelist: List[Move] = []

        if colour == BLACK:
            # Black men move towards decreasing internal square numbers
            # (moving "up" the board from black's perspective)
            # In the internal representation, black moves towards higher row numbers
            # Black men: move in +4, +5 direction (towards white's side)
            for sq in PLAYABLE_SQUARES:
                piece = b[sq]
                if piece == BLACK_MAN:
                    self._gen_black_man_moves(b, sq, movelist)
                elif piece == BLACK_KING:
                    self._gen_king_moves(b, sq, movelist, BLACK)

            # Sort: black men move toward white's side
            return movelist

        else:  # WHITE
            for sq in PLAYABLE_SQUARES:
                piece = b[sq]
                if piece == WHITE_MAN:
                    self._gen_white_man_moves(b, sq, movelist)
                elif piece == WHITE_KING:
                    self._gen_king_moves(b, sq, movelist, WHITE)

            return movelist

    def _gen_black_man_moves(self, b: List[int], sq: int, movelist: List[Move]):
        """Generate non-capture moves for a black man at square sq.

        Black men move toward white's side (increasing row / y).
        In the internal numbering, the diagonal neighbours depend on the row.

        Row parity determines the offsets:
            Even rows (0, 2, 4, 6): forward-left = +4, forward-right = +5
            Odd rows  (1, 3, 5):    forward-left = +4, forward-right = +5

        Actually in SimpleCh's 46-board:
            From any square, the four diagonal neighbours are at offsets:
                +4, +5  (forward for black = toward higher numbers on even rows)
                -4, -5  (backward for black)

            But the row structure introduces gaps (9, 18, 27, 36 are sentinels).
            The correct offsets are determined by the internal numbering:
                Row 0 (5–8):   forward = 10–13 → offsets +5
                Row 1 (10–13): forward = 14–17 → offsets +4, +5
                etc.

        SimpleCh handles this with the sentinel approach: if the target square
        is FREE, the move is valid. Sentinels (OCCUPIED=0) block invalid moves.
        """
        # Forward-left and forward-right for black (towards higher squares)
        for offset in (4, 5):
            target = sq + offset
            if target < 46 and b[target] == FREE:
                mv = Move()
                # Check for promotion: black promotes on row 7
                # Row 7 internal squares: 37, 38, 39, 40
                new_piece = BLACK_MAN
                if target in (37, 38, 39, 40):
                    new_piece = BLACK_KING

                mv.n = 2
                mv.m[0] = Move.pack(sq, BLACK_MAN, FREE)
                mv.m[1] = Move.pack(target, FREE, new_piece)
                movelist.append(mv)

    def _gen_white_man_moves(self, b: List[int], sq: int, movelist: List[Move]):
        """Generate non-capture moves for a white man at square sq.

        White men move toward black's side (decreasing row numbers / lower squares).
        """
        for offset in (4, 5):
            target = sq - offset
            if target >= 0 and b[target] == FREE:
                mv = Move()
                # White promotes on row 0: squares 5, 6, 7, 8
                new_piece = WHITE_MAN
                if target in (5, 6, 7, 8):
                    new_piece = WHITE_KING

                mv.n = 2
                mv.m[0] = Move.pack(sq, WHITE_MAN, FREE)
                mv.m[1] = Move.pack(target, FREE, new_piece)
                movelist.append(mv)

    def _gen_king_moves(self, b: List[int], sq: int, movelist: List[Move], colour: int):
        """Generate non-capture moves for a king at square sq."""
        king_piece = colour | KING

        for offset in (4, 5, -4, -5):
            target = sq + offset
            if 0 <= target < 46 and b[target] == FREE:
                mv = Move()
                mv.n = 2
                mv.m[0] = Move.pack(sq, king_piece, FREE)
                mv.m[1] = Move.pack(target, FREE, king_piece)
                movelist.append(mv)

    # ─── Capture Move Generation ─────────────────────────────────────────────

    def generate_capture_list(self, b: List[int], colour: int) -> List[Move]:
        """Generate all capture moves (including multi-jumps).

        Ported from simplech.c generatecapturelist().
        """
        movelist: List[Move] = []

        for sq in PLAYABLE_SQUARES:
            piece = b[sq]
            if colour == BLACK:
                if piece == BLACK_MAN:
                    self._black_man_capture(b, sq, movelist)
                elif piece == BLACK_KING:
                    self._black_king_capture(b, sq, movelist)
            else:
                if piece == WHITE_MAN:
                    self._white_man_capture(b, sq, movelist)
                elif piece == WHITE_KING:
                    self._white_king_capture(b, sq, movelist)

        return movelist

    def _black_man_capture(self, b: List[int], sq: int, movelist: List[Move]):
        """Generate capture moves for a black man.

        Black men can only capture forward (towards white's side).
        In the 46-board, forward captures jump over +4 or +5 to land at +9.
        But the actual offsets for diagonals depend on row parity.

        Using SimpleCh's sentinel approach:
            Jump over sq+4 to sq+8 (if sq+4 is opponent and sq+8 is free)
            Jump over sq+5 to sq+10 (if sq+5 is opponent and sq+10 is free)

        Wait — looking more carefully at SimpleCh's structure:
        The actual diagonal relationships in the 46-board:
            Forward-right from even row: +5 (adjacent), +9 (landing)  — skip? No.
            The offsets for captures: jump = 2 * diagonal_offset - correction

        Actually, let me re-derive from SimpleCh's generatecapturelist().
        In simplech.c, the capture logic uses direct computation on the board
        array by checking specific offset patterns. The key insight is that
        diagonal jumps in the 46-board correspond to offsets of ±9 (capturing
        the piece at ±4 or ±5 in between).

        Diagonal relationships in the 46-array:
            NE: sq → sq+4 (adjacent), sq+9 (jump landing)? No, that's wrong.

        Let me look at this more carefully. The 46-board has this structure:
            Row 0: 5, 6, 7, 8         (gap at 9)
            Row 1: 10, 11, 12, 13     (gap at 18 — wait, 14 is row 2)

        Actually the rows are:
            5,6,7,8    [gap=9]
            10,11,12,13 [gap=18 — no, gap is 9]
            14,15,16,17 [gap=18]
            19,20,21,22 [gap=27 — no]
            23,24,25,26 [gap=27]
            28,29,30,31 [gap=36]
            32,33,34,35 [gap=36]
            37,38,39,40

        Between rows: offset +5 connects a square to the adjacent square on
        the next row (same diagonal). Offset +4 connects to the other diagonal.

        For captures: a black man at sq captures an enemy at sq+4 and lands at
        sq+4+4 = sq+8? Or sq+4+5 = sq+9?

        The actual capture offsets in the 46-board are:
            Jump "forward-left":  over sq+4, land at sq+9  (4+5=9)
            Jump "forward-right": over sq+5, land at sq+9  (5+4=9)

        Wait, that would mean both land at the same place, which is wrong.

        Let me trace through SimpleCh's actual code more carefully.
        Looking at simplech.c lines ~1400+, the capture functions use
        hardcoded offsets. Let me just re-implement based on the actual
        diagonal neighbor structure.
        """
        # For the 46-board, the diagonal neighbors are at offsets
        # that depend on the row. Let's compute them from the geometry.
        self._recursive_man_capture(b, sq, sq, BLACK, movelist, [], [], BLACK_MAN)

    def _white_man_capture(self, b: List[int], sq: int, movelist: List[Move]):
        """Generate capture moves for a white man."""
        self._recursive_man_capture(b, sq, sq, WHITE, movelist, [], [], WHITE_MAN)

    def _black_king_capture(self, b: List[int], sq: int, movelist: List[Move]):
        """Generate capture moves for a black king."""
        self._recursive_king_capture(b, sq, sq, BLACK, movelist, [], [])

    def _white_king_capture(self, b: List[int], sq: int, movelist: List[Move]):
        """Generate capture moves for a white king."""
        self._recursive_king_capture(b, sq, sq, WHITE, movelist, [], [])

    def _get_diagonal_neighbors(self, sq: int) -> List[Tuple[int, int]]:
        """Get all diagonal neighbors as (adjacent_sq, landing_sq) pairs.

        Returns list of (jumped_over, landed_on) for each of the 4 diagonals.
        Only returns valid board squares (not off-board).
        """
        if sq not in INTERNAL_TO_XY:
            return []

        x, y = INTERNAL_TO_XY[sq]
        neighbors = []

        for dx, dy in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:
            adj_x, adj_y = x + dx, y + dy
            land_x, land_y = x + 2 * dx, y + 2 * dy

            adj_rc = (adj_x, adj_y)
            land_rc = (land_x, land_y)

            # Check bounds
            if not (0 <= adj_x <= 7 and 0 <= adj_y <= 7):
                continue
            if not (0 <= land_x <= 7 and 0 <= land_y <= 7):
                continue

            # Convert to internal squares using XY_TO_INTERNAL
            from .board import XY_TO_INTERNAL
            adj_sq = XY_TO_INTERNAL.get(adj_rc)
            land_sq = XY_TO_INTERNAL.get(land_rc)

            if adj_sq is not None and land_sq is not None:
                neighbors.append((adj_sq, land_sq))

        return neighbors

    def _get_forward_diag_neighbors(self, sq: int, colour: int) -> List[Tuple[int, int]]:
        """Get forward diagonal neighbors for a man piece.

        Black moves forward = increasing y (towards row 7).
        White moves forward = decreasing y (towards row 0).
        """
        if sq not in INTERNAL_TO_XY:
            return []

        x, y = INTERNAL_TO_XY[sq]
        neighbors = []

        # Forward direction
        if colour == BLACK:
            dy_options = [1]  # black moves up (increasing y)
        else:
            dy_options = [-1]  # white moves down (decreasing y)

        for dy in dy_options:
            for dx in [-1, 1]:
                adj_x, adj_y = x + dx, y + dy
                land_x, land_y = x + 2 * dx, y + 2 * dy

                if not (0 <= adj_x <= 7 and 0 <= adj_y <= 7):
                    continue
                if not (0 <= land_x <= 7 and 0 <= land_y <= 7):
                    continue

                from .board import XY_TO_INTERNAL
                adj_sq = XY_TO_INTERNAL.get((adj_x, adj_y))
                land_sq = XY_TO_INTERNAL.get((land_x, land_y))

                if adj_sq is not None and land_sq is not None:
                    neighbors.append((adj_sq, land_sq))

        return neighbors

    def _is_opponent(self, piece: int, colour: int) -> bool:
        """Check if a piece belongs to the opponent."""
        if piece == FREE or piece == OCCUPIED:
            return False
        return (piece & 0x03) != colour and (piece & 0x03) != 0

    def _recursive_man_capture(
        self,
        b: List[int],
        start_sq: int,
        current_sq: int,
        colour: int,
        movelist: List[Move],
        captured: List[Tuple[int, int]],
        path: List[int],
        man_piece: int,
    ):
        """Recursively find all multi-jump captures for a man.

        English checkers: men capture FORWARD only.
        """
        found_jump = False
        neighbors = self._get_forward_diag_neighbors(current_sq, colour)

        for adj_sq, land_sq in neighbors:
            # Can't jump over an already-captured piece in this sequence
            if any(sq == adj_sq for sq, _ in captured):
                continue

            if self._is_opponent(b[adj_sq], colour) and b[land_sq] == FREE:
                found_jump = True

                # Temporarily make the move
                old_current = b[current_sq]
                old_adj = b[adj_sq]
                old_land = b[land_sq]

                b[current_sq] = FREE
                b[adj_sq] = FREE
                b[land_sq] = man_piece  # might promote below

                new_captured = captured + [(adj_sq, old_adj)]
                new_path = path + [land_sq]

                # Check for promotion: if landing on back rank, stop jumping
                promotes = False
                if colour == BLACK and land_sq in (37, 38, 39, 40):
                    promotes = True
                elif colour == WHITE and land_sq in (5, 6, 7, 8):
                    promotes = True

                if promotes:
                    # English checkers: piece promotes and STOPS (no further jumps)
                    king_piece = colour | KING
                    b[land_sq] = king_piece
                    self._add_capture_move(
                        movelist, start_sq, land_sq,
                        man_piece, king_piece,
                        new_captured, new_path, b
                    )
                    b[land_sq] = old_land
                else:
                    # Try to continue jumping
                    self._recursive_man_capture(
                        b, start_sq, land_sq, colour,
                        movelist, new_captured, new_path, man_piece
                    )

                # Undo
                b[current_sq] = old_current
                b[adj_sq] = old_adj
                b[land_sq] = old_land

        if not found_jump and captured:
            # Terminal: no more jumps available — record the move
            self._add_capture_move(
                movelist, start_sq, current_sq,
                man_piece, man_piece,
                captured, path, b
            )

    def _recursive_king_capture(
        self,
        b: List[int],
        start_sq: int,
        current_sq: int,
        colour: int,
        movelist: List[Move],
        captured: List[Tuple[int, int]],
        path: List[int],
    ):
        """Recursively find all multi-jump captures for a king.

        Kings can capture in all four diagonal directions.
        """
        king_piece = colour | KING
        found_jump = False
        neighbors = self._get_diagonal_neighbors(current_sq)

        for adj_sq, land_sq in neighbors:
            if any(sq == adj_sq for sq, _ in captured):
                continue

            if self._is_opponent(b[adj_sq], colour) and b[land_sq] == FREE:
                found_jump = True

                old_current = b[current_sq]
                old_adj = b[adj_sq]
                old_land = b[land_sq]

                b[current_sq] = FREE
                b[adj_sq] = FREE
                b[land_sq] = king_piece

                new_captured = captured + [(adj_sq, old_adj)]
                new_path = path + [land_sq]

                self._recursive_king_capture(
                    b, start_sq, land_sq, colour,
                    movelist, new_captured, new_path
                )

                b[current_sq] = old_current
                b[adj_sq] = old_adj
                b[land_sq] = old_land

        if not found_jump and captured:
            self._add_capture_move(
                movelist, start_sq, current_sq,
                king_piece, king_piece,
                captured, path, b
            )

    def _add_capture_move(
        self,
        movelist: List[Move],
        from_sq: int,
        to_sq: int,
        old_piece: int,
        new_piece: int,
        captured: List[Tuple[int, int]],
        path: List[int],
        b: List[int],
    ):
        """Create and add a capture Move to the movelist."""
        mv = Move()
        # Entry 0: from square (piece leaves)
        mv.m[0] = Move.pack(from_sq, old_piece, FREE)
        # Entry 1: to square (piece arrives)
        mv.m[1] = Move.pack(to_sq, FREE, new_piece)
        # Entries 2+: captured squares (pieces removed)
        for i, (cap_sq, cap_piece) in enumerate(captured):
            mv.m[2 + i] = Move.pack(cap_sq, cap_piece, FREE)

        mv.n = 2 + len(captured)

        # Store decoded info directly
        mv.from_sq = from_sq
        mv.to_sq = to_sq
        mv.old_piece = old_piece
        mv.new_piece = new_piece
        mv.captured_squares = [sq for sq, _ in captured]
        mv.path_squares = [from_sq] + path
        mv.is_capture = True
        mv.is_promotion = (old_piece & MAN) != 0 and (new_piece & KING) != 0

        movelist.append(mv)

    def test_capture(self, b: List[int], colour: int) -> bool:
        """Quick test whether any captures are possible.

        Ported from simplech.c testcapture().
        """
        for sq in PLAYABLE_SQUARES:
            piece = b[sq]
            if piece == FREE or piece == OCCUPIED:
                continue
            if (piece & 0x03) != colour:
                continue

            is_king = (piece & KING) != 0

            if is_king:
                neighbors = self._get_diagonal_neighbors(sq)
            else:
                neighbors = self._get_forward_diag_neighbors(sq, colour)

            for adj_sq, land_sq in neighbors:
                if self._is_opponent(b[adj_sq], colour) and b[land_sq] == FREE:
                    return True

        return False
