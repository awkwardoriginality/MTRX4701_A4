"""
board.py — Board representation for English checkers.

Ported from KingsRow / SimpleCh (Martin Fierz).

Internal representation uses a 46-element array matching SimpleCh:
    Playable squares numbered 5–8, 10–13, 14–17, 19–22, 23–26, 28–31, 32–35, 37–40.
    Sentinel (off-board) squares are 0 (OCCUPIED).

Board layout (internal numbering):

        (white / top, rows 6–7)
      37  38  39  40          row 7
    32  33  34  35            row 6
      28  29  30  31          row 5
    23  24  25  26            row 4
      19  20  21  22          row 3
    14  15  16  17            row 2
      10  11  12  13          row 1
    5   6   7   8             row 0
        (black / bottom, rows 0–1)

Standard English checkers notation (1–32):

        (white)
      32  31  30  29          row 7
    28  27  26  25            row 6
      24  23  22  21          row 5
    20  19  18  17            row 4
      16  15  14  13          row 3
    12  11  10   9            row 2
       8   7   6   5         row 1
     4   3   2   1            row 0
        (black)
"""

from __future__ import annotations
from typing import List, Optional, Tuple
import copy

# ─── Piece Constants (matching cb_interface.h) ───────────────────────────────
CB_FREE = 0
CB_WHITE = 1
CB_BLACK = 2
CB_MAN = 4
CB_KING = 8

# Composite piece types
FREE = 16           # Empty playable square (SimpleCh convention)
OCCUPIED = 0        # Off-board sentinel
BLACK_MAN = CB_BLACK | CB_MAN      # 6
BLACK_KING = CB_BLACK | CB_KING    # 10
WHITE_MAN = CB_WHITE | CB_MAN     # 5
WHITE_KING = CB_WHITE | CB_KING   # 9
EMPTY = FREE

# Aliases used in the engine
BLACK = CB_BLACK
WHITE = CB_WHITE
MAN = CB_MAN
KING = CB_KING

# ─── Board8x8 ↔ Internal-46 Mapping ─────────────────────────────────────────
# From simplech.c getmove() lines 333–364:
#   board[i] = b[x][y]
# where b is Board8x8[8][8] — only dark squares are used.
#
# Row 0 (y=0): (0,0) (2,0) (4,0) (6,0) → squares 5,6,7,8
# Row 1 (y=1): (1,1) (3,1) (5,1) (7,1) → squares 10,11,12,13
# Row 2 (y=2): (0,2) (2,2) (4,2) (6,2) → squares 14,15,16,17
# Row 3 (y=3): (1,3) (3,3) (5,3) (7,3) → squares 19,20,21,22
# Row 4 (y=4): (0,4) (2,4) (4,4) (6,4) → squares 23,24,25,26
# Row 5 (y=5): (1,5) (3,5) (5,5) (7,5) → squares 28,29,30,31
# Row 6 (y=6): (0,6) (2,6) (4,6) (6,6) → squares 32,33,34,35
# Row 7 (y=7): (1,7) (3,7) (5,7) (7,7) → squares 37,38,39,40

# Internal square → (x, y) in Board8x8 coordinates
INTERNAL_TO_XY = {
    5:  (0, 0),  6:  (2, 0),  7:  (4, 0),  8:  (6, 0),
    10: (1, 1),  11: (3, 1),  12: (5, 1),  13: (7, 1),
    14: (0, 2),  15: (2, 2),  16: (4, 2),  17: (6, 2),
    19: (1, 3),  20: (3, 3),  21: (5, 3),  22: (7, 3),
    23: (0, 4),  24: (2, 4),  25: (4, 4),  26: (6, 4),
    28: (1, 5),  29: (3, 5),  30: (5, 5),  31: (7, 5),
    32: (0, 6),  33: (2, 6),  34: (4, 6),  35: (6, 6),
    37: (1, 7),  38: (3, 7),  39: (5, 7),  40: (7, 7),
}

# (x, y) → internal square
XY_TO_INTERNAL = {v: k for k, v in INTERNAL_TO_XY.items()}

# Internal square → standard checkers notation (1–32)
def _internal_to_standard(sq: int) -> int:
    """Convert internal 46-board square to standard 1–32 notation."""
    x, y = INTERNAL_TO_XY[sq]
    return 4 * (y + 1) - x // 2

def _standard_to_internal(num: int) -> int:
    """Convert standard 1–32 notation to internal 46-board square."""
    num0 = num - 1
    y = num0 // 4
    remainder = 3 - (num0 % 4)
    x = 2 * remainder
    if y % 2 == 1:
        x += 1
    return XY_TO_INTERNAL[(x, y)]

INTERNAL_TO_STANDARD = {sq: _internal_to_standard(sq) for sq in INTERNAL_TO_XY}
STANDARD_TO_INTERNAL = {v: k for k, v in INTERNAL_TO_STANDARD.items()}

# List of all 32 playable squares in internal numbering
PLAYABLE_SQUARES = sorted(INTERNAL_TO_XY.keys())

# Internal square → (row, col) for physical board addressing
# row 0 is black's home (bottom), row 7 is white's home (top)
# col 0 is leftmost from black's perspective
INTERNAL_TO_ROWCOL = {}
for sq, (x, y) in INTERNAL_TO_XY.items():
    INTERNAL_TO_ROWCOL[sq] = (y, x)  # row = y, col = x

ROWCOL_TO_INTERNAL = {v: k for k, v in INTERNAL_TO_ROWCOL.items()}


class Move:
    """Represents a single checkers move in internal representation.

    Mirrors the `move2` struct from SimpleCh (enginedefs.h):
        - n: number of entries in m[]
        - m[]: packed integers encoding square, piece-before, piece-after

    Also stores decoded information for manipulation planning:
        - from_sq, to_sq: source and destination squares (internal numbering)
        - captured_squares: list of squares where captured pieces were
        - path_squares: sequence of squares the piece travels through
        - old_piece, new_piece: piece type before and after the move
        - is_capture, is_promotion: boolean flags
    """

    def __init__(self):
        self.n: int = 0
        self.m: List[int] = [0] * 12

        # Decoded fields (populated after generation)
        self.from_sq: int = 0
        self.to_sq: int = 0
        self.captured_squares: List[int] = []
        self.path_squares: List[int] = []
        self.old_piece: int = 0
        self.new_piece: int = 0
        self.is_capture: bool = False
        self.is_promotion: bool = False

    @staticmethod
    def pack(square: int, piece_before: int, piece_after: int) -> int:
        """Pack square, before-piece, after-piece into one int."""
        return square | (piece_before << 8) | (piece_after << 16)

    @staticmethod
    def unpack(packed: int) -> Tuple[int, int, int]:
        """Unpack into (square, piece_before, piece_after)."""
        sq = packed & 0xFF
        before = (packed >> 8) & 0xFF
        after = (packed >> 16) & 0xFF
        return sq, before, after

    def decode(self):
        """Populate decoded fields from packed m[] array."""
        if self.n < 2:
            return

        from_sq, from_before, from_after = Move.unpack(self.m[0])
        to_sq, to_before, to_after = Move.unpack(self.m[1])

        self.from_sq = from_sq
        self.to_sq = to_sq
        self.old_piece = from_before
        self.new_piece = to_after

        # Captures are entries m[2] through m[n-1]
        self.captured_squares = []
        for i in range(2, self.n):
            cap_sq, cap_before, cap_after = Move.unpack(self.m[i])
            self.captured_squares.append(cap_sq)

        self.is_capture = len(self.captured_squares) > 0
        self.is_promotion = (self.old_piece & MAN) != 0 and (self.new_piece & KING) != 0

        # Build path (from → to, with intermediate points for multi-jumps)
        self.path_squares = [self.from_sq, self.to_sq]

    def copy(self) -> 'Move':
        mv = Move()
        mv.n = self.n
        mv.m = self.m[:]
        mv.from_sq = self.from_sq
        mv.to_sq = self.to_sq
        mv.captured_squares = self.captured_squares[:]
        mv.path_squares = self.path_squares[:]
        mv.old_piece = self.old_piece
        mv.new_piece = self.new_piece
        mv.is_capture = self.is_capture
        mv.is_promotion = self.is_promotion
        return mv

    def to_notation(self) -> str:
        """Convert to standard checkers notation like '11-15' or '7x14'."""
        if self.from_sq == 0 and self.to_sq == 0:
            self.decode()
        from_std = INTERNAL_TO_STANDARD.get(self.from_sq, self.from_sq)
        to_std = INTERNAL_TO_STANDARD.get(self.to_sq, self.to_sq)
        sep = 'x' if self.is_capture else '-'
        return f"{from_std}{sep}{to_std}"

    def __repr__(self) -> str:
        return f"Move({self.to_notation()})"


class Board:
    """English checkers board using the internal 46-square representation.

    Directly mirrors the `int b[46]` from SimpleCh.
    """

    def __init__(self):
        """Create a new board in the starting position."""
        self.b: List[int] = [OCCUPIED] * 46

        # Set all playable squares to FREE
        for sq in PLAYABLE_SQUARES:
            self.b[sq] = FREE

        # Re-set the sentinel squares (indices 9, 18, 27, 36 are also OCCUPIED)
        for i in (9, 18, 27, 36):
            self.b[i] = OCCUPIED

        # Initial position: black men on squares 1–12 (rows 0–2)
        # In internal numbering: 5–8, 10–13, 14–17
        for sq in [5, 6, 7, 8, 10, 11, 12, 13, 14, 15, 16, 17]:
            self.b[sq] = BLACK_MAN

        # White men on squares 21–32 (rows 5–7)
        # In internal numbering: 28–31, 32–35, 37–40
        for sq in [28, 29, 30, 31, 32, 33, 34, 35, 37, 38, 39, 40]:
            self.b[sq] = WHITE_MAN

    @classmethod
    def from_array(cls, b: List[int]) -> 'Board':
        """Create a board from a raw 46-element array."""
        board = cls.__new__(cls)
        board.b = b[:]
        return board

    @classmethod
    def from_board8x8(cls, b8: List[List[int]]) -> 'Board':
        """Create from an 8x8 array (Board8x8 format from cb_interface.h)."""
        board = cls.__new__(cls)
        board.b = [OCCUPIED] * 46

        # Set all playable squares to FREE first
        for sq in PLAYABLE_SQUARES:
            board.b[sq] = FREE
        for i in (9, 18, 27, 36):
            board.b[i] = OCCUPIED

        # Map Board8x8 to internal (from simplech.c getmove)
        for sq, (x, y) in INTERNAL_TO_XY.items():
            piece = b8[x][y] if b8[x][y] != 0 else FREE
            board.b[sq] = piece if piece != 0 else FREE

        return board

    def to_board8x8(self) -> List[List[int]]:
        """Convert to Board8x8[8][8] format."""
        b8 = [[0] * 8 for _ in range(8)]
        for sq in PLAYABLE_SQUARES:
            x, y = INTERNAL_TO_XY[sq]
            piece = self.b[sq]
            b8[x][y] = piece if piece != FREE else 0
        return b8

    def copy(self) -> 'Board':
        """Return a deep copy."""
        return Board.from_array(self.b)

    def get_piece(self, sq: int) -> int:
        """Get the piece at an internal square."""
        return self.b[sq]

    def set_piece(self, sq: int, piece: int):
        """Set the piece at an internal square."""
        self.b[sq] = piece

    def count_pieces(self, colour: int) -> Tuple[int, int]:
        """Count (men, kings) for a given colour (CB_BLACK or CB_WHITE)."""
        men = 0
        kings = 0
        for sq in PLAYABLE_SQUARES:
            p = self.b[sq]
            if p == (colour | MAN):
                men += 1
            elif p == (colour | KING):
                kings += 1
        return men, kings

    def all_pieces(self, colour: int) -> List[Tuple[int, int]]:
        """Return [(square, piece_type), ...] for all pieces of a colour."""
        pieces = []
        for sq in PLAYABLE_SQUARES:
            p = self.b[sq]
            if p != FREE and p != OCCUPIED and (p & 0x03) == colour:
                pieces.append((sq, p))
        return pieces

    def apply_move(self, move: Move) -> 'Board':
        """Apply a move and return a new board. Does not modify self."""
        new_board = self.copy()
        for i in range(move.n):
            sq, before, after = Move.unpack(move.m[i])
            new_board.b[sq] = after
        return new_board

    def apply_move_inplace(self, move: Move):
        """Apply a move in-place (modifies self)."""
        for i in range(move.n):
            sq, before, after = Move.unpack(move.m[i])
            self.b[sq] = after

    def undo_move_inplace(self, move: Move):
        """Undo a move in-place (modifies self)."""
        for i in range(move.n - 1, -1, -1):
            sq, before, after = Move.unpack(move.m[i])
            self.b[sq] = before

    def to_flat64(self) -> List[int]:
        """Convert to a flat 64-element array (row-major, for ROS messages).

        Index = row * 8 + col. Values:
            0 = empty
            5 = white man, 9 = white king
            6 = black man, 10 = black king
        """
        flat = [0] * 64
        for sq in PLAYABLE_SQUARES:
            row, col = INTERNAL_TO_ROWCOL[sq]
            idx = row * 8 + col
            piece = self.b[sq]
            flat[idx] = piece if piece != FREE else 0
        return flat

    @classmethod
    def from_flat64(cls, flat: List[int]) -> 'Board':
        """Create from a flat 64-element array."""
        board = cls.__new__(cls)
        board.b = [OCCUPIED] * 46
        for sq in PLAYABLE_SQUARES:
            board.b[sq] = FREE
        for i in (9, 18, 27, 36):
            board.b[i] = OCCUPIED

        for sq in PLAYABLE_SQUARES:
            row, col = INTERNAL_TO_ROWCOL[sq]
            idx = row * 8 + col
            piece = flat[idx]
            board.b[sq] = piece if piece != 0 else FREE
        return board

    def boards_match(self, other: 'Board') -> bool:
        """Check if two boards have identical piece positions."""
        for sq in PLAYABLE_SQUARES:
            if self.b[sq] != other.b[sq]:
                return False
        return True

    def diff(self, other: 'Board') -> List[Tuple[int, int, int]]:
        """Return [(square, self_piece, other_piece), ...] for differences."""
        diffs = []
        for sq in PLAYABLE_SQUARES:
            if self.b[sq] != other.b[sq]:
                diffs.append((sq, self.b[sq], other.b[sq]))
        return diffs

    def __str__(self) -> str:
        """Pretty-print the board."""
        piece_chars = {
            FREE: '.',
            BLACK_MAN: 'b',
            BLACK_KING: 'B',
            WHITE_MAN: 'w',
            WHITE_KING: 'W',
            OCCUPIED: ' ',
        }

        lines = []
        lines.append("    (white)")
        for row in range(7, -1, -1):
            cells = []
            for col in range(8):
                rc = (row, col)
                if rc in ROWCOL_TO_INTERNAL:
                    sq = ROWCOL_TO_INTERNAL[rc]
                    cells.append(piece_chars.get(self.b[sq], '?'))
                else:
                    cells.append(' ')

            # Indent even rows
            prefix = "  " if row % 2 == 0 else ""
            row_str = prefix + "  ".join(cells)
            std_nums = []
            for col in range(8):
                rc = (row, col)
                if rc in ROWCOL_TO_INTERNAL:
                    sq = ROWCOL_TO_INTERNAL[rc]
                    std_nums.append(str(INTERNAL_TO_STANDARD[sq]).rjust(2))
            lines.append(f"  {row_str}     [{', '.join(std_nums)}]")

        lines.append("    (black)")
        return "\n".join(lines)

    def __eq__(self, other) -> bool:
        if not isinstance(other, Board):
            return False
        return self.boards_match(other)

    def __hash__(self) -> int:
        return hash(tuple(self.b))
