"""
board_coordinates.py — Maps checkers board squares to robot Cartesian coordinates.

Converts between:
    - Internal 46-board square numbers
    - Standard 1–32 checkers notation
    - (row, col) grid positions
    - Physical (x, y, z) Cartesian coordinates in the robot base frame

The physical board is an 8×8 grid with ArUco markers at corners.
Board-to-robot transform is obtained via hand-eye calibration (Tsai-Lenz).
"""

from __future__ import annotations
import numpy as np
from typing import Tuple, Optional, Dict

from ..game_engine.board import (
    INTERNAL_TO_ROWCOL, ROWCOL_TO_INTERNAL,
    INTERNAL_TO_STANDARD, STANDARD_TO_INTERNAL,
    PLAYABLE_SQUARES,
)


class BoardCoordinates:
    """Maps board squares to physical Cartesian coordinates.

    The coordinate system:
        - Board origin (0, 0) is at the corner nearest square 1 (black's
          bottom-right from their perspective).
        - x-axis runs along the board columns (left → right).
        - y-axis runs along the board rows (bottom → top, black → white).
        - z-axis points up from the board surface.

    The board-to-robot transform converts board-frame coordinates
    to the UR5e base frame.
    """

    # ─── Default geometry (configurable via YAML) ────────────────────────
    DEFAULT_SQUARE_SIZE = 0.050     # 50 mm per square
    DEFAULT_PIECE_HEIGHT = 0.020    # 20 mm piece height
    DEFAULT_KING_HEIGHT = 0.040     # 40 mm (stacked) king height

    # Heights for manipulation
    SAFE_TRANSIT_HEIGHT = 0.250     # 250 mm — transit altitude; keeps joint-space arcs above the floor
    HOVER_HEIGHT = 0.080            # 80 mm above board surface
    APPROACH_HEIGHT = 0.020         # 20 mm above board for slow approach
    BOB_HEIGHT = 0.025              # 25 mm above board during bobbing

    def __init__(
        self,
        board_to_robot_tf: np.ndarray = np.eye(4),
        square_size: float = DEFAULT_SQUARE_SIZE,
        piece_height: float = DEFAULT_PIECE_HEIGHT,
        king_height: float = DEFAULT_KING_HEIGHT,
    ):
        """
        Args:
            board_to_robot_tf: 4×4 homogeneous transform from board frame
                to robot base frame. Obtained from hand-eye calibration.
            square_size: Side length of one square in metres.
            piece_height: Height of a single checker piece in metres.
            king_height: Height of a stacked king (two pieces) in metres.
        """
        self.T = board_to_robot_tf
        self.sq = square_size
        self.piece_height = piece_height
        self.king_height = king_height

        # Discard pile configuration
        self._discard_offset_x = -0.06  # 6 cm to the left of the board
        self._discard_spacing = 0.035   # 35 mm between pieces in discard
        self._discard_start_y = 0.05    # Start 5 cm from board bottom edge

        # King reserve pile (pieces used for king promotion stacking)
        self._reserve_offset_x = -0.06
        self._reserve_start_y = 0.35    # Above the discard pile

    def square_to_cartesian(
        self,
        row: int,
        col: int,
        height_offset: float = 0.0,
    ) -> np.ndarray:
        """Convert a board (row, col) to Cartesian (x, y, z) in robot frame.

        The point is at the centre of the square, at the board surface
        plus any height offset.

        Args:
            row: Board row (0 = black's home, 7 = white's home).
            col: Board column (0–7).
            height_offset: Additional height above the board surface (metres).

        Returns:
            3-element position vector [x, y, z] in robot base frame.
        """
        # Centre of the square in board coordinates
        x_board = (col + 0.5) * self.sq
        y_board = (row + 0.5) * self.sq
        z_board = height_offset

        # Transform to robot frame
        p_board = np.array([x_board, y_board, z_board, 1.0])
        p_robot = self.T @ p_board
        return p_robot[:3]

    def internal_square_to_cartesian(
        self,
        sq: int,
        height_offset: float = 0.0,
    ) -> np.ndarray:
        """Convert an internal 46-board square to Cartesian coordinates."""
        row, col = INTERNAL_TO_ROWCOL[sq]
        return self.square_to_cartesian(row, col, height_offset)

    def safe_transit_position(self, row: int, col: int) -> np.ndarray:
        """Get the safe transit altitude above a square.

        Used as the first and last waypoint in every pick/place sequence so that
        all large inter-square transit moves stay at a high altitude.  At this
        height, even joint-space interpolation (used when GetCartesianPath fails)
        cannot swing the arm below the table surface.
        """
        return self.square_to_cartesian(row, col, self.SAFE_TRANSIT_HEIGHT)

    def hover_position(self, row: int, col: int) -> np.ndarray:
        """Get the hover position above a square (for transit)."""
        return self.square_to_cartesian(row, col, self.HOVER_HEIGHT)

    def approach_position(self, row: int, col: int) -> np.ndarray:
        """Get the slow approach position above a square."""
        return self.square_to_cartesian(row, col, self.APPROACH_HEIGHT)

    def grasp_position(self, row: int, col: int, is_king: bool = False) -> np.ndarray:
        """Get the grasp/release position at piece height."""
        h = self.king_height if is_king else self.piece_height
        return self.square_to_cartesian(row, col, h)

    def bob_position(self, row: int, col: int) -> np.ndarray:
        """Get the bob-down position during multi-jump traversal."""
        return self.square_to_cartesian(row, col, self.BOB_HEIGHT)

    def king_stack_position(self, row: int, col: int) -> np.ndarray:
        """Get the position for stacking a king piece on top of an existing piece."""
        return self.square_to_cartesian(row, col, self.piece_height + 0.001)

    def discard_position(self, discard_index: int) -> np.ndarray:
        """Get the position for placing a piece in the discard pile.

        Pieces are arranged in a line along the left side of the board.

        Args:
            discard_index: 0-based index of the piece in the discard pile.

        Returns:
            3-element position vector in robot base frame.
        """
        x_board = self._discard_offset_x
        y_board = self._discard_start_y + discard_index * self._discard_spacing
        z_board = self.piece_height  # Place at piece height

        p_board = np.array([x_board, y_board, z_board, 1.0])
        p_robot = self.T @ p_board
        return p_robot[:3]

    def reserve_position(self, colour: int, reserve_index: int) -> np.ndarray:
        """Get the position of a reserve piece (for king promotion).

        Reserve pieces are kept to the side of the board, available for
        placing on top of a promoted piece.

        Args:
            colour: CB_BLACK or CB_WHITE.
            reserve_index: 0-based index within this colour's reserve.

        Returns:
            3-element position vector in robot base frame.
        """
        # Separate columns for black and white reserves
        x_offset = self._reserve_offset_x if colour == 2 else (8 * self.sq + 0.06)
        y_board = self._reserve_start_y + reserve_index * self._discard_spacing

        p_board = np.array([x_offset, y_board, self.piece_height, 1.0])
        p_robot = self.T @ p_board
        return p_robot[:3]

    def board_center(self) -> np.ndarray:
        """Get the centre of the board in robot frame (for gestures)."""
        return self.square_to_cartesian(3, 3, self.HOVER_HEIGHT)

    def finger_wag_position(self) -> np.ndarray:
        """Get the position for the finger wag gesture."""
        return self.square_to_cartesian(3, 3, 0.15)  # 15 cm above board centre

    def all_square_positions(self) -> Dict[int, np.ndarray]:
        """Get Cartesian positions for all 32 playable squares.

        Returns:
            Dictionary mapping internal square number to position vector.
        """
        positions = {}
        for sq in PLAYABLE_SQUARES:
            positions[sq] = self.internal_square_to_cartesian(sq)
        return positions

    def set_transform(self, T: np.ndarray):
        """Update the board-to-robot transform (e.g., after recalibration)."""
        assert T.shape == (4, 4), "Transform must be 4×4"
        self.T = T.copy()

    def set_square_size(self, size: float):
        """Update the square size."""
        self.sq = size
