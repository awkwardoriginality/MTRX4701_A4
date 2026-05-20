"""
pick_place.py — Pick-and-place motion primitives for the UR5e.

Generates sequences of Cartesian waypoints for:
    - Picking a piece from a board square
    - Placing a piece at a board square
    - Bobbing down and up over a square (multi-jump traversal)
    - King promotion (stacking a piece on top)
    - Discarding captured pieces
    - Finger wag gesture (illegal move disapproval)

Each primitive is a generator yielding MotionCommand objects that the
manipulation node translates into MoveIt2 goals or direct joint commands.
"""

from __future__ import annotations
import numpy as np
from dataclasses import dataclass, field
from enum import Enum
from typing import Generator, List, Optional

from .board_coordinates import BoardCoordinates
from ..game_engine.board import INTERNAL_TO_ROWCOL, CB_BLACK, CB_WHITE


class MotionType(Enum):
    MOVE_TO = "MOVE_TO"
    MOVE_TO_JOINT = "MOVE_TO_JOINT"
    GRASP = "GRASP"         # Activate vacuum / close gripper
    RELEASE = "RELEASE"     # Deactivate vacuum / open gripper
    WAIT = "WAIT"           # Pause for a specified duration


@dataclass
class MotionCommand:
    """A single motion command for the UR5e."""

    motion_type: MotionType
    position: Optional[np.ndarray] = None   # Target Cartesian position [x, y, z]
    orientation: Optional[np.ndarray] = None  # Target orientation [qx, qy, qz, qw]
    speed: float = 0.2                       # Cartesian speed (m/s)
    acceleration: float = 0.5                # Cartesian acceleration (m/s²)
    wait_duration: float = 0.0               # For WAIT type (seconds)
    force_limit: float = 10.0                # Force limit for contact detection (N)
    label: str = ""                          # Human-readable description


class PickPlace:
    """Generates motion command sequences for checkers piece manipulation.

    Uses the BoardCoordinates to convert board squares to Cartesian positions,
    then generates sequences of MotionCommands for each manipulation primitive.
    """

    # Default speeds (m/s)
    TRANSIT_SPEED = 0.3     # Fast movement between positions
    APPROACH_SPEED = 0.1    # Slow approach for precision
    BOB_SPEED = 0.15        # Speed during bobbing

    # Default orientation: gripper pointing straight down
    # (rotation about z-axis may be adjusted for board alignment)
    DEFAULT_ORIENTATION = np.array([0.0, 1.0, 0.0, 0.0])  # 180° about x

    def __init__(self, board_coords: BoardCoordinates):
        self.coords = board_coords

    def pick_piece(
        self, row: int, col: int, is_king: bool = False
    ) -> List[MotionCommand]:
        """Generate commands to pick a piece from a board square.

        Sequence: Safe transit altitude → hover → approach → grasp
        → activate grasp → ascend to safe transit altitude.

        The safe transit altitude is high enough that even joint-space
        interpolation (used when MoveIt cannot plan a straight Cartesian path)
        cannot swing the arm below the table surface.
        """
        safe    = self.coords.safe_transit_position(row, col)
        hover   = self.coords.hover_position(row, col)
        approach = self.coords.approach_position(row, col)
        grasp   = self.coords.grasp_position(row, col, is_king)

        return [
            MotionCommand(
                MotionType.MOVE_TO, safe, self.DEFAULT_ORIENTATION,
                speed=self.TRANSIT_SPEED,
                label=f"Safe transit above ({row},{col})",
            ),
            MotionCommand(
                MotionType.MOVE_TO, hover, self.DEFAULT_ORIENTATION,
                speed=self.APPROACH_SPEED,
                label=f"Hover over ({row},{col})",
            ),
            MotionCommand(
                MotionType.MOVE_TO, approach, self.DEFAULT_ORIENTATION,
                speed=self.APPROACH_SPEED,
                label=f"Approach ({row},{col})",
            ),
            MotionCommand(
                MotionType.MOVE_TO, grasp, self.DEFAULT_ORIENTATION,
                speed=self.APPROACH_SPEED,
                label=f"Descend to grasp at ({row},{col})",
            ),
            MotionCommand(
                MotionType.GRASP,
                label="Activate grasp",
            ),
            MotionCommand(
                MotionType.WAIT, wait_duration=0.3,
                label="Wait for grasp to settle",
            ),
            MotionCommand(
                MotionType.MOVE_TO, safe, self.DEFAULT_ORIENTATION,
                speed=self.APPROACH_SPEED,
                label=f"Lift to safe altitude from ({row},{col})",
            ),
        ]

    def place_piece(
        self, row: int, col: int, is_king_stack: bool = False
    ) -> List[MotionCommand]:
        """Generate commands to place a piece at a board square.

        Sequence: Safe transit altitude → hover → approach → place
        → release → ascend to safe transit altitude.
        """
        safe    = self.coords.safe_transit_position(row, col)
        hover   = self.coords.hover_position(row, col)
        approach = self.coords.approach_position(row, col)

        if is_king_stack:
            place = self.coords.king_stack_position(row, col)
        else:
            place = self.coords.grasp_position(row, col)

        return [
            MotionCommand(
                MotionType.MOVE_TO, safe, self.DEFAULT_ORIENTATION,
                speed=self.TRANSIT_SPEED,
                label=f"Safe transit above ({row},{col}) for placement",
            ),
            MotionCommand(
                MotionType.MOVE_TO, hover, self.DEFAULT_ORIENTATION,
                speed=self.APPROACH_SPEED,
                label=f"Hover over ({row},{col}) for placement",
            ),
            MotionCommand(
                MotionType.MOVE_TO, approach, self.DEFAULT_ORIENTATION,
                speed=self.APPROACH_SPEED,
                label=f"Approach ({row},{col}) for placement",
            ),
            MotionCommand(
                MotionType.MOVE_TO, place, self.DEFAULT_ORIENTATION,
                speed=self.APPROACH_SPEED,
                label=f"Descend to place at ({row},{col})",
            ),
            MotionCommand(
                MotionType.RELEASE,
                label="Release piece",
            ),
            MotionCommand(
                MotionType.WAIT, wait_duration=0.2,
                label="Wait for release",
            ),
            MotionCommand(
                MotionType.MOVE_TO, safe, self.DEFAULT_ORIENTATION,
                speed=self.APPROACH_SPEED,
                label=f"Retreat to safe altitude from ({row},{col})",
            ),
        ]

    def bob_over_square(self, row: int, col: int) -> List[MotionCommand]:
        """Generate commands to bob down and up over a square.

        Used during multi-jump captures to show the piece's path.
        Sequence: Safe transit altitude → bob height → safe transit altitude.
        """
        safe = self.coords.safe_transit_position(row, col)
        bob  = self.coords.bob_position(row, col)

        return [
            MotionCommand(
                MotionType.MOVE_TO, safe, self.DEFAULT_ORIENTATION,
                speed=self.TRANSIT_SPEED,
                label=f"Safe transit to ({row},{col}) for bob",
            ),
            MotionCommand(
                MotionType.MOVE_TO, bob, self.DEFAULT_ORIENTATION,
                speed=self.BOB_SPEED,
                label=f"Bob down over ({row},{col})",
            ),
            MotionCommand(
                MotionType.MOVE_TO, safe, self.DEFAULT_ORIENTATION,
                speed=self.BOB_SPEED,
                label=f"Bob up from ({row},{col})",
            ),
        ]

    def pick_and_discard(
        self,
        row: int,
        col: int,
        discard_index: int,
        is_king: bool = False,
    ) -> List[MotionCommand]:
        """Generate commands to pick a captured piece and move to discard pile."""
        commands = self.pick_piece(row, col, is_king)

        # Move to discard position
        discard_pos = self.coords.discard_position(discard_index)
        discard_safe = discard_pos.copy()
        discard_safe[2] += self.coords.SAFE_TRANSIT_HEIGHT

        commands.extend([
            MotionCommand(
                MotionType.MOVE_TO, discard_safe, self.DEFAULT_ORIENTATION,
                speed=self.TRANSIT_SPEED,
                label=f"Safe transit to discard pile [{discard_index}]",
            ),
            MotionCommand(
                MotionType.MOVE_TO, discard_pos, self.DEFAULT_ORIENTATION,
                speed=self.APPROACH_SPEED,
                label=f"Lower to discard pile [{discard_index}]",
            ),
            MotionCommand(
                MotionType.RELEASE,
                label="Release captured piece",
            ),
            MotionCommand(
                MotionType.WAIT, wait_duration=0.2,
                label="Wait for release",
            ),
            MotionCommand(
                MotionType.MOVE_TO, discard_safe, self.DEFAULT_ORIENTATION,
                speed=self.APPROACH_SPEED,
                label="Retreat from discard pile",
            ),
        ])

        return commands

    def king_promotion(
        self,
        row: int,
        col: int,
        colour: int,
        reserve_index: int,
    ) -> List[MotionCommand]:
        """Generate commands for king promotion (stacking a piece).

        Sequence: Pick a reserve piece → place it on top of the promoted piece.
        """
        # Pick from reserve
        reserve_pos = self.coords.reserve_position(colour, reserve_index)
        reserve_safe = reserve_pos.copy()
        reserve_safe[2] += self.coords.SAFE_TRANSIT_HEIGHT

        commands = [
            MotionCommand(
                MotionType.MOVE_TO, reserve_safe, self.DEFAULT_ORIENTATION,
                speed=self.TRANSIT_SPEED,
                label="Safe transit to reserve pile",
            ),
            MotionCommand(
                MotionType.MOVE_TO, reserve_pos, self.DEFAULT_ORIENTATION,
                speed=self.APPROACH_SPEED,
                label="Descend to reserve piece",
            ),
            MotionCommand(
                MotionType.GRASP,
                label="Grasp reserve piece",
            ),
            MotionCommand(
                MotionType.WAIT, wait_duration=0.3,
                label="Wait for grasp",
            ),
            MotionCommand(
                MotionType.MOVE_TO, reserve_safe, self.DEFAULT_ORIENTATION,
                speed=self.APPROACH_SPEED,
                label="Lift reserve piece to safe altitude",
            ),
        ]

        # Place on top of the promoted piece
        commands.extend(self.place_piece(row, col, is_king_stack=True))

        return commands

    def finger_wag(self) -> List[MotionCommand]:
        """Generate commands for the finger-wag gesture.

        3 side-to-side oscillations above the board centre to indicate
        disapproval of an illegal move.
        """
        centre = self.coords.finger_wag_position()
        wag_amplitude = 0.04  # 4 cm side-to-side

        commands = [
            MotionCommand(
                MotionType.MOVE_TO, centre, self.DEFAULT_ORIENTATION,
                speed=self.TRANSIT_SPEED,
                label="Move to wag position",
            ),
        ]

        for i in range(3):
            left = centre.copy()
            left[0] -= wag_amplitude
            right = centre.copy()
            right[0] += wag_amplitude

            commands.extend([
                MotionCommand(
                    MotionType.MOVE_TO, left, self.DEFAULT_ORIENTATION,
                    speed=0.8,
                    label=f"Wag left [{i+1}]",
                ),
                MotionCommand(
                    MotionType.MOVE_TO, right, self.DEFAULT_ORIENTATION,
                    speed=0.8,
                    label=f"Wag right [{i+1}]",
                ),
            ])

        commands.append(
            MotionCommand(
                MotionType.MOVE_TO, centre, self.DEFAULT_ORIENTATION,
                speed=0.5,
                label="Return to centre after wag",
            )
        )

        return commands

    def execute_simple_move(
        self,
        from_row: int,
        from_col: int,
        to_row: int,
        to_col: int,
        is_king: bool = False,
    ) -> List[MotionCommand]:
        """Generate full command sequence for a simple (non-capture) move."""
        commands = self.pick_piece(from_row, from_col, is_king)
        commands.extend(self.place_piece(to_row, to_col))
        return commands

    def execute_capture_move(
        self,
        from_sq: int,
        to_sq: int,
        path_squares: List[int],
        captured_squares: List[int],
        discard_start_index: int,
        is_king: bool = False,
    ) -> List[MotionCommand]:
        """Generate full command sequence for a capture move.

        Includes: pick piece → bob over intermediate squares → place piece
        → pick up each captured piece → discard each.
        """
        from_row, from_col = INTERNAL_TO_ROWCOL[from_sq]
        to_row, to_col = INTERNAL_TO_ROWCOL[to_sq]

        # 1. Pick the moving piece
        commands = self.pick_piece(from_row, from_col, is_king)

        # 2. Bob over intermediate path squares (skip first = from, last = to)
        for i, sq in enumerate(path_squares):
            if sq == from_sq or sq == to_sq:
                continue
            bob_row, bob_col = INTERNAL_TO_ROWCOL[sq]
            commands.extend(self.bob_over_square(bob_row, bob_col))

        # 3. Place the piece at the destination
        commands.extend(self.place_piece(to_row, to_col))

        # 4. Pick up each captured piece and discard it
        for i, cap_sq in enumerate(captured_squares):
            cap_row, cap_col = INTERNAL_TO_ROWCOL[cap_sq]
            commands.extend(
                self.pick_and_discard(
                    cap_row, cap_col,
                    discard_start_index + i,
                )
            )

        return commands
