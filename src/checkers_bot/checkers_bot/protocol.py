"""
protocol.py - Structured JSON payloads for checkers ROS topic exchange.

The package currently uses `ament_python`, so a dedicated ROS interface
package is not available yet for generated `.msg` / `.action` types.
These dataclasses provide a stable schema for action-like goal, feedback,
and result topics while keeping the package lightweight and testable.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
import json
from typing import Any, Dict, List, Optional


@dataclass
class JsonMessage:
    """Simple JSON serialisation helpers shared by all protocol messages."""

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

    def to_json(self) -> str:
        return json.dumps(self.to_dict(), sort_keys=True)

    @classmethod
    def from_json(cls, payload: str):
        return cls(**json.loads(payload))


@dataclass
class BoardStateReport(JsonMessage):
    """Board state emitted by perception after stability filtering."""

    flat64: List[int]
    stable_count: int = 1
    hand_present: bool = False
    confidence: float = 1.0
    source: str = "perception"


@dataclass
class RobotInstruction(JsonMessage):
    """Human-readable robot instruction stream for debugging and dashboards."""

    state: str
    summary: str
    move_number: int
    current_turn: str
    active_command: Optional[str] = None
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class GameStatus(JsonMessage):
    """Compact snapshot of the game manager state."""

    state: str
    move_number: int
    current_turn: str
    board_summary: str
    legal_move: Optional[str] = None
    winner: Optional[str] = None


@dataclass
class ManipulationGoal(JsonMessage):
    """Action-like goal topic payload sent to the manipulation node."""

    command_id: str
    command_type: str
    square: Optional[int] = None
    row: int = 0
    col: int = 0
    is_king_stack: bool = False
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class ManipulationFeedback(JsonMessage):
    """Action-like feedback topic payload from the manipulation node."""

    command_id: str
    command_type: str
    stage: str
    detail: str = ""


@dataclass
class ManipulationResult(JsonMessage):
    """Action-like result topic payload from the manipulation node."""

    command_id: str
    command_type: str
    success: bool
    detail: str = ""
