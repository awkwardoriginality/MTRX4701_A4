"""ROS-facing contract tests for the integrated perception/game workflow."""

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from checkers_bot.game_engine.board import Board
from checkers_bot.nodes.game_manager_node import GameManagerNode
from checkers_bot.protocol import (
    BoardStateReport,
    GameStatus,
    ManipulationFeedback,
    ManipulationGoal,
    ManipulationResult,
    RobotInstruction,
)


class _Msg:
    def __init__(self, data):
        self.data = data


def test_board_state_report_json_contract():
    """Perception reports should serialise the fields the game manager now relies on."""
    report = BoardStateReport(
        flat64=[0] * 64,
        stable_count=4,
        board_blocked=True,
        confidence=0.85,
        encoding="flat64-piece-codes",
        source="perception_adapter",
    )
    payload = json.loads(report.to_json())
    assert payload["stable_count"] == 4
    assert payload["board_blocked"] is True
    assert payload["encoding"] == "flat64-piece-codes"


def test_game_manager_ignores_blocked_board_messages():
    """Blocked-board signals should prevent stale perception boards from reaching the state machine."""
    manager = GameManagerNode(use_ros2=False)
    board = Board()
    flat = board.to_flat64()

    manager._board_blocked_callback(_Msg(True))
    manager._board_state_callback(_Msg(flat))
    assert manager.ctx.perceived_board is None

    manager._board_blocked_callback(_Msg(False))
    manager._board_state_callback(_Msg(flat))
    manager._board_state_callback(_Msg(flat))
    assert manager.ctx.perceived_board is not None
    assert manager.ctx.perceived_board.boards_match(board)


def test_game_manager_uses_structured_report_blocking():
    """Structured board reports should clear perceived state when the board is blocked."""
    manager = GameManagerNode(use_ros2=False)
    board = Board()
    flat = board.to_flat64()

    clear_report = BoardStateReport(flat64=flat, stable_count=3, board_blocked=False)
    manager._board_state_report_callback(_Msg(clear_report.to_json()))
    assert manager.ctx.perceived_board is not None

    blocked_report = BoardStateReport(flat64=flat, stable_count=3, board_blocked=True)
    manager._board_state_report_callback(_Msg(blocked_report.to_json()))
    assert manager.ctx.perceived_board is None


def test_manipulation_topic_payload_roundtrips():
    """Action-like manipulation topic payloads should remain stable."""
    goal = ManipulationGoal(
        command_id="cmd-3",
        command_type="MOVE_HOME",
        metadata={"reason": "safe_idle"},
    )
    feedback = ManipulationFeedback(
        command_id="cmd-3",
        command_type="MOVE_HOME",
        stage="executing",
        detail="Returning to default pose",
    )
    result = ManipulationResult(
        command_id="cmd-3",
        command_type="MOVE_HOME",
        success=True,
        detail="Returned home",
    )

    assert ManipulationGoal.from_json(goal.to_json()).metadata["reason"] == "safe_idle"
    assert ManipulationFeedback.from_json(feedback.to_json()).stage == "executing"
    assert ManipulationResult.from_json(result.to_json()).success is True


def test_status_and_instruction_payloads_roundtrip():
    """Game status and instruction topics should stay parseable for dashboards and adapters."""
    status = GameStatus(
        state="WAIT_HUMAN_MOVE",
        move_number=7,
        current_turn="black",
        board_summary="Black: 8 men | White: 8 men",
    )
    instruction = RobotInstruction(
        state="RETURN_HOME",
        summary="Robot returning to safe idle pose",
        move_number=7,
        current_turn="white",
        active_command="MOVE_HOME",
        metadata={"command_id": "cmd-7"},
    )

    assert GameStatus.from_json(status.to_json()).state == "WAIT_HUMAN_MOVE"
    restored_instruction = RobotInstruction.from_json(instruction.to_json())
    assert restored_instruction.active_command == "MOVE_HOME"
    assert restored_instruction.metadata["command_id"] == "cmd-7"
