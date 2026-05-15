"""
test_state_machine_integration.py - Focused tests for control-flow integration.

These tests avoid ROS runtime dependencies while validating the structured
protocol payloads, perception stability gating, and sequential robot command
dispatch used by the game manager and state machine.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from checkers_bot.game_engine.board import Board, CB_BLACK, CB_WHITE
from checkers_bot.nodes.game_manager_node import GameManagerNode
from checkers_bot.protocol import (
    BoardStateReport, GameStatus, ManipulationGoal,
    ManipulationResult, RobotInstruction,
)
from checkers_bot.state_machine.game_states import (
    GameContext, GameState, GameStateMachine, ManipulationCommand,
)


def _step_until(predicate, step_fn, limit=30):
    for _ in range(limit):
        step_fn()
        if predicate():
            return
    assert False, "Condition was not reached within the step limit"


def test_protocol_json_roundtrip():
    """Structured protocol payloads should serialise cleanly."""
    report = BoardStateReport(flat64=[0] * 64, stable_count=3, confidence=0.9)
    report_roundtrip = BoardStateReport.from_json(report.to_json())
    assert report_roundtrip.stable_count == 3
    assert report_roundtrip.confidence == 0.9

    goal = ManipulationGoal(command_id="cmd-7", command_type="MOVE_HOME")
    goal_roundtrip = ManipulationGoal.from_json(goal.to_json())
    assert goal_roundtrip.command_id == "cmd-7"
    assert goal_roundtrip.command_type == "MOVE_HOME"

    result = ManipulationResult(command_id="cmd-7", command_type="MOVE_HOME", success=True)
    result_roundtrip = ManipulationResult.from_json(result.to_json())
    assert result_roundtrip.success is True

    status = GameStatus(
        state="WAIT_HUMAN_MOVE",
        move_number=12,
        current_turn="black",
        board_summary="Black: 8 men, 1 kings | White: 7 men, 0 kings | Total: 16 pieces",
    )
    status_roundtrip = GameStatus.from_json(status.to_json())
    assert status_roundtrip.move_number == 12

    instruction = RobotInstruction(
        state="RETURN_HOME",
        summary="Robot returning home",
        move_number=12,
        current_turn="white",
        active_command="MOVE_HOME",
    )
    instruction_roundtrip = RobotInstruction.from_json(instruction.to_json())
    assert instruction_roundtrip.active_command == "MOVE_HOME"


def test_game_manager_requires_stable_board_before_update():
    """Standalone game manager should ignore single-frame board changes."""
    manager = GameManagerNode(use_ros2=False)
    board = Board()
    flat = board.to_flat64()

    manager._accept_stable_board(flat)
    assert manager.ctx.perceived_board is None

    manager._accept_stable_board(flat)
    assert manager.ctx.perceived_board is not None
    assert manager.ctx.perceived_board.boards_match(board)


def test_robot_commands_dispatch_sequentially_and_return_home():
    """Robot move execution should wait for each completion before sending the next command."""
    machine = GameStateMachine(search_time=0.01, human_colour=CB_WHITE)
    ctx = GameContext(human_colour=CB_WHITE, robot_colour=CB_BLACK)
    emitted: list[ManipulationCommand] = []

    machine.set_callbacks(on_manipulation_cmd=lambda cmd: emitted.append(cmd))

    _step_until(lambda: machine.state == GameState.EXECUTE_SIMPLE, lambda: machine.step(ctx))
    _step_until(lambda: len(emitted) == 1, lambda: machine.step(ctx))
    assert emitted[0].command_type == ManipulationCommand.Type.PICK

    machine.step(ctx)
    assert len(emitted) == 1, "Second command should not dispatch before completion acknowledgement"

    machine.notify_manipulation_done()
    _step_until(lambda: len(emitted) == 2, lambda: machine.step(ctx))
    assert emitted[1].command_type == ManipulationCommand.Type.PLACE

    machine.notify_manipulation_done()
    _step_until(lambda: any(cmd.command_type == ManipulationCommand.Type.MOVE_HOME for cmd in emitted), lambda: machine.step(ctx))
    assert emitted[-1].command_type == ManipulationCommand.Type.MOVE_HOME

    machine.notify_manipulation_done()
    _step_until(lambda: machine.state == GameState.WAIT_HUMAN_MOVE, lambda: machine.step(ctx))
    assert ctx.current_turn == ctx.human_colour
