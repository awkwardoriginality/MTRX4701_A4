"""
test_state_machine_integration.py - Focused tests for control-flow integration.

These tests avoid ROS runtime dependencies while validating the structured
protocol payloads, perception stability gating, and sequential robot command
dispatch used by the game manager and state machine.
"""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from checkers_bot.game_engine.board import (
    Board, CB_BLACK, CB_WHITE, BLACK_MAN, WHITE_MAN, FREE, PLAYABLE_SQUARES,
)
from checkers_bot.nodes.game_manager_node import GameManagerNode
from checkers_bot.protocol import (
    BoardStateReport, GameStatus, ManipulationGoal,
    ManipulationResult, RobotInstruction,
)
from checkers_bot.state_machine.game_states import (
    GameContext, GameState, GameStateMachine, ManipulationCommand,
)
from play_checkers import CheckersGame


def _step_until(predicate, step_fn, limit=30):
    for _ in range(limit):
        step_fn()
        if predicate():
            return
    assert False, "Condition was not reached within the step limit"


def _empty_board() -> Board:
    board = Board()
    for sq in PLAYABLE_SQUARES:
        board.set_piece(sq, FREE)
    return board


def _configure_manager_position(manager: GameManagerNode, board: Board, current_turn: int):
    manager.ctx.board = board.copy()
    manager.ctx.previous_board = None
    manager.ctx.perceived_board = None
    manager.ctx.current_turn = current_turn
    manager.ctx.move_number = 0
    manager.ctx.move_history = []
    manager.ctx.game_over = False
    manager.ctx.winner = None
    manager.ctx.error_message = ""
    manager.ctx.command_queue = []
    manager.ctx.command_continuation = None
    manager.ctx.next_state_after_home = None
    manager.state_machine.state = GameState.WAIT_HUMAN_MOVE if current_turn == manager.ctx.human_colour else GameState.PLAN_ROBOT_MOVE
    manager._pending_commands.clear()
    manager._stable_board_data = None
    manager._stable_board_count = 0


def _play_robot_cycle(manager: GameManagerNode, gui_game: CheckersGame, limit: int = 400):
    for _ in range(limit):
        manager.step()

        if (
            gui_game.current_turn == gui_game.robot_colour
            and not gui_game.board.boards_match(manager.ctx.board)
        ):
            synced_move = gui_game.sync_from_observed_board(manager.ctx.board.copy(), gui_game.robot_colour)
            assert synced_move is not None, "GUI bridge could not reconcile the robot move"

        for _cmd in manager.get_pending_commands():
            manager.state_machine.notify_manipulation_done()

        if manager.state_machine.state in {GameState.WAIT_HUMAN_MOVE, GameState.GAME_OVER, GameState.ERROR}:
            if manager.state_machine._manipulation_done and not manager._pending_commands:
                return

    assert False, "Robot cycle did not settle within the step limit"


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


def test_manipulation_failure_transitions_manager_to_error():
    """A failed manipulation result should stop the workflow instead of continuing."""
    manager = GameManagerNode(use_ros2=False)
    manager.state_machine.step(manager.ctx)  # INIT -> WAIT_HUMAN_MOVE

    manager.process_manipulation_result(
        ManipulationResult(
            command_id="cmd-99",
            command_type="PLACE",
            success=False,
            detail="Simulated gripper failure",
        )
    )

    assert manager.state_machine.state == GameState.ERROR
    assert manager.ctx.error_message == "Simulated gripper failure"


def test_no_camera_workflow_survives_multiple_turns():
    """The GUI bridge workflow should stay in sync across multiple human/robot turns."""
    manager = GameManagerNode(search_time=0.01, human_colour=CB_BLACK, use_ros2=False)
    gui_game = CheckersGame(human_colour=CB_BLACK, ai_time=0.01)

    manager.state_machine.step(manager.ctx)  # INIT -> WAIT_HUMAN_MOVE

    for _turn in range(4):
        assert manager.state_machine.state == GameState.WAIT_HUMAN_MOVE
        assert gui_game.current_turn == gui_game.human_colour

        human_move = gui_game.rules.get_legal_moves(gui_game.board, gui_game.human_colour)[0]
        gui_game.apply_move(human_move)
        manager.ingest_board_report(
            BoardStateReport(
                flat64=gui_game.board.to_flat64(),
                stable_count=2,
                source="test_gui_bridge",
            )
        )

        _play_robot_cycle(manager, gui_game)

        assert manager.state_machine.state != GameState.ERROR
        assert gui_game.board.boards_match(manager.ctx.board)
        if manager.state_machine.state == GameState.GAME_OVER:
            break


def test_no_camera_workflow_reaches_game_over_from_short_endgame():
    """A reduced endgame should still complete cleanly through the GUI-style workflow."""
    manager = GameManagerNode(search_time=0.01, human_colour=CB_BLACK, use_ros2=False)
    gui_game = CheckersGame(human_colour=CB_BLACK, ai_time=0.01)

    board = _empty_board()
    board.set_piece(10, BLACK_MAN)
    board.set_piece(20, WHITE_MAN)

    gui_game.board = board.copy()
    gui_game.board_history = [gui_game.board.copy()]
    gui_game.current_turn = CB_BLACK
    gui_game.move_number = 0
    gui_game.move_history = []
    gui_game.last_move = None

    _configure_manager_position(manager, board, CB_BLACK)

    finishing_move = None
    for move in gui_game.rules.get_legal_moves(gui_game.board, gui_game.human_colour):
        candidate_gui = CheckersGame(human_colour=CB_BLACK, ai_time=0.01)
        candidate_gui.board = board.copy()
        candidate_gui.board_history = [candidate_gui.board.copy()]
        candidate_gui.current_turn = CB_BLACK
        candidate_gui.move_number = 0
        candidate_gui.move_history = []
        candidate_gui.last_move = None
        candidate_gui.apply_move(move)

        candidate_manager = GameManagerNode(search_time=0.01, human_colour=CB_BLACK, use_ros2=False)
        _configure_manager_position(candidate_manager, board, CB_BLACK)
        candidate_manager.ingest_board_report(
            BoardStateReport(
                flat64=candidate_gui.board.to_flat64(),
                stable_count=2,
                source="test_gui_bridge",
            )
        )
        _play_robot_cycle(candidate_manager, candidate_gui)
        if candidate_manager.state_machine.state == GameState.GAME_OVER:
            finishing_move = move
            break

    assert finishing_move is not None, "Expected at least one human move to produce a complete endgame"

    gui_game.apply_move(finishing_move)
    manager.ingest_board_report(
        BoardStateReport(
            flat64=gui_game.board.to_flat64(),
            stable_count=2,
            source="test_gui_bridge",
        )
    )
    _play_robot_cycle(manager, gui_game)

    assert manager.state_machine.state == GameState.GAME_OVER
    assert gui_game.board.boards_match(manager.ctx.board)
    assert manager.ctx.winner is not None
