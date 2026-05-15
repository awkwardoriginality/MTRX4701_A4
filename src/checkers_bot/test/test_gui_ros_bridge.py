"""Focused tests for the GUI-to-ROS bridge support in `play_checkers.py`."""

import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from checkers_bot.game_engine.board import CB_BLACK
from play_checkers import CheckersGame


def test_sync_from_observed_board_applies_robot_move():
    """A board received back from ROS should reconcile as the robot's legal move."""
    game = CheckersGame(human_colour=CB_BLACK, ai_time=0.01)

    human_move = game.rules.get_legal_moves(game.board, game.human_colour)[0]
    game.apply_move(human_move)

    robot_move = game.rules.get_legal_moves(game.board, game.robot_colour)[0]
    observed_board = game.board.copy()
    observed_board.apply_move_inplace(robot_move)

    synced_move = game.sync_from_observed_board(observed_board, game.robot_colour)
    assert synced_move is not None
    assert synced_move.to_notation() == robot_move.to_notation()
    assert game.board.boards_match(observed_board)
    assert game.current_turn == game.human_colour
