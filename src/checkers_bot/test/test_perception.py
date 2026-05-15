"""Unit tests for the integrated perception adapter."""

import sys
import os
import numpy as np
import cv2

# Add package roots for standalone testing
CHECKERS_BOT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
PERCEPTION_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'perception'))
sys.path.insert(0, CHECKERS_BOT_ROOT)
sys.path.insert(0, PERCEPTION_ROOT)

from checkers_bot.game_engine.board import (
    WHITE_MAN, WHITE_KING, BLACK_MAN, BLACK_KING,
)
from perception.checkers_perception_node import CheckersPerceptionNode


def _make_adapter():
    """Create a non-ROS instance exposing the pure helper methods for tests."""
    adapter = CheckersPerceptionNode.__new__(CheckersPerceptionNode)
    adapter.green_lower = np.array([45, 80, 80], dtype=np.uint8)
    adapter.green_upper = np.array([85, 255, 255], dtype=np.uint8)
    adapter.purple_lower = np.array([105, 50, 50], dtype=np.uint8)
    adapter.purple_upper = np.array([145, 255, 255], dtype=np.uint8)
    adapter.piece_min_ratio = 0.18
    adapter.king_height_threshold_mm = 9.0
    adapter.green_man_value = BLACK_MAN
    adapter.green_king_value = BLACK_KING
    adapter.purple_man_value = WHITE_MAN
    adapter.purple_king_value = WHITE_KING
    adapter.cell_size = 100
    adapter.top_row_is_white = True
    return adapter


def test_perception_initialization():
    """Verify the test adapter exposes the canonical piece mapping."""
    adapter = _make_adapter()
    assert adapter.green_man_value == BLACK_MAN
    assert adapter.green_king_value == BLACK_KING
    assert adapter.purple_man_value == WHITE_MAN
    assert adapter.purple_king_value == WHITE_KING


def test_classify_empty_square():
    """Test that an empty dark square is classified as 0."""
    adapter = _make_adapter()
    roi_bgr = np.full((100, 100, 3), (30, 50, 80), dtype=np.uint8)
    roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    piece, label = adapter.classify_square(roi_hsv, roi_bgr, roi_depth=None)
    assert piece == 0
    assert label == "."


def test_classify_green_man():
    """Test classification of a green piece ROI into canonical black-man semantics."""
    adapter = _make_adapter()
    roi_bgr = np.full((100, 100, 3), (30, 50, 80), dtype=np.uint8)
    cv2.circle(roi_bgr, (50, 50), 30, (0, 255, 0), -1)
    roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    piece, label = adapter.classify_square(roi_hsv, roi_bgr, roi_depth=None)
    assert piece == BLACK_MAN
    assert label == "G"


def test_classify_purple_man():
    """Test classification of a purple piece ROI into canonical white-man semantics."""
    adapter = _make_adapter()
    roi_bgr = np.full((100, 100, 3), (30, 50, 80), dtype=np.uint8)
    cv2.circle(roi_bgr, (50, 50), 30, (255, 0, 255), -1)
    roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    piece, label = adapter.classify_square(roi_hsv, roi_bgr, roi_depth=None)
    assert piece == WHITE_MAN
    assert label == "P"


def test_classify_king_via_depth():
    """Test king classification using the configured depth threshold."""
    adapter = _make_adapter()
    roi_bgr = np.full((100, 100, 3), (30, 50, 80), dtype=np.uint8)
    cv2.circle(roi_bgr, (50, 50), 30, (0, 255, 0), -1)
    roi_hsv = cv2.cvtColor(roi_bgr, cv2.COLOR_BGR2HSV)
    roi_depth = np.zeros((100, 100), dtype=np.float32)
    cv2.circle(roi_depth, (50, 50), 30, 12.5, -1)
    piece, label = adapter.classify_square(roi_hsv, roi_bgr, roi_depth=roi_depth)
    assert piece == BLACK_KING
    assert label == "GK"


def test_board_to_flat64_respects_orientation():
    """Top-row board coordinates should map into canonical bottom-origin flat64 form."""
    adapter = _make_adapter()
    board = np.zeros((8, 8), dtype=np.uint8)
    board[0, 1] = WHITE_MAN
    board[7, 6] = BLACK_MAN
    flat64 = adapter.board_to_flat64(board)
    assert flat64[7 * 8 + 1] == WHITE_MAN
    assert flat64[0 * 8 + 6] == BLACK_MAN


if __name__ == '__main__':
    test_perception_initialization()
    test_classify_empty_square()
    test_classify_green_man()
    test_classify_purple_man()
    test_classify_king_via_depth()
    test_board_to_flat64_respects_orientation()
