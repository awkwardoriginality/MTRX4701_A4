"""
test_perception.py — Unit tests for the computer vision perception pipeline.

Tests:
    1. Instantiation of BoardPerception pipeline
    2. Classification of square ROIs (Empty, Red/Black Man, White Man)
    3. King differentiation via depth thresholds and concentric contours
    4. Integration with synthetic images

Run with:
    pytest test/test_perception.py -v
    or standalone:
    python3 test/test_perception.py
"""

import sys
import os
import numpy as np

# Add parent directory to path for standalone testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from checkers_bot.game_engine.board import (
    WHITE_MAN, WHITE_KING, BLACK_MAN, BLACK_KING, FREE,
)
from checkers_bot.nodes.perception_node import BoardPerception


def test_perception_initialization():
    """Verify that the perception pipeline initializes with valid ranges."""
    bp = BoardPerception(canonical_size=800)
    assert bp.size == 800
    assert bp.sq_size == 100
    assert 'red_low' in bp.hsv_ranges
    assert 'white' in bp.hsv_ranges
    print("✓ Perception initialization verified")


def test_classify_empty_square():
    """Test that an empty dark square is classified as 0."""
    bp = BoardPerception(canonical_size=800)
    # Synthetic dark brown background image (BGR)
    roi_bgr = np.full((100, 100, 3), (30, 50, 80), dtype=np.uint8)
    piece = bp.classify_square(roi_bgr)
    assert piece == 0, f"Expected empty square (0), got {piece}"
    print("✓ Empty square classification correct")


def test_classify_red_man():
    """Test classification of a Red/Black piece ROI."""
    bp = BoardPerception(canonical_size=800)
    # Synthetic image with a bright red circle in the center
    roi_bgr = np.full((100, 100, 3), (30, 50, 80), dtype=np.uint8)
    import cv2
    cv2.circle(roi_bgr, (50, 50), 30, (0, 0, 200), -1)  # BGR Red

    piece = bp.classify_square(roi_bgr)
    assert piece == BLACK_MAN, f"Expected BLACK_MAN ({BLACK_MAN}), got {piece}"
    print("✓ Red/Black Man classification correct")


def test_classify_white_man():
    """Test classification of a White piece ROI."""
    bp = BoardPerception(canonical_size=800)
    roi_bgr = np.full((100, 100, 3), (30, 50, 80), dtype=np.uint8)
    import cv2
    cv2.circle(roi_bgr, (50, 50), 30, (240, 240, 240), -1)  # BGR White

    piece = bp.classify_square(roi_bgr)
    assert piece == WHITE_MAN, f"Expected WHITE_MAN ({WHITE_MAN}), got {piece}"
    print("✓ White Man classification correct")


def test_classify_king_via_depth():
    """Test King classification using average depth thresholds."""
    bp = BoardPerception(canonical_size=800)
    roi_bgr = np.full((100, 100, 3), (30, 50, 80), dtype=np.uint8)
    import cv2
    cv2.circle(roi_bgr, (50, 50), 30, (0, 0, 200), -1)  # BGR Red

    # Synthetic depth map where the piece region sits at Z = 12.5 mm
    roi_depth = np.zeros((100, 100), dtype=np.float32)
    cv2.circle(roi_depth, (50, 50), 30, 12.5, -1)

    piece = bp.classify_square(roi_bgr, roi_depth=roi_depth)
    assert piece == BLACK_KING, f"Expected BLACK_KING ({BLACK_KING}), got {piece}"
    print("✓ King classification via depth correct")


if __name__ == '__main__':
    print("=" * 60)
    print("PERCEPTION PIPELINE TESTS")
    print("=" * 60)

    test_perception_initialization()
    test_classify_empty_square()
    test_classify_red_man()
    test_classify_white_man()
    test_classify_king_via_depth()

    print("\n" + "=" * 60)
    print("ALL PERCEPTION TESTS PASSED ✓")
    print("=" * 60)
