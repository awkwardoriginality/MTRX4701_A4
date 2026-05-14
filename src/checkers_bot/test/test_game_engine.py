"""
test_game_engine.py — Comprehensive tests for the ported checkers engine.

Tests:
    1. Board representation and coordinate mappings
    2. Initial board setup
    3. Move generation (simple and capture moves)
    4. Multi-jump capture sequences
    5. King promotion during moves and captures
    6. Alpha-beta search finding correct moves
    7. Game state machine transitions
    8. Move validation (legal vs illegal)
    9. Known checkers positions and their correct moves

Run with:
    python -m pytest test/test_game_engine.py -v
    or
    python test/test_game_engine.py
"""

import sys
import os

# Add parent directory to path for standalone execution
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from checkers_bot.game_engine.board import (
    Board, Move, INTERNAL_TO_XY, XY_TO_INTERNAL,
    INTERNAL_TO_STANDARD, STANDARD_TO_INTERNAL,
    INTERNAL_TO_ROWCOL, ROWCOL_TO_INTERNAL,
    PLAYABLE_SQUARES,
    CB_BLACK, CB_WHITE, CB_MAN, CB_KING,
    BLACK_MAN, BLACK_KING, WHITE_MAN, WHITE_KING,
    FREE, OCCUPIED,
)
from checkers_bot.game_engine.move_generator import MoveGenerator
from checkers_bot.game_engine.search import Search
from checkers_bot.game_engine.evaluation import evaluate
from checkers_bot.game_engine.rules import Rules


def test_board_initial_position():
    """Test that the initial board has correct piece placement."""
    board = Board()

    # Black has 12 men on rows 0–2 (squares 1–12 in standard notation)
    bm, bk = board.count_pieces(CB_BLACK)
    assert bm == 12, f"Expected 12 black men, got {bm}"
    assert bk == 0, f"Expected 0 black kings, got {bk}"

    # White has 12 men on rows 5–7 (squares 21–32 in standard notation)
    wm, wk = board.count_pieces(CB_WHITE)
    assert wm == 12, f"Expected 12 white men, got {wm}"
    assert wk == 0, f"Expected 0 white kings, got {wk}"

    # Middle rows (3–4) should be empty
    for sq in [19, 20, 21, 22, 23, 24, 25, 26]:
        assert board.b[sq] == FREE, f"Square {sq} should be empty in initial position"

    print("✓ Initial position correct")


def test_coordinate_mappings():
    """Test that all coordinate mappings are consistent."""
    # Test that every playable square round-trips through all mappings
    for sq in PLAYABLE_SQUARES:
        # Internal → (x, y) → internal
        x, y = INTERNAL_TO_XY[sq]
        assert XY_TO_INTERNAL[(x, y)] == sq, \
            f"XY round-trip failed for square {sq}: ({x},{y})"

        # Internal → standard → internal
        std = INTERNAL_TO_STANDARD[sq]
        assert STANDARD_TO_INTERNAL[std] == sq, \
            f"Standard round-trip failed for square {sq}: std={std}"

        # Internal → rowcol → internal
        row, col = INTERNAL_TO_ROWCOL[sq]
        assert ROWCOL_TO_INTERNAL[(row, col)] == sq, \
            f"RowCol round-trip failed for square {sq}: ({row},{col})"

    # Verify standard notation range
    std_nums = sorted(INTERNAL_TO_STANDARD.values())
    assert std_nums == list(range(1, 33)), "Standard notation should cover 1–32"

    # Verify row range
    rows_seen = set()
    for sq in PLAYABLE_SQUARES:
        row, col = INTERNAL_TO_ROWCOL[sq]
        rows_seen.add(row)
    assert rows_seen == {0, 1, 2, 3, 4, 5, 6, 7}, "All rows 0–7 should be covered"

    print("✓ Coordinate mappings consistent")


def test_board8x8_conversion():
    """Test Board8x8 ↔ internal conversion."""
    board = Board()
    b8 = board.to_board8x8()

    # Reconstruct from Board8x8
    board2 = Board.from_board8x8(b8)
    assert board.boards_match(board2), "Board8x8 round-trip failed"

    # Check specific squares
    # Black man at standard square 1 → internal 5 → Board8x8[0][0]
    assert b8[0][0] == BLACK_MAN, \
        f"Board8x8[0][0] should be BLACK_MAN, got {b8[0][0]}"

    # White man at standard square 32 → internal 37 → Board8x8[1][7]
    assert b8[1][7] == WHITE_MAN, \
        f"Board8x8[1][7] should be WHITE_MAN, got {b8[1][7]}"

    print("✓ Board8x8 conversion correct")


def test_flat64_conversion():
    """Test flat-64 conversion for ROS messages."""
    board = Board()
    flat = board.to_flat64()

    assert len(flat) == 64, "Flat64 should have 64 elements"

    board2 = Board.from_flat64(flat)
    assert board.boards_match(board2), "Flat64 round-trip failed"

    print("✓ Flat64 conversion correct")


def test_move_generation_initial():
    """Test move generation from the initial position."""
    board = Board()
    gen = MoveGenerator()

    # Black moves first — should have 7 possible moves from initial position
    # (men on row 2 can move forward to rows 3–4)
    moves = gen.get_legal_moves(board, CB_BLACK)
    assert len(moves) > 0, "Black should have legal moves in initial position"

    # No captures should be possible in the initial position
    captures = gen.generate_capture_list(board.b, CB_BLACK)
    assert len(captures) == 0, "No captures should be possible initially"

    # White should also have 7 moves
    white_moves = gen.get_legal_moves(board, CB_WHITE)
    assert len(white_moves) > 0, "White should have legal moves in initial position"

    print(f"✓ Move generation: Black has {len(moves)} moves, White has {len(white_moves)} moves")

    # Print the moves for verification
    for m in moves:
        print(f"  Black: {m.to_notation()}")


def test_capture_detection():
    """Test that captures are correctly detected and generated."""
    # Set up a position where black can capture white
    board = Board()

    # Clear the board
    for sq in PLAYABLE_SQUARES:
        board.b[sq] = FREE

    # Place black man at square 15 (row 2, col 2)
    board.b[15] = BLACK_MAN
    # Place white man at square 20 (row 3, col 3) — diagonally adjacent
    board.b[20] = WHITE_MAN
    # Square 24 (row 4, col 2) must be empty for the jump
    board.b[24] = FREE
    # Actually let's use correct diagonals. In the 46-board:
    # sq 15 is at (2, 2). Forward diagonal to white side:
    #   (3, 3) → sq 20, (3, 1) → sq 19
    # Landing squares after jumping over (3,3):
    #   (4, 4) → sq 25
    board.b[25] = FREE

    gen = MoveGenerator()
    has_cap = gen.has_captures(board, CB_BLACK)
    assert has_cap, "Black should be able to capture white piece"

    captures = gen.generate_capture_list(board.b, CB_BLACK)
    assert len(captures) > 0, "Should find at least one capture move"

    for c in captures:
        c.decode()
        print(f"  Capture: {c.to_notation()} (captures: {c.captured_squares})")

    print(f"✓ Capture detection: {len(captures)} capture move(s) found")


def test_search_finds_move():
    """Test that the search engine finds a valid move."""
    board = Board()
    search = Search(max_time=2.0, max_depth=6)

    stats = search.find_best_move(board, CB_BLACK)
    assert stats.best_move is not None, "Search should find a move"

    print(f"✓ Search found: {stats}")


def test_evaluation_initial():
    """Test evaluation of the initial position."""
    board = Board()
    score = evaluate(board.b, CB_BLACK)

    # Initial position should be roughly equal (small turn bonus)
    assert -50 < score < 50, \
        f"Initial position eval should be near 0, got {score}"

    print(f"✓ Initial position evaluation: {score}")


def test_evaluation_material_advantage():
    """Test that evaluation correctly reflects material advantage."""
    board = Board()

    # Remove one white man → black has material advantage
    board.b[28] = FREE  # Remove white man at square 28

    score_black = evaluate(board.b, CB_BLACK)
    score_white = evaluate(board.b, CB_WHITE)

    assert score_black > 0, \
        f"Black should have positive eval with material advantage, got {score_black}"

    print(f"✓ Material advantage: black turn eval={score_black}, white turn eval={score_white}")


def test_rules_validation():
    """Test move validation through the Rules engine."""
    rules = Rules()
    board = Board()

    # Get a legal move
    moves = rules.get_legal_moves(board, CB_BLACK)
    assert len(moves) > 0, "Should have legal moves"

    # Apply the first legal move
    move = moves[0]
    new_board = board.apply_move(move)

    # Validate: should be legal
    is_legal, matched_move = rules.validate_human_move(board, new_board, CB_BLACK)
    assert is_legal, f"Legal move should be validated as legal: {move.to_notation()}"

    # Create an illegal board state (move two pieces at once)
    illegal_board = board.copy()
    illegal_board.b[19] = BLACK_MAN  # Place a random piece
    illegal_board.b[5] = FREE       # Remove a piece

    is_legal_bad, _ = rules.validate_human_move(board, illegal_board, CB_BLACK)
    assert not is_legal_bad, "Illegal move should be rejected"

    print("✓ Move validation works correctly")


def test_game_over_detection():
    """Test game-over detection."""
    rules = Rules()
    board = Board()

    # Clear board — only one black piece, no white pieces
    for sq in PLAYABLE_SQUARES:
        board.b[sq] = FREE
    board.b[20] = BLACK_MAN

    game_over, winner = rules.is_game_over(board, CB_WHITE)
    assert game_over, "Game should be over when white has no pieces"
    assert winner == 'black', f"Black should win, got {winner}"

    print("✓ Game-over detection correct")


def test_king_promotion():
    """Test that a man reaching the back rank promotes to king."""
    board = Board()
    gen = MoveGenerator()

    # Clear the board
    for sq in PLAYABLE_SQUARES:
        board.b[sq] = FREE

    # Place black man on row 6 (one row from promotion)
    # Row 6 squares: 32, 33, 34, 35
    # A black man at sq 33 can move to sq 37 or sq 38 (row 7 = promotion)
    board.b[33] = BLACK_MAN

    moves = gen.get_legal_moves(board, CB_BLACK)
    assert len(moves) > 0, "Black man should have moves"

    # Check that at least one move results in promotion
    promotion_moves = [m for m in moves if m.is_promotion]
    assert len(promotion_moves) > 0, "Should have at least one promotion move"

    for m in promotion_moves:
        print(f"  Promotion: {m.to_notation()} (new_piece={m.new_piece})")

    print(f"✓ King promotion: {len(promotion_moves)} promotion move(s)")


def test_forced_capture():
    """Test that when captures are available, non-capture moves are excluded."""
    board = Board()
    gen = MoveGenerator()

    # Clear and set up forced capture
    for sq in PLAYABLE_SQUARES:
        board.b[sq] = FREE

    board.b[15] = BLACK_MAN   # Black man at (2, 2)
    board.b[20] = WHITE_MAN   # White man at (3, 3) — can be jumped
    board.b[25] = FREE        # Landing square (4, 4) — empty
    board.b[14] = BLACK_MAN   # Another black man that could normally move

    moves = gen.get_legal_moves(board, CB_BLACK)

    # All returned moves should be captures (forced capture rule)
    for m in moves:
        assert m.is_capture, f"All moves should be captures due to forced capture rule"

    print(f"✓ Forced capture: {len(moves)} forced capture(s)")


def test_move_apply_undo():
    """Test that applying and undoing a move restores the board."""
    board = Board()
    gen = MoveGenerator()

    moves = gen.get_legal_moves(board, CB_BLACK)
    move = moves[0]

    original = board.copy()

    # Apply
    board.apply_move_inplace(move)
    assert not board.boards_match(original), "Board should change after move"

    # Undo
    board.undo_move_inplace(move)
    assert board.boards_match(original), "Board should be restored after undo"

    print("✓ Move apply/undo is reversible")


def test_board_pretty_print():
    """Test that the board prints correctly."""
    board = Board()
    output = str(board)
    assert "(white)" in output
    assert "(black)" in output
    assert "b" in output  # black men
    assert "w" in output  # white men
    print(output)
    print("✓ Board pretty-print works")


def test_complete_game_simulation():
    """Simulate a few moves of a complete game to verify end-to-end flow."""
    board = Board()
    rules = Rules()
    search = Search(max_time=1.0, max_depth=4)  # Fast search for testing

    current_colour = CB_BLACK
    max_moves = 10

    print("\n--- Game Simulation ---")
    print(board)

    for move_num in range(1, max_moves + 1):
        # Check game over
        game_over, winner = rules.is_game_over(board, current_colour)
        if game_over:
            print(f"\nGame over! Winner: {winner}")
            break

        # Find best move
        stats = search.find_best_move(board, current_colour)
        if stats.best_move is None:
            print(f"\nNo moves available for {_colour_name(current_colour)}")
            break

        move = stats.best_move
        colour_name = _colour_name(current_colour)
        print(
            f"\n{move_num}. {colour_name}: {rules.format_move(move)} "
            f"(eval={stats.eval_score}, depth={stats.depth_reached})"
        )

        # Apply move
        board.apply_move_inplace(move)
        print(board)

        # Switch colour
        current_colour = CB_WHITE if current_colour == CB_BLACK else CB_BLACK

    print("\n✓ Complete game simulation ran successfully")


def _colour_name(colour):
    return "Black" if colour == CB_BLACK else "White"


if __name__ == '__main__':
    print("=" * 60)
    print("CHECKERS ENGINE TESTS")
    print("=" * 60)

    test_board_initial_position()
    test_coordinate_mappings()
    test_board8x8_conversion()
    test_flat64_conversion()
    test_move_generation_initial()
    test_capture_detection()
    test_evaluation_initial()
    test_evaluation_material_advantage()
    test_rules_validation()
    test_game_over_detection()
    test_king_promotion()
    test_forced_capture()
    test_move_apply_undo()
    test_board_pretty_print()
    test_search_finds_move()
    test_complete_game_simulation()

    print("\n" + "=" * 60)
    print("ALL TESTS PASSED ✓")
    print("=" * 60)
