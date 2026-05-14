"""
evaluation.py — Board evaluation function for English checkers.

Ported from SimpleCh (Martin Fierz), simplech.c lines 756–1100+.

Features evaluated:
    - Material balance (men=100, kings=130 centipawns)
    - Exchange bonus (favour trades when ahead)
    - Tempo (piece advancement)
    - Back rank guard
    - Center control
    - Edge penalty
    - Cramp detection
    - Intact double corner

The evaluation is from BLACK's perspective: positive = good for black.
"""

from __future__ import annotations
from typing import List

from .board import (
    FREE, OCCUPIED,
    CB_BLACK, CB_WHITE, CB_MAN, CB_KING,
    BLACK_MAN, BLACK_KING, WHITE_MAN, WHITE_KING,
    PLAYABLE_SQUARES,
)

BLACK = CB_BLACK
WHITE = CB_WHITE
MAN = CB_MAN
KING = CB_KING

# ─── Internal square indices ────────────────────────────────────────────────
# Row mapping (which row each internal square belongs to)
ROW = [0] * 41
for i in range(5, 9):
    ROW[i] = 0
for i in range(10, 14):
    ROW[i] = 1
for i in range(14, 18):
    ROW[i] = 2
for i in range(19, 23):
    ROW[i] = 3
for i in range(23, 27):
    ROW[i] = 4
for i in range(28, 32):
    ROW[i] = 5
for i in range(32, 36):
    ROW[i] = 6
for i in range(37, 41):
    ROW[i] = 7

# Edge squares (less desirable — pieces can be trapped)
EDGE = [5, 6, 7, 8, 13, 14, 22, 23, 31, 32, 37, 38, 39, 40]

# Center squares (more desirable — better control)
CENTER = [15, 16, 20, 21, 24, 25, 29, 30]

# Safe edge squares (double corner protection)
SAFE_EDGE = [8, 13, 32, 37]

# ─── Evaluation Constants ───────────────────────────────────────────────────
TURN_BONUS = 2          # Side to move gets a small bonus
BRV = 3                 # Back rank value multiplier
KCV = 5                 # King center value multiplier
MCV = 1                 # Man center value multiplier
MEV = 1                 # Man edge value multiplier (penalty)
KEV = 5                 # King edge value multiplier (penalty)
CRAMP = 5               # Cramp penalty
OPENING_TEMPO = -2      # Tempo multiplier in opening
MIDGAME_TEMPO = -1      # Tempo multiplier in midgame
ENDGAME_TEMPO = 2       # Tempo multiplier in endgame
INTACT_DOUBLE_CORNER = 3  # Intact double corner bonus

# Value lookup for piece counting
# Indexed by piece code: value[piece] gives the contribution to a packed counter
VALUE_MAP = [0] * 17
VALUE_MAP[BLACK_MAN] = 256      # contributes to nbm count (bits 8-11)
VALUE_MAP[BLACK_KING] = 4096    # contributes to nbk count (bits 12-15)
VALUE_MAP[WHITE_MAN] = 1        # contributes to nwm count (bits 0-3)
VALUE_MAP[WHITE_KING] = 16      # contributes to nwk count (bits 4-7)


def evaluate(b: List[int], colour: int) -> int:
    """Evaluate the board position.

    Ported from simplech.c evaluation() (lines 756–1100+).

    Args:
        b: 46-element board array (internal representation).
        colour: Side to move (CB_BLACK or CB_WHITE).

    Returns:
        Evaluation score from BLACK's perspective.
        Positive = good for black, negative = good for white.
    """
    # ─── Count pieces using packed value trick ───────────────────────────
    code = 0
    for i in PLAYABLE_SQUARES:
        piece = b[i]
        if piece != FREE and piece < len(VALUE_MAP):
            code += VALUE_MAP[piece]

    nwm = code & 0xF               # number of white men
    nwk = (code >> 4) & 0xF        # number of white kings
    nbm = (code >> 8) & 0xF        # number of black men
    nbk = (code >> 12) & 0xF       # number of black kings

    # Material values
    v1 = 100 * nbm + 130 * nbk     # black material
    v2 = 100 * nwm + 130 * nwk     # white material

    if v1 + v2 == 0:
        return 0

    # Base evaluation: material + exchange bonus
    eval_score = v1 - v2
    eval_score += (250 * (v1 - v2)) // (v1 + v2)

    nm = nbm + nwm  # total men
    nk = nbk + nwk  # total kings

    # ─── Turn bonus ──────────────────────────────────────────────────────
    if colour == BLACK:
        eval_score += TURN_BONUS
    else:
        eval_score -= TURN_BONUS

    # ─── Cramp detection ─────────────────────────────────────────────────
    # Black cramp: black man on 23, white man on 28
    if b[23] == BLACK_MAN and b[28] == WHITE_MAN:
        eval_score += CRAMP
    # White cramp: white man on 22, black man on 17
    if b[22] == WHITE_MAN and b[17] == BLACK_MAN:
        eval_score -= CRAMP

    # ─── Back rank guard ─────────────────────────────────────────────────
    # Black back rank (row 0): squares 5, 6, 7, 8
    black_backrank = _back_rank_value(b, [5, 6, 7, 8])
    # White back rank (row 7): squares 37, 38, 39, 40 (reversed order)
    white_backrank = _back_rank_value(b, [40, 39, 38, 37])

    eval_score += BRV * (black_backrank - white_backrank)

    # ─── Intact double corner ────────────────────────────────────────────
    if b[8] == BLACK_MAN:
        if b[12] == BLACK_MAN or b[13] == BLACK_MAN:
            eval_score += INTACT_DOUBLE_CORNER

    if b[37] == WHITE_MAN:
        if b[32] == WHITE_MAN or b[33] == WHITE_MAN:
            eval_score -= INTACT_DOUBLE_CORNER

    # ─── Center control ──────────────────────────────────────────────────
    nbmc = nbkc = nwmc = nwkc = 0
    for sq in CENTER:
        piece = b[sq]
        if piece == BLACK_MAN:
            nbmc += 1
        elif piece == BLACK_KING:
            nbkc += 1
        elif piece == WHITE_MAN:
            nwmc += 1
        elif piece == WHITE_KING:
            nwkc += 1

    eval_score += (nbmc - nwmc) * MCV
    eval_score += (nbkc - nwkc) * KCV

    # ─── Edge penalty ────────────────────────────────────────────────────
    nbme = nbke = nwme = nwke = 0
    for sq in EDGE:
        piece = b[sq]
        if piece == BLACK_MAN:
            nbme += 1
        elif piece == BLACK_KING:
            nbke += 1
        elif piece == WHITE_MAN:
            nwme += 1
        elif piece == WHITE_KING:
            nwke += 1

    eval_score -= (nbme - nwme) * MEV
    eval_score -= (nbke - nwke) * KEV

    # ─── Tempo ───────────────────────────────────────────────────────────
    tempo = 0
    for i in PLAYABLE_SQUARES:
        if b[i] == BLACK_MAN:
            tempo += ROW[i] if i < len(ROW) else 0
        elif b[i] == WHITE_MAN:
            tempo -= (7 - ROW[i]) if i < len(ROW) else 0

    if nm >= 16:
        eval_score += OPENING_TEMPO * tempo
    elif nm <= 12:
        eval_score += ENDGAME_TEMPO * tempo
    else:
        eval_score += MIDGAME_TEMPO * tempo

    return eval_score


def _back_rank_value(b: List[int], squares: List[int]) -> int:
    """Compute the back rank guard value for a set of 4 back-rank squares.

    Ported from simplech.c (lines 880–953).
    The encoding checks which of the 4 back-rank squares have men on them
    and assigns a value based on the pattern.
    """
    code = 0
    if b[squares[0]] & MAN:
        code += 1
    if b[squares[1]] & MAN:
        code += 2
    if b[squares[2]] & MAN:
        code += 4
    if b[squares[3]] & MAN:
        code += 8

    BACKRANK_TABLE = {
        0: 0, 1: -1, 2: 1, 3: 0, 4: 1, 5: 1, 6: 2, 7: 1,
        8: 1, 9: 0, 10: 7, 11: 4, 12: 2, 13: 2, 14: 9, 15: 8,
    }
    return BACKRANK_TABLE.get(code, 0)
