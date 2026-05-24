# Game engine — ported from KingsRow SimpleCh (Martin Fierz)
# Core algorithms: board representation, move generation, alpha-beta search, evaluation

from .board import Board, EMPTY, BLACK_MAN, BLACK_KING, WHITE_MAN, WHITE_KING
from .board import CB_BLACK, CB_WHITE, CB_MAN, CB_KING, FREE, OCCUPIED
from .move_generator import MoveGenerator
from .search import Search
from .evaluation import evaluate
from .rules import Rules

__all__ = [
    'Board', 'MoveGenerator', 'Search', 'evaluate', 'Rules',
    'EMPTY', 'BLACK_MAN', 'BLACK_KING', 'WHITE_MAN', 'WHITE_KING',
    'CB_BLACK', 'CB_WHITE', 'CB_MAN', 'CB_KING', 'FREE', 'OCCUPIED',
]
