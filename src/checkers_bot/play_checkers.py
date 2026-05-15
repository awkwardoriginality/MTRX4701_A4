#!/usr/bin/env python3
"""
play_checkers.py — Standalone interactive English checkers demo.

Supports two premium play modes:
    1. Point-and-Click Graphical User Interface (GUI) via Tkinter (Default)
    2. Coloured terminal interactive console (--terminal)

Features:
    - Grounded in the fully ported pure-Python KingsRow checkers engine
    - Validates all English checkers rules (mandatory captures, king promotion)
    - Full point-and-click move preview, illegal move rollback, and hop animation
    - Real-time evaluation feedback, game history, and play-state decision logs
    - Standalone gameplay with no dependency on the arm visualisation

Usage:
    python3 play_checkers.py                  # launches point-and-click GUI
    python3 play_checkers.py --terminal       # launches terminal CLI mode
    python3 play_checkers.py --colour white   # play as white
    python3 play_checkers.py --time 2.0       # AI search time budget
    python3 play_checkers.py --ros-bridge     # GUI publishes board updates to ROS
"""

from __future__ import annotations
import sys
import os
import re
import time
import argparse
import threading
from typing import Callable, List, Optional, Tuple

# Add parent directory for imports
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from checkers_bot.game_engine.board import (
    Board, Move,
    CB_BLACK, CB_WHITE,
    BLACK_MAN, BLACK_KING, WHITE_MAN, WHITE_KING,
    FREE,
    INTERNAL_TO_STANDARD, STANDARD_TO_INTERNAL,
    INTERNAL_TO_XY, ROWCOL_TO_INTERNAL, XY_TO_INTERNAL,
)
from checkers_bot.game_engine.move_generator import MoveGenerator
from checkers_bot.game_engine.search import Search
from checkers_bot.game_engine.rules import Rules
from checkers_bot.protocol import BoardStateReport, GameStatus, RobotInstruction

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import Bool, String, UInt8MultiArray
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, font
    HAS_TKINTER = True
except ImportError:
    HAS_TKINTER = False


# ─── ANSI Colour Codes (Terminal Mode) ──────────────────────────────────────

class C:
    """ANSI colour codes for terminal output."""
    RESET    = '\033[0m'
    BOLD     = '\033[1m'
    DIM      = '\033[2m'
    RED      = '\033[91m'
    GREEN    = '\033[92m'
    YELLOW   = '\033[93m'
    BLUE     = '\033[94m'
    MAGENTA  = '\033[95m'
    CYAN     = '\033[96m'
    WHITE    = '\033[97m'
    BG_BLACK = '\033[40m'
    BG_RED   = '\033[41m'
    BG_GREEN = '\033[42m'
    BG_BROWN = '\033[43m'


# ─── Terminal Board Rendering ───────────────────────────────────────────────

def render_board(board: Board, last_move: Optional[Move] = None,
                 perspective: int = CB_BLACK) -> str:
    """Render the board as a colourful terminal string."""
    highlights = set()
    if last_move:
        highlights.add(last_move.from_sq)
        highlights.add(last_move.to_sq)
        for sq in getattr(last_move, 'captured_squares', []):
            highlights.add(sq)

    lines = ["", f"  {C.DIM}┌────────────────────────────────────┐{C.RESET}"]

    row_range = range(7, -1, -1) if perspective == CB_BLACK else range(0, 8)

    for row in row_range:
        line = f"  {C.DIM}│{C.RESET} "
        for col in range(8):
            is_dark = (row + col) % 2 == 0
            rc = (row, col)
            sq = ROWCOL_TO_INTERNAL.get(rc)

            if is_dark and sq is not None:
                piece = board.b[sq]
                is_highlighted = sq in highlights

                bg = '\033[48;5;28m' if is_highlighted else '\033[48;5;130m'

                if piece == BLACK_MAN:
                    char = f'{C.RED}{C.BOLD} ● {C.RESET}'
                elif piece == BLACK_KING:
                    char = f'{C.RED}{C.BOLD} ♛ {C.RESET}'
                elif piece == WHITE_MAN:
                    char = f'{C.WHITE}{C.BOLD} ● {C.RESET}'
                elif piece == WHITE_KING:
                    char = f'{C.WHITE}{C.BOLD} ♛ {C.RESET}'
                else:
                    std_num = INTERNAL_TO_STANDARD.get(sq, 0)
                    char = f'{C.DIM}{std_num:3d}{C.RESET}'

                line += f'{bg}{char}{bg} {C.RESET}'
            else:
                line += f'\033[48;5;222m    {C.RESET}'

        line += f" {C.DIM}│{C.RESET}"
        lines.append(line)

    lines.append(f"  {C.DIM}└────────────────────────────────────┘{C.RESET}")
    lines.append(
        f"    {C.RED}{C.BOLD}●{C.RESET} = Red (Black)   "
        f"{C.WHITE}{C.BOLD}●{C.RESET} = White   "
        f"{C.RED}{C.BOLD}♛{C.RESET}/{C.WHITE}{C.BOLD}♛{C.RESET} = King"
    )
    return '\n'.join(lines)


# ─── Move Parsing & Search Helpers ──────────────────────────────────────────

def parse_move_input(text: str) -> Optional[Tuple[int, int]]:
    text = text.strip()
    m = re.match(r'(\d+)\s*[-xX,\s]\s*(\d+)', text)
    if m:
        return int(m.group(1)), int(m.group(2))
    return None


def find_matching_move(from_std: int, to_std: int, legal_moves: List[Move]) -> Optional[Move]:
    from_internal = STANDARD_TO_INTERNAL.get(from_std)
    to_internal = STANDARD_TO_INTERNAL.get(to_std)
    if from_internal is None or to_internal is None:
        return None
    for move in legal_moves:
        if move.from_sq == from_internal and move.to_sq == to_internal:
            return move
    return None


def format_square_move(from_sq: int, to_sq: int) -> str:
    """Format an observed move attempt between two internal squares."""
    from_std = INTERNAL_TO_STANDARD.get(from_sq, from_sq)
    to_std = INTERNAL_TO_STANDARD.get(to_sq, to_sq)
    return f"{from_std}-{to_std}"


def is_multi_hop_capture(move: Move) -> bool:
    """Return True when a capture traverses multiple landing squares."""
    return move.is_capture and len(get_move_path_squares(move)) > 2


def get_move_path_squares(move: Move) -> List[int]:
    """Return the best available square-by-square trajectory for a move."""
    if not move.is_capture:
        return move.path_squares if move.path_squares else [move.from_sq, move.to_sq]

    if move.path_squares and len(move.path_squares) > 2:
        return move.path_squares

    if not move.captured_squares:
        return move.path_squares if move.path_squares else [move.from_sq, move.to_sq]

    path = [move.from_sq]
    current_sq = move.from_sq

    for cap_sq in move.captured_squares:
        current_xy = INTERNAL_TO_XY.get(current_sq)
        captured_xy = INTERNAL_TO_XY.get(cap_sq)
        if current_xy is None or captured_xy is None:
            return move.path_squares if move.path_squares else [move.from_sq, move.to_sq]

        dx = captured_xy[0] - current_xy[0]
        dy = captured_xy[1] - current_xy[1]
        if abs(dx) != 1 or abs(dy) != 1:
            return move.path_squares if move.path_squares else [move.from_sq, move.to_sq]

        landing_xy = (captured_xy[0] + dx, captured_xy[1] + dy)
        landing_sq = XY_TO_INTERNAL.get(landing_xy)
        if landing_sq is None:
            return move.path_squares if move.path_squares else [move.from_sq, move.to_sq]

        path.append(landing_sq)
        current_sq = landing_sq

    if path[-1] != move.to_sq:
        path.append(move.to_sq)

    return path


class GuiRosBridge:
    """Bridge the standalone GUI board state into the ROS checkers topics."""

    def __init__(
        self,
        on_internal_board: Optional[Callable[[Board], None]] = None,
        on_game_status: Optional[Callable[[GameStatus], None]] = None,
        on_robot_instruction: Optional[Callable[[RobotInstruction], None]] = None,
    ):
        if not HAS_ROS2:
            raise RuntimeError("ROS 2 Python bindings are not available for GUI bridge mode.")

        rclpy.init(args=None)
        self.node = Node('checkers_gui_bridge')
        self._on_internal_board = on_internal_board
        self._on_game_status = on_game_status
        self._on_robot_instruction = on_robot_instruction
        self._stable_count = 0
        self._last_flat64: Optional[List[int]] = None

        self.board_pub = self.node.create_publisher(
            UInt8MultiArray, '/checkers/board_state', 10
        )
        self.report_pub = self.node.create_publisher(
            String, '/checkers/board_state_report', 10
        )
        self.blocked_pub = self.node.create_publisher(
            Bool, '/checkers/board_blocked', 10
        )

        self.node.create_subscription(
            UInt8MultiArray, '/checkers/internal_board', self._internal_board_cb, 10
        )
        self.node.create_subscription(
            String, '/checkers/game_status', self._game_status_cb, 10
        )
        self.node.create_subscription(
            String, '/checkers/robot_instruction', self._robot_instruction_cb, 10
        )

        self._spin_thread = threading.Thread(target=self._spin, daemon=True)
        self._spin_thread.start()

    def _spin(self):
        rclpy.spin(self.node)

    def publish_board(self, board: Board, *, source: str):
        """Publish the canonical board and a stable board-state report."""
        flat64 = board.to_flat64()
        if self._last_flat64 == flat64:
            self._stable_count += 1
        else:
            self._last_flat64 = flat64[:]
            self._stable_count = 1

        board_msg = UInt8MultiArray()
        board_msg.data = flat64
        self.board_pub.publish(board_msg)

        blocked_msg = Bool()
        blocked_msg.data = False
        self.blocked_pub.publish(blocked_msg)

        report_msg = String()
        report_msg.data = BoardStateReport(
            flat64=flat64,
            stable_count=max(self._stable_count, 2),
            hand_present=False,
            board_blocked=False,
            confidence=1.0,
            source=source,
        ).to_json()
        self.report_pub.publish(report_msg)

    def _internal_board_cb(self, msg: UInt8MultiArray):
        if self._on_internal_board is None:
            return
        self._on_internal_board(Board.from_flat64(list(msg.data)))

    def _game_status_cb(self, msg: String):
        if self._on_game_status is None:
            return
        try:
            self._on_game_status(GameStatus.from_json(msg.data))
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

    def _robot_instruction_cb(self, msg: String):
        if self._on_robot_instruction is None:
            return
        try:
            self._on_robot_instruction(RobotInstruction.from_json(msg.data))
        except (TypeError, ValueError, json.JSONDecodeError):
            pass

    def shutdown(self):
        """Release ROS resources on GUI exit."""
        if not HAS_ROS2:
            return
        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


# ─── Core Game Logic Controller ─────────────────────────────────────────────

class CheckersGame:
    """Core game state container and orchestration logic."""

    def __init__(self, human_colour: int = CB_BLACK, ai_time: float = 5.0, ai_depth: int = 99):
        self.board = Board()
        self.rules = Rules()
        self.search = Search(max_time=ai_time, max_depth=ai_depth)
        self.move_gen = MoveGenerator()

        self.human_colour = human_colour
        self.robot_colour = CB_WHITE if human_colour == CB_BLACK else CB_BLACK
        self.current_turn = CB_BLACK  # Black always moves first in standard checkers

        self.move_number = 0
        self.move_history: List[str] = []
        self.decision_history: List[str] = []
        self.board_history: List[Board] = [self.board.copy()]
        self.last_move: Optional[Move] = None

        self.human_name = "Red" if human_colour == CB_BLACK else "White"
        self.robot_name = "White" if human_colour == CB_BLACK else "Red"
        self.log_decision(
            "INIT",
            f"Game initialised. Human={self.human_name}, robot={self.robot_name}.",
        )

    def log_decision(self, state: str, detail: str):
        """Record a play-state decision for UI display."""
        entry_no = len(self.decision_history) + 1
        self.decision_history.append(f"{entry_no}. {state}: {detail}")

    def apply_move(self, move: Move):
        """Apply a move to the authoritative board state."""
        self.board.apply_move_inplace(move)
        self.board_history.append(self.board.copy())
        self.last_move = move
        self.move_number += 1

        notation = move.to_notation()
        extras = []
        if move.is_capture:
            extras.append(f"captures {len(move.captured_squares)}")
        if move.is_promotion:
            extras.append("→ KING")
        extra_str = f" ({', '.join(extras)})" if extras else ""

        player = self.human_name if self.current_turn == self.human_colour else self.robot_name
        self.move_history.append(f"{self.move_number}. {player}: {notation}{extra_str}")

        # Switch turn
        self.current_turn = CB_WHITE if self.current_turn == CB_BLACK else CB_BLACK

    def undo_last_round(self) -> bool:
        """Undo both AI and Human half-moves to revert to human's choice."""
        if len(self.board_history) < 3:
            return False

        self.board_history.pop()  # AI move
        self.board_history.pop()  # Human move
        self.board = self.board_history[-1].copy()

        self.move_number = max(0, self.move_number - 2)
        self.current_turn = self.human_colour
        self.last_move = None

        if len(self.move_history) >= 2:
            self.move_history.pop()
            self.move_history.pop()

        return True

    def sync_from_observed_board(self, observed_board: Board, mover_colour: int) -> Optional[Move]:
        """Apply a legal observed board transition from an external authority."""
        if self.board.boards_match(observed_board):
            return None

        is_legal, move = self.rules.validate_human_move(
            self.board, observed_board, mover_colour
        )
        if not is_legal or move is None:
            return None

        self.apply_move(move)
        return move


# ─── Premium Graphical User Interface (GUI Mode) ────────────────────────────

class CheckersGUI:
    """Point-and-click Tkinter interface for the standalone checkers demo."""

    def __init__(self, root: tk.Tk, game: CheckersGame, ros_bridge_mode: bool = False):
        self.root = root
        self.game = game
        self.ros_bridge_mode = ros_bridge_mode
        self.ros_bridge: Optional[GuiRosBridge] = None

        self.root.title("English Checkers")
        self.root.geometry("1360x760")
        self.root.minsize(1220, 650)

        # Configure dark-mode inspired premium styling
        self.bg_base = "#1E1E1E"
        self.bg_panel = "#252526"
        self.fg_text = "#D4D4D4"
        self.color_dark_sq = "#5C3A21"      # Premium rich walnut
        self.color_light_sq = "#DEB887"     # Smooth warm maple
        self.color_selected = "#00FFCC"     # Vibrant cyan highlight
        self.color_target = "#7FFF00"       # Chartreuse valid target dot
        self.color_status_normal = "#00FFCC"
        self.color_status_error = "#FF4D4D"

        self.root.configure(bg=self.bg_base)

        # Interaction state
        self.selected_sq: Optional[int] = None
        self.legal_moves_cache: List[Move] = []
        self.valid_targets: dict[int, Move] = {}  # destination_square -> Move object
        self.ai_thinking = False
        self.animating_move = False
        self.display_board: Optional[Board] = None
        self.preview_piece: Optional[int] = None
        self.preview_target_rc: Optional[Tuple[int, int]] = None
        self._last_turn_signature: Optional[Tuple[int, int]] = None
        self._game_over_logged = False
        self._last_ros_status_signature: Optional[Tuple[str, int, str]] = None
        self._last_ros_instruction_signature: Optional[Tuple[str, str, Optional[str], int]] = None

        if self.ros_bridge_mode:
            self.ros_bridge = GuiRosBridge(
                on_internal_board=lambda board: self.root.after(0, lambda: self._on_ros_internal_board(board)),
                on_game_status=lambda status: self.root.after(0, lambda: self._on_ros_game_status(status)),
                on_robot_instruction=lambda instr: self.root.after(0, lambda: self._on_ros_robot_instruction(instr)),
            )

        self.root.protocol("WM_DELETE_WINDOW", self._on_close)
        self._setup_ui()
        self._update_state()
        if self.ros_bridge is not None:
            self.root.after(200, lambda: self.ros_bridge.publish_board(self.game.board, source='gui_bridge_initial'))

    def _setup_ui(self):
        """Construct the board, controls, move history, and decision log panes."""
        # Main layout frames
        self.main_panes = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, bg=self.bg_base, bd=0)
        self.main_panes.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left Frame: Board Canvas
        self.board_frame = tk.Frame(self.main_panes, bg=self.bg_base)
        self.main_panes.add(self.board_frame, width=650)

        self.canvas = tk.Canvas(self.board_frame, bg=self.bg_base, bd=0, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", self._on_resize)
        self.canvas.bind("<Button-1>", self._on_click)

        # Middle Frame: Statistics, controls, and move history
        self.sidebar = tk.Frame(self.main_panes, bg=self.bg_panel, bd=1, relief=tk.SUNKEN)
        self.main_panes.add(self.sidebar, width=320)

        # Custom heading styling
        title_font = font.Font(family="Helvetica", size=16, weight="bold")
        lbl_title = tk.Label(self.sidebar, text="♔ English Checkers ♔", font=title_font,
                             bg=self.bg_panel, fg="#E5E5E5", pady=10)
        lbl_title.pack(fill=tk.X)

        # Status Bar
        self.status_var = tk.StringVar(value="Game initialized.")
        self.lbl_status = tk.Label(self.sidebar, textvariable=self.status_var,
                                   font=("Helvetica", 12, "bold"), bg=self.bg_panel, fg=self.color_status_normal, pady=5)
        self.lbl_status.pack(fill=tk.X)

        # Score Counters
        self.score_var = tk.StringVar()
        self.lbl_score = tk.Label(self.sidebar, textvariable=self.score_var,
                                  font=("Helvetica", 11), bg=self.bg_panel, fg=self.fg_text, justify=tk.LEFT)
        self.lbl_score.pack(fill=tk.X, padx=10, pady=5)

        # Control Buttons
        btn_frame = tk.Frame(self.sidebar, bg=self.bg_panel)
        btn_frame.pack(fill=tk.X, padx=15, pady=10)

        style = ttk.Style()
        style.theme_use('clam')
        style.configure('TButton', font=('Helvetica', 10, 'bold'), padding=5)

        self.btn_undo = ttk.Button(btn_frame, text="↩ Undo Round", command=self._undo_round)
        self.btn_undo.pack(fill=tk.X, pady=3)

        self.btn_reset = ttk.Button(btn_frame, text="🔄 Reset Game", command=self._reset_game)
        self.btn_reset.pack(fill=tk.X, pady=3)

        # Move Log Listbox
        tk.Label(self.sidebar, text="Move History:", font=("Helvetica", 10, "bold"),
                 bg=self.bg_panel, fg=self.fg_text, anchor=tk.W).pack(fill=tk.X, padx=10, pady=(10, 0))

        log_frame = tk.Frame(self.sidebar, bg=self.bg_panel)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(2, 10))

        self.log_box = tk.Listbox(log_frame, bg="#1E1E1E", fg=self.fg_text, bd=0,
                                  highlightthickness=1, highlightcolor="#333333", font=("Consolas", 10))
        scrollbar = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_box.yview)
        self.log_box.configure(yscrollcommand=scrollbar.set)

        self.log_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Right Frame: play-state decision trace
        self.decision_sidebar = tk.Frame(self.main_panes, bg=self.bg_panel, bd=1, relief=tk.SUNKEN)
        self.main_panes.add(self.decision_sidebar, width=380)

        tk.Label(self.decision_sidebar, text="Play State Decisions:", font=("Helvetica", 10, "bold"),
                 bg=self.bg_panel, fg=self.fg_text, anchor=tk.W, pady=10).pack(fill=tk.X, padx=10)

        decision_frame = tk.Frame(self.decision_sidebar, bg=self.bg_panel)
        decision_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=(2, 10))

        self.decision_box = tk.Listbox(decision_frame, bg="#1E1E1E", fg=self.fg_text, bd=0,
                                       highlightthickness=1, highlightcolor="#333333", font=("Consolas", 10))
        decision_scrollbar = ttk.Scrollbar(decision_frame, orient=tk.VERTICAL, command=self.decision_box.yview)
        self.decision_box.configure(yscrollcommand=decision_scrollbar.set)

        self.decision_box.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        decision_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        self.sq_size = 75
        self.board_offset_x = 25
        self.board_offset_y = 25

    def _on_resize(self, event):
        """Dynamically redraw grid proportionally to window geometry."""
        size = min(event.width - 50, event.height - 50)
        self.sq_size = max(size // 8, 40)
        self.board_offset_x = (event.width - (self.sq_size * 8)) // 2
        self.board_offset_y = (event.height - (self.sq_size * 8)) // 2
        self._draw_board()

    def _draw_board(self):
        """Draw interactive grid squares, piece circles, and king crowns."""
        self.canvas.delete("all")
        board_view = self.display_board if self.display_board is not None else self.game.board

        # Determine rendering orientation
        perspective = self.game.human_colour
        row_order = range(7, -1, -1) if perspective == CB_BLACK else range(0, 8)

        # Draw border frame
        x_start = self.board_offset_x
        y_start = self.board_offset_y
        grid_w = self.sq_size * 8
        self.canvas.create_rectangle(x_start-4, y_start-4, x_start+grid_w+4, y_start+grid_w+4,
                                     fill="", outline="#333333", width=4)

        last_move_sqs = set()
        if self.game.last_move:
            last_move_sqs.add(self.game.last_move.from_sq)
            last_move_sqs.add(self.game.last_move.to_sq)

        for r_idx, row in enumerate(row_order):
            for col in range(8):
                x1 = x_start + col * self.sq_size
                y1 = y_start + r_idx * self.sq_size
                x2 = x1 + self.sq_size
                y2 = y1 + self.sq_size

                is_dark = (row + col) % 2 == 0
                rc = (row, col)
                sq = ROWCOL_TO_INTERNAL.get(rc)

                # Base tile shading
                fill_color = self.color_dark_sq if is_dark else self.color_light_sq
                self.canvas.create_rectangle(x1, y1, x2, y2, fill=fill_color, outline="#3E3E3E")

                if not is_dark or sq is None:
                    continue

                # Highlight last moves
                if sq in last_move_sqs:
                    self.canvas.create_rectangle(x1+2, y1+2, x2-2, y2-2,
                                                 fill="", outline="#FFA500", width=2)

                # Highlight selected source square
                if sq == self.selected_sq:
                    self.canvas.create_rectangle(x1+2, y1+2, x2-2, y2-2,
                                                 fill="", outline=self.color_selected, width=3)

                # Draw target dot overlays for valid move destinations
                if sq in self.valid_targets:
                    cx = (x1 + x2) // 2
                    cy = (y1 + y2) // 2
                    rad = self.sq_size // 6
                    self.canvas.create_oval(cx-rad, cy-rad, cx+rad, cy+rad,
                                            fill=self.color_target, outline="#000000")

                # Render pieces
                piece = board_view.b[sq]
                if piece != FREE:
                    self._draw_piece(piece, x1, y1, x2, y2)
                else:
                    # Subtle square numbering for reference
                    std_num = INTERNAL_TO_STANDARD.get(sq, "")
                    self.canvas.create_text(x1+8, y2-8, text=str(std_num), font=("Helvetica", 8),
                                            fill="#A0A0A0" if is_dark else "")

        if self.preview_piece is not None and self.preview_target_rc is not None:
            preview_row, preview_col = self.preview_target_rc
            preview_r_idx = (7 - preview_row) if perspective == CB_BLACK else preview_row
            x1 = x_start + preview_col * self.sq_size
            y1 = y_start + preview_r_idx * self.sq_size
            x2 = x1 + self.sq_size
            y2 = y1 + self.sq_size
            self._draw_piece(self.preview_piece, x1, y1, x2, y2)

    def _draw_piece(self, piece: int, x1: int, y1: int, x2: int, y2: int):
        """Draw a checker piece inside the given square bounds."""
        pad = self.sq_size // 6
        px1 = x1 + pad
        py1 = y1 + pad
        px2 = x2 - pad
        py2 = y2 - pad

        if piece in (BLACK_MAN, BLACK_KING):
            p_fill = "#DC143C"
            p_out = "#8B0000"
        else:
            p_fill = "#F8F8FF"
            p_out = "#A9A9A9"

        self.canvas.create_oval(px1, py1 + 2, px2, py2 + 2, fill="#0F0F0F", outline="")
        self.canvas.create_oval(px1, py1, px2, py2, fill=p_fill, outline=p_out, width=2)

        ipad = pad + (self.sq_size // 12)
        self.canvas.create_oval(x1 + ipad, y1 + ipad, x2 - ipad, y2 - ipad, fill="", outline=p_out, width=1)

        if piece in (BLACK_KING, WHITE_KING):
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            crown_color = "#FFD700" if piece == BLACK_KING else "#DAA520"
            self.canvas.create_text(cx, cy, text="♛", font=("Helvetica", max(self.sq_size // 3, 14)),
                                    fill=crown_color)

    def _set_status(self, text: str, *, error: bool = False):
        """Update status text and colour."""
        self.status_var.set(text)
        self.lbl_status.configure(fg=self.color_status_error if error else self.color_status_normal)

    def _refresh_logs(self):
        """Synchronise the move and decision listboxes with stored history."""
        self.log_box.delete(0, tk.END)
        for item in self.game.move_history:
            self.log_box.insert(tk.END, item)
        self.log_box.yview(tk.END)

        self.decision_box.delete(0, tk.END)
        for item in self.game.decision_history:
            self.decision_box.insert(tk.END, item)
        self.decision_box.yview(tk.END)

    def _log_decision(self, state: str, detail: str):
        """Record a decision entry and refresh the visible log."""
        self.game.log_decision(state, detail)
        self._refresh_logs()

    def _publish_board_to_ros(self, source: str):
        """Publish the current board through the GUI bridge when enabled."""
        if self.ros_bridge is not None:
            self.ros_bridge.publish_board(self.game.board, source=source)

    def _on_ros_internal_board(self, board: Board):
        """Mirror the game manager's authoritative board back into the GUI."""
        if self.game.board.boards_match(board):
            return

        move = self.game.sync_from_observed_board(board, self.game.robot_colour)
        if move is None:
            self._log_decision("ROS_SYNC", "Received a ROS board update that could not be reconciled locally.")
            return

        self._log_decision("ROS_SYNC", f"Applied ROS robot move {move.to_notation()} to the GUI board.")
        if move.is_promotion:
            self._log_decision("ROS_SYNC", f"{move.to_notation()} promoted to a king via ROS state sync.")
        self._update_state()

    def _on_ros_game_status(self, status: GameStatus):
        """Surface the state-machine snapshot in the decision log."""
        signature = (status.state, status.move_number, status.current_turn)
        if signature == self._last_ros_status_signature:
            return
        self._last_ros_status_signature = signature
        self._log_decision(
            "ROS_STATUS",
            f"{status.state} | turn={status.current_turn} | move={status.move_number}",
        )

    def _on_ros_robot_instruction(self, instruction: RobotInstruction):
        """Surface robot-instruction updates while the ROS game manager is active."""
        signature = (
            instruction.state,
            instruction.summary,
            instruction.active_command,
            instruction.move_number,
        )
        if signature == self._last_ros_instruction_signature:
            return
        self._last_ros_instruction_signature = signature
        summary = instruction.summary
        if instruction.active_command:
            summary += f" [{instruction.active_command}]"
        self._log_decision("ROS_INSTRUCTION", summary)

    def _on_close(self):
        """Shutdown ROS bridge resources before closing the GUI."""
        if self.ros_bridge is not None:
            self.ros_bridge.shutdown()
        self.root.destroy()

    def _select_square(self, sq: int):
        """Select a human piece and show all currently legal destinations."""
        self.selected_sq = sq
        self.valid_targets.clear()
        for move in self.legal_moves_cache:
            if move.from_sq == sq:
                self.valid_targets[move.to_sq] = move
        self._draw_board()

    def _build_attempt_board(self, from_sq: int, to_sq: int) -> Board:
        """Create a temporary board showing a manually attempted move."""
        preview = self.game.board.copy()
        piece = preview.b[from_sq]
        preview.b[from_sq] = FREE
        if to_sq in ROWCOL_TO_INTERNAL.values():
            preview.b[to_sq] = piece
        return preview

    def _show_attempt_preview(self, from_sq: int, target_row: int, target_col: int):
        """Visually transport a piece to an attempted destination square."""
        piece = self.game.board.b[from_sq]
        self.display_board = self.game.board.copy()
        self.display_board.b[from_sq] = FREE
        self.preview_piece = piece
        self.preview_target_rc = (target_row, target_col)

    def _build_hop_frames(self, move: Move) -> List[Tuple[Board, int]]:
        """Build intermediate landing positions for a multi-hop capture."""
        frames: List[Tuple[Board, int]] = []
        working = self.game.board.copy()
        path = get_move_path_squares(move)
        piece = working.b[path[0]]
        current_sq = path[0]

        for idx, next_sq in enumerate(path[1:]):
            working.b[current_sq] = FREE
            working.b[next_sq] = piece
            if idx < len(move.captured_squares):
                working.b[move.captured_squares[idx]] = FREE
            if idx == len(path) - 2 and move.is_promotion:
                working.b[next_sq] = move.new_piece
                piece = move.new_piece
            frames.append((working.copy(), next_sq))
            current_sq = next_sq

        return frames

    def _finish_move(self, move: Move):
        """Commit a validated move and continue the game flow."""
        self.display_board = None
        self.preview_piece = None
        self.preview_target_rc = None
        self.animating_move = False
        mover_colour = self.game.current_turn
        self.game.apply_move(move)
        if move.is_promotion:
            self._log_decision("CHECK_KING_PROMOTION", f"{move.to_notation()} reaches the back rank.")
            self._log_decision("PROMOTE_TO_KING", f"{move.to_notation()} promotes to a king.")
        else:
            self._log_decision("CHECK_KING_PROMOTION", f"No promotion follows {move.to_notation()}.")
        if self.ros_bridge_mode and mover_colour == self.game.human_colour:
            self._publish_board_to_ros('gui_bridge_human')
        self._update_state()

    def _animate_hop_move(self, move: Move, actor: str):
        """Animate each landing square in a multi-hop capture."""
        frames = self._build_hop_frames(move)
        if not frames:
            self._finish_move(move)
            return

        self.animating_move = True
        self.selected_sq = None
        self.valid_targets.clear()

        def _step(idx: int):
            if idx >= len(frames):
                self._finish_move(move)
                return

            frame_board, landing_sq = frames[idx]
            landing_std = INTERNAL_TO_STANDARD.get(landing_sq, landing_sq)
            self.display_board = frame_board
            self.preview_piece = None
            self.preview_target_rc = None
            self._set_status(f"{actor} hopping to square {landing_std}...")
            self._log_decision(
                "BOB_OVER_SQUARE",
                f"{actor} lands hop {idx + 1}/{len(frames)} on square {landing_std}.",
            )
            self._draw_board()
            self.root.after(220, lambda: _step(idx + 1))

        _step(0)

    def _execute_move(self, move: Move, actor: str):
        """Execute a validated move, animating multi-hop captures when needed."""
        if move.is_capture:
            capture_text = f"{actor} executes capture {move.to_notation()}."
            if is_multi_hop_capture(move):
                capture_text = f"{actor} executes multi-hop capture {move.to_notation()}."
            self._log_decision("EXECUTE_CAPTURE", capture_text)
        else:
            self._log_decision("EXECUTE_SIMPLE", f"{actor} executes {move.to_notation()}.")

        if is_multi_hop_capture(move):
            self._animate_hop_move(move, actor)
        else:
            self._finish_move(move)

    def _attempt_illegal_move(self, from_sq: int, to_sq: int):
        """Show an illegal human move briefly, then restore the board."""
        attempted = format_square_move(from_sq, to_sq)
        target_row, target_col = next((rc for rc, internal_sq in ROWCOL_TO_INTERNAL.items() if internal_sq == to_sq), (0, 0))
        self._log_decision("VALIDATE_HUMAN_MOVE", f"Observed attempted move {attempted}.")
        self._log_decision("ILLEGAL_MOVE_RESPONSE", f"Flagged {attempted} as illegal.")
        self._set_status(f"Illegal move attempted: {attempted}. Reverting...", error=True)
        self._show_attempt_preview(from_sq, target_row, target_col)
        self.animating_move = True
        self.selected_sq = None
        self.valid_targets.clear()
        self._draw_board()
        self.root.after(450, lambda: self._finish_illegal_attempt(from_sq))

    def _attempt_illegal_dark_square_move(self, from_sq: int, row: int, col: int):
        """Treat a wrong-coloured square click as an explicit illegal move attempt."""
        attempted_std = ((7 - row) * 4 + (col // 2) + 1) if self.game.human_colour == CB_BLACK else (row * 4 + (col // 2) + 1)
        from_std = INTERNAL_TO_STANDARD.get(from_sq, from_sq)
        attempted = f"{from_std}-{attempted_std}"
        self._log_decision("VALIDATE_HUMAN_MOVE", f"Observed attempted move {attempted}.")
        self._log_decision("ILLEGAL_MOVE_RESPONSE", f"Flagged {attempted} as illegal (wrong-coloured square).")
        self._set_status(f"Illegal move attempted: {attempted}. Reverting...", error=True)
        self._show_attempt_preview(from_sq, row, col)
        self.animating_move = True
        self.selected_sq = None
        self.valid_targets.clear()
        self._draw_board()
        self.root.after(450, lambda: self._finish_illegal_attempt(from_sq))

    def _finish_illegal_attempt(self, from_sq: int):
        """Restore the board after an illegal move attempt."""
        self.display_board = None
        self.preview_piece = None
        self.preview_target_rc = None
        self.animating_move = False
        self._set_status("Illegal move undone. Choose a legal move.", error=True)
        self._log_decision("UNDO_ILLEGAL", "Restored the board and returned control to the player.")
        if self.game.board.b[from_sq] != FREE:
            self._select_square(from_sq)
        else:
            self._draw_board()

    def _update_state(self):
        """Update side panel counters, evaluate turn legality, and drive AI states."""
        self.selected_sq = None
        self.valid_targets.clear()
        self.display_board = None
        self.preview_piece = None
        self.preview_target_rc = None
        self.animating_move = False

        # Update lists
        self._refresh_logs()

        # Check Win/Loss status
        game_over, winner = self.game.rules.is_game_over(self.game.board, self.game.current_turn)
        if game_over:
            win_str = "DRAW" if winner == "draw" else f"{winner.upper()} WINS!"
            self._set_status(f"Game Over: {win_str}")
            if not self._game_over_logged:
                self._log_decision("GAME_OVER", f"Result recorded: {win_str}.")
                self._game_over_logged = True
            self._draw_board()
            messagebox.showinfo("Game Over", f"The game has ended.\nResult: {win_str}")
            return
        self._game_over_logged = False

        # Precalculate legal moves
        self.legal_moves_cache = self.game.rules.get_legal_moves(self.game.board, self.game.current_turn)

        # Update counter indicators
        bm, bk = self.game.board.count_pieces(CB_BLACK)
        wm, wk = self.game.board.count_pieces(CB_WHITE)
        self.score_var.set(f"Red (Black): {bm}m + {bk}k = {bm+bk}\nWhite: {wm}m + {wk}k = {wm+wk}")

        # Update active turn status header
        turn_str = "Red's Turn" if self.game.current_turn == CB_BLACK else "White's Turn"
        if any(m.is_capture for m in self.legal_moves_cache):
            turn_str += "  [⚡ Mandatory Capture]"
        self._set_status(turn_str)

        # Enable/disable human inputs depending on turn ownership
        is_human = (self.game.current_turn == self.game.human_colour)
        if self.ros_bridge_mode:
            self.btn_undo.state(['disabled'])
            self.btn_reset.state(['disabled'])
        else:
            self.btn_undo.state(['!disabled'] if is_human and len(self.game.board_history) > 2 else ['disabled'])
            self.btn_reset.state(['!disabled'])

        turn_signature = (self.game.move_number, self.game.current_turn)
        if turn_signature != self._last_turn_signature:
            if is_human:
                self._log_decision(
                    "WAIT_HUMAN_MOVE",
                    f"Awaiting {self.game.human_name}. {len(self.legal_moves_cache)} legal move(s) available.",
                )
            else:
                if self.ros_bridge_mode:
                    self._log_decision(
                        "WAIT_ROBOT",
                        f"Waiting for ROS state machine to execute {self.game.robot_name}'s turn.",
                    )
                else:
                    self._log_decision(
                        "PLAN_ROBOT_MOVE",
                        f"{self.game.robot_name} to move. Search starts from {len(self.legal_moves_cache)} legal move(s).",
                    )
            self._last_turn_signature = turn_signature

        self._draw_board()

        # Automatically execute AI thread if active
        if self.ros_bridge_mode and not is_human:
            self._set_status("ROS game manager is planning / executing the robot move...")
            return

        if not is_human and not self.ai_thinking and not self.animating_move:
            self.ai_thinking = True
            self._set_status("🤖 AI Engine Thinking...")
            self.root.update_idletasks()
            threading.Thread(target=self._run_ai_thread, daemon=True).start()

    def _on_click(self, event):
        """Map mouse clicks to grid tiles to preview paths or dispatch moves."""
        if self.ai_thinking or self.animating_move or self.game.current_turn != self.game.human_colour:
            return

        # Translate canvas clicks to internal coordinates
        col = (event.x - self.board_offset_x) // self.sq_size
        perspective = self.game.human_colour
        r_grid = (event.y - self.board_offset_y) // self.sq_size
        row = (7 - r_grid) if perspective == CB_BLACK else r_grid

        if col < 0 or col > 7 or r_grid < 0 or r_grid > 7:
            return

        rc = (row, col)
        sq = ROWCOL_TO_INTERNAL.get(rc)
        if sq is None:
            if self.selected_sq is not None:
                self._attempt_illegal_dark_square_move(self.selected_sq, row, col)
            return

        piece = self.game.board.b[sq]

        # Clicked on a highlighted target destination → EXECUTE MOVE
        if sq in self.valid_targets:
            move = self.valid_targets[sq]
            self._log_decision("VALIDATE_HUMAN_MOVE", f"Accepted {move.to_notation()} as a legal move.")
            self._execute_move(move, self.game.human_name)
            return

        # Clicked on own piece → SELECT & COMPUTE DESTINATIONS
        if piece != FREE and (piece & 0x03) == self.game.human_colour:
            self._select_square(sq)
            return

        # Any other playable destination after selecting a piece is treated as a human illegal attempt.
        if self.selected_sq is not None and sq != self.selected_sq:
            self._attempt_illegal_move(self.selected_sq, sq)
            return

        self.selected_sq = None
        self.valid_targets.clear()
        self._draw_board()

    def _run_ai_thread(self):
        """Invoke Alpha-Beta engine seamlessly in background to avoid blocking Tk UI."""
        start = time.monotonic()
        stats = self.game.search.find_best_move(self.game.board, self.game.robot_colour)
        elapsed = time.monotonic() - start

        # Ensure UI updates route safely through UI controller thread loop
        def _apply_ai():
            self.ai_thinking = True
            if stats.best_move:
                self._log_decision(
                    "PLAN_ROBOT_MOVE",
                    f"{self.game.robot_name} selects {stats.best_move.to_notation()} "
                    f"(depth {stats.depth_reached}, nodes {stats.nodes}, {stats.time_elapsed:.2f}s).",
                )
                # Add minimal reading delay so interaction feels incredibly natural
                sleep_needed = max(0.0, 0.4 - elapsed)
                if sleep_needed > 0:
                    time.sleep(sleep_needed)
                self.ai_thinking = False
                self._execute_move(stats.best_move, self.game.robot_name)
            else:
                self._log_decision("PLAN_ROBOT_MOVE", f"{self.game.robot_name} found no valid response and resigns.")
                messagebox.showinfo("Game Over", "AI has no valid responses and resigns!")
                self.ai_thinking = False
                self._update_state()

        self.root.after(10, _apply_ai)

    def _undo_round(self):
        """Take back previous AI round sequence execution."""
        if self.ros_bridge_mode:
            self._set_status("Undo is disabled while ROS bridge mode is active.", error=True)
            return
        if self.ai_thinking or self.animating_move:
            return
        if self.game.undo_last_round():
            self._last_turn_signature = None
            self._update_state()
            self._set_status("Round execution rolled back successfully.")

    def _reset_game(self):
        """Reset match configuration cleanly."""
        if self.ros_bridge_mode:
            self._set_status("Reset is disabled while ROS bridge mode is active. Restart the app instead.", error=True)
            return
        if self.ai_thinking:
            return
        if messagebox.askyesno("Confirm Reset", "Reset board to standard opening format?"):
            self.game.__init__(human_colour=self.game.human_colour,
                               ai_time=self.game.search.max_time,
                               ai_depth=self.game.search.max_depth)
            self.selected_sq = None
            self.valid_targets.clear()
            self.display_board = None
            self.preview_piece = None
            self.preview_target_rc = None
            self.animating_move = False
            self.ai_thinking = False
            self._last_turn_signature = None
            self._game_over_logged = False
            self._update_state()


# ─── Console CLI Loop (Fallback Mode) ───────────────────────────────────────

def run_console_game(game: CheckersGame):
    """Pure interactive terminal loop flow."""
    print(f"\n  {C.BOLD}{'═' * 50}{C.RESET}")
    print(f"  {C.BOLD}{C.CYAN}   ♔  ENGLISH CHECKERS CONSOLE  ♔{C.RESET}")
    print(f"  {C.BOLD}{'═' * 50}{C.RESET}\n")

    while True:
        game_over, winner = game.rules.is_game_over(game.board, game.current_turn)
        if game_over:
            print(f"\n  {C.BOLD}{C.YELLOW}Game Over: {winner.upper()}{C.RESET}")
            break

        print(render_board(game.board, game.last_move, game.human_colour))

        bm, bk = game.board.count_pieces(CB_BLACK)
        wm, wk = game.board.count_pieces(CB_WHITE)
        active_str = "Red" if game.current_turn == CB_BLACK else "White"
        print(f"\n  Move {game.move_number+1} | {C.BOLD}{active_str}'s turn{C.RESET} | "
              f"Red: {bm}+{bk} vs White: {wm}+{wk}")

        if game.current_turn == game.human_colour:
            legal_moves = game.rules.get_legal_moves(game.board, game.human_colour)
            if any(m.is_capture for m in legal_moves):
                print(f"  {C.YELLOW}⚡ Mandatory Capture Enforced{C.RESET}")

            try:
                raw = input(f"  {C.GREEN}Enter move{C.RESET} (e.g. 11-15, moves, quit): ").strip().lower()
            except (EOFError, KeyboardInterrupt):
                break

            if raw in ('q', 'quit', 'exit'):
                break
            elif raw in ('moves', 'm'):
                for idx, m in enumerate(legal_moves, 1):
                    print(f"    {idx}. {m.to_notation()}")
                continue

            parsed = parse_move_input(raw)
            if parsed:
                m = find_matching_move(parsed[0], parsed[1], legal_moves)
                if m:
                    game.log_decision("VALIDATE_HUMAN_MOVE", f"Accepted {m.to_notation()} as a legal move.")
                    game.apply_move(m)
                else:
                    attempted = f"{parsed[0]}-{parsed[1]}"
                    game.log_decision("ILLEGAL_MOVE_RESPONSE", f"Rejected attempted move {attempted}.")
                    print(f"  {C.RED}Illegal move attempted; board restored. Please choose again.{C.RESET}")
        else:
            print(f"  {C.YELLOW}🤖 AI thinking...{C.RESET}")
            stats = game.search.find_best_move(game.board, game.robot_colour)
            if stats.best_move:
                game.log_decision("PLAN_ROBOT_MOVE", f"AI selected {stats.best_move.to_notation()}.")
                print(f"  🤖 Engine plays: {stats.best_move.to_notation()}")
                game.apply_move(stats.best_move)
            else:
                print("  AI resigns.")
                break


# ─── System Entry Integration Dispatcher ────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Standalone English checkers interactive demo.")
    parser.add_argument('--terminal', action='store_true', help="Launch interactive console mode instead of GUI.")
    parser.add_argument('--colour', '-c', choices=['black', 'red', 'white'], default='black', help="Player alignment.")
    parser.add_argument('--time', '-t', type=float, default=2.0, help="AI search budget duration (seconds).")
    parser.add_argument('--ros-bridge', action='store_true',
                        help="Publish GUI board updates to ROS and mirror robot moves from the ROS game manager.")

    args = parser.parse_args()
    if args.terminal and args.ros_bridge:
        parser.error("--ros-bridge requires GUI mode.")
    human_colour = CB_BLACK if args.colour in ('black', 'red') else CB_WHITE

    game = CheckersGame(human_colour=human_colour, ai_time=args.time)

    # Launch GUI unless explicit terminal requested or Tk is unavailable
    if not args.terminal and HAS_TKINTER and os.environ.get('DISPLAY', True):
        root = tk.Tk()
        CheckersGUI(root, game, ros_bridge_mode=args.ros_bridge)
        root.mainloop()
    else:
        run_console_game(game)


if __name__ == '__main__':
    main()
