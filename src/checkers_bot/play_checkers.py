#!/usr/bin/env python3
"""
play_checkers.py — Standalone interactive English checkers demo.

Supports two premium play modes:
    1. Point-and-Click Graphical User Interface (GUI) via Tkinter (Default)
    2. Coloured terminal interactive console (--terminal)

Features:
    - Grounded in the fully ported pure-Python KingsRow checkers engine
    - Validates all English checkers rules (mandatory captures, king promotion)
    - Full point-and-click move preview and piece selection highlighting
    - AI search hints, real-time evaluation scores, and game history logs
    - Parallel-jaw gripper manipulation simulation compatibility

Usage:
    python3 play_checkers.py                  # launches point-and-click GUI
    python3 play_checkers.py --terminal       # launches terminal CLI mode
    python3 play_checkers.py --colour white   # play as white
    python3 play_checkers.py --time 2.0       # AI search time budget
"""

from __future__ import annotations
import sys
import os
import re
import time
import argparse
import threading
from typing import List, Optional, Tuple

# Add parent directory for imports
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))

from checkers_bot.game_engine.board import (
    Board, Move,
    CB_BLACK, CB_WHITE, CB_MAN, CB_KING,
    BLACK_MAN, BLACK_KING, WHITE_MAN, WHITE_KING,
    FREE, OCCUPIED,
    INTERNAL_TO_STANDARD, STANDARD_TO_INTERNAL,
    INTERNAL_TO_ROWCOL, ROWCOL_TO_INTERNAL, PLAYABLE_SQUARES,
)
from checkers_bot.game_engine.move_generator import MoveGenerator
from checkers_bot.game_engine.search import Search
from checkers_bot.game_engine.evaluation import evaluate
from checkers_bot.game_engine.rules import Rules
from checkers_bot.manipulation.board_coordinates import BoardCoordinates
from simulate_kinematics import UR5eKinematics
import numpy as np

try:
    import tkinter as tk
    from tkinter import ttk, messagebox, font
    HAS_TKINTER = True
except ImportError:
    HAS_TKINTER = False

try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


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
        self.board_history: List[Board] = [self.board.copy()]
        self.last_move: Optional[Move] = None

        self.human_name = "Red" if human_colour == CB_BLACK else "White"
        self.robot_name = "White" if human_colour == CB_BLACK else "Red"

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


# ─── Premium Graphical User Interface (GUI Mode) ────────────────────────────

class CheckersGUI:
    """Stunning Point-and-Click Tkinter interface for the checkers demo."""

    def __init__(self, root: tk.Tk, game: CheckersGame):
        self.root = root
        self.game = game

        self.root.title("UR5e Checkers Bot — Premium Game Suite")
        self.root.geometry("1450x750")
        self.root.minsize(1200, 650)

        # Configure dark-mode inspired premium styling
        self.bg_base = "#1E1E1E"
        self.bg_panel = "#252526"
        self.fg_text = "#D4D4D4"
        self.color_dark_sq = "#5C3A21"      # Premium rich walnut
        self.color_light_sq = "#DEB887"     # Smooth warm maple
        self.color_selected = "#00FFCC"     # Vibrant cyan highlight
        self.color_target = "#7FFF00"       # Chartreuse valid target dot

        self.root.configure(bg=self.bg_base)

        # Interaction state
        self.selected_sq: Optional[int] = None
        self.legal_moves_cache: List[Move] = []
        self.valid_targets: dict[int, Move] = {}  # destination_square -> Move object
        self.ai_thinking = False

        # Digital Twin kinematics state
        self.coords = BoardCoordinates()
        self.current_joints = np.array([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0])
        self.target_pos = np.array([0.5, 0.0, 0.2])
        self.twin_animating = False

        self._setup_ui()
        self._update_state()

    def _setup_ui(self):
        """Construct canvas grid, digital twin viewport, and persistent statistics panel."""
        # Main layout frames
        self.main_panes = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, bg=self.bg_base, bd=0)
        self.main_panes.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left Frame: Board Canvas
        self.board_frame = tk.Frame(self.main_panes, bg=self.bg_base)
        self.main_panes.add(self.board_frame, width=580)

        self.canvas = tk.Canvas(self.board_frame, bg=self.bg_base, bd=0, highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        self.canvas.bind("<Configure>", self._on_resize)
        self.canvas.bind("<Button-1>", self._on_click)

        # Middle Frame: 3D Robotic Arm Real-Time Digital Twin Viewport
        self.twin_frame = tk.Frame(self.main_panes, bg=self.bg_base)
        self.main_panes.add(self.twin_frame, width=580)

        if HAS_MPL:
            self.fig = plt.figure(figsize=(6, 6), facecolor=self.bg_base)
            self.ax = self.fig.add_subplot(111, projection='3d')
            self.ax.set_facecolor(self.bg_base)
            self.twin_canvas = FigureCanvasTkAgg(self.fig, master=self.twin_frame)
            self.twin_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Right Frame: Statistics & Controls Sidebar
        self.sidebar = tk.Frame(self.main_panes, bg=self.bg_panel, bd=1, relief=tk.SUNKEN)
        self.main_panes.add(self.sidebar, width=270)

        # Custom heading styling
        title_font = font.Font(family="Helvetica", size=16, weight="bold")
        lbl_title = tk.Label(self.sidebar, text="♔ English Checkers ♔", font=title_font,
                             bg=self.bg_panel, fg="#E5E5E5", pady=10)
        lbl_title.pack(fill=tk.X)

        # Status Bar
        self.status_var = tk.StringVar(value="Game initialized.")
        self.lbl_status = tk.Label(self.sidebar, textvariable=self.status_var,
                                   font=("Helvetica", 12, "bold"), bg=self.bg_panel, fg="#00FFCC", pady=5)
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

        self.btn_hint = ttk.Button(btn_frame, text="💡 AI Hint", command=self._request_hint)
        self.btn_hint.pack(fill=tk.X, pady=3)

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
                piece = self.game.board.b[sq]
                if piece != FREE:
                    pad = self.sq_size // 6
                    px1 = x1 + pad
                    py1 = y1 + pad
                    px2 = x2 - pad
                    py2 = y2 - pad

                    # Styling per player
                    if piece in (BLACK_MAN, BLACK_KING):
                        p_fill = "#DC143C"  # Crimson Red
                        p_out = "#8B0000"
                    else:
                        p_fill = "#F8F8FF"  # Crisp Ivory White
                        p_out = "#A9A9A9"

                    # Shaded 3D layered circles
                    self.canvas.create_oval(px1, py1+2, px2, py2+2, fill="#0F0F0F", outline="") # shadow
                    self.canvas.create_oval(px1, py1, px2, py2, fill=p_fill, outline=p_out, width=2)

                    # Inner ring for premium mechatronic look
                    ipad = pad + (self.sq_size // 12)
                    self.canvas.create_oval(x1+ipad, y1+ipad, x2-ipad, y2-ipad, fill="", outline=p_out, width=1)

                    # King status symbol
                    if piece in (BLACK_KING, WHITE_KING):
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2
                        crown_color = "#FFD700" if piece == BLACK_KING else "#DAA520"
                        self.canvas.create_text(cx, cy, text="♛", font=("Helvetica", max(self.sq_size//3, 14)),
                                                fill=crown_color)
                else:
                    # Subtle square numbering for reference
                    std_num = INTERNAL_TO_STANDARD.get(sq, "")
                    self.canvas.create_text(x1+8, y2-8, text=str(std_num), font=("Helvetica", 8),
                                            fill="#A0A0A0" if is_dark else "")

        self._draw_twin_scene()

    def _update_state(self):
        """Update side panel counters, evaluate turn legality, and drive AI states."""
        self.selected_sq = None
        self.valid_targets.clear()

        # Update lists
        self.log_box.delete(0, tk.END)
        for item in self.game.move_history:
            self.log_box.insert(tk.END, item)
        self.log_box.yview(tk.END)

        # Check Win/Loss status
        game_over, winner = self.game.rules.is_game_over(self.game.board, self.game.current_turn)
        if game_over:
            win_str = "DRAW" if winner == "draw" else f"{winner.upper()} WINS!"
            self.status_var.set(f"Game Over: {win_str}")
            self._draw_board()
            messagebox.showinfo("Game Over", f"The game has ended.\nResult: {win_str}")
            return

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
        self.status_var.set(turn_str)

        # Enable/disable human inputs depending on turn ownership
        is_human = (self.game.current_turn == self.game.human_colour)
        self.btn_hint.state(['!disabled'] if is_human else ['disabled'])
        self.btn_undo.state(['!disabled'] if is_human and len(self.game.board_history) > 2 else ['disabled'])

        self._draw_board()

        # Automatically execute AI thread if active
        if not is_human and not self.ai_thinking and not self.twin_animating:
            self.ai_thinking = True
            self.status_var.set("🤖 AI Engine Thinking...")
            self.root.update_idletasks()
            threading.Thread(target=self._run_ai_thread, daemon=True).start()

    def _draw_twin_scene(self):
        """Redraw live robot digital twin linkage chains and physical checkers board surface."""
        if not HAS_MPL or not hasattr(self, 'ax'):
            return

        self.ax.clear()
        self.ax.set_xlim(-0.2, 0.8)
        self.ax.set_ylim(-0.5, 0.5)
        self.ax.set_zlim(0.0, 0.8)

        self.ax.set_xlabel('X (m)', color=self.fg_text)
        self.ax.set_ylabel('Y (m)', color=self.fg_text)
        self.ax.set_zlabel('Z (m)', color=self.fg_text)
        self.ax.tick_params(colors=self.fg_text, labelsize=8)
        self.ax.grid(True, color="#333333", linestyle=':', linewidth=0.5)
        self.ax.view_init(elev=25, azim=-55)

        # Draw calibrated checkers board ground plane
        sq_m = self.coords.sq
        for r in range(8):
            for c in range(8):
                is_dark = (r + c) % 2 == 0
                c_color = "#5C3A21" if is_dark else "#DEB887"
                pts_b = np.array([
                    [c*sq_m, r*sq_m, 0.0, 1.0],
                    [(c+1)*sq_m, r*sq_m, 0.0, 1.0],
                    [(c+1)*sq_m, (r+1)*sq_m, 0.0, 1.0],
                    [c*sq_m, (r+1)*sq_m, 0.0, 1.0]
                ])
                pts_w = (self.coords.T @ pts_b.T).T[:, :3]
                self.ax.plot_trisurf(pts_w[:, 0], pts_w[:, 1], pts_w[:, 2], color=c_color, alpha=0.85 if is_dark else 0.4)

        # Plot current forward kinematics link frames
        T_flange, origins = UR5eKinematics.forward_kinematics(self.current_joints)
        pts = np.array(origins)

        for k in range(len(pts) - 1):
            p1 = pts[k]
            p2 = pts[k+1]
            seg_color = "#FF0000" if (p1[2] < -0.005 or p2[2] < -0.005) else ("#007ACC" if k < 6 else "#FFA500")
            lw = 5 if k < 6 else 3
            self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
                         color=seg_color, linewidth=lw, marker='o' if k < 6 else '',
                         markersize=5, markerfacecolor="#FFFFFF")

        # Jaw lines
        jaw_color = "#32CD32"
        self.ax.plot([pts[7, 0], pts[8, 0]], [pts[7, 1], pts[8, 1]], [pts[7, 2], pts[8, 2]], color=jaw_color, linewidth=3)
        self.ax.plot([pts[7, 0], pts[9, 0]], [pts[7, 1], pts[9, 1]], [pts[7, 2], pts[9, 2]], color=jaw_color, linewidth=3)

        # Draw all active checkers pieces in 3D sitting flat facing upwards
        p_h = self.coords.piece_height
        target_sq = getattr(self, 'twin_moving_to_sq', None)
        p_tcp = pts[7]

        angles = np.linspace(0, 2 * np.pi, 18)
        cos_a = np.cos(angles)
        sin_a = np.sin(angles)
        radius = 0.018  # 18mm physical checker token radius

        for sq in PLAYABLE_SQUARES:
            piece = self.game.board.b[sq]
            if piece != FREE:
                r, c = INTERNAL_TO_ROWCOL[sq]
                cx_b = (c + 0.5) * sq_m
                cy_b = (r + 0.5) * sq_m

                # Determine if piece is currently lifted by the gripper (carried during trajectory)
                is_carried = False
                if getattr(self, 'twin_animating', False) and target_sq == sq:
                    if p_tcp[2] > 0.015:
                        is_carried = True
                        # Draw checker token flat circle centered directly at tool center point tip
                        pts_ring_w = np.zeros((18, 3))
                        pts_ring_w[:, 0] = p_tcp[0] + radius * cos_a
                        pts_ring_w[:, 1] = p_tcp[1] + radius * sin_a
                        pts_ring_w[:, 2] = p_tcp[2] - 0.005
                        center_w = np.array([p_tcp[0], p_tcp[1], p_tcp[2] - 0.005])

                if not is_carried:
                    # Transform standard perimeter points to world coordinates
                    pts_ring_b = np.zeros((18, 4))
                    pts_ring_b[:, 0] = cx_b + radius * cos_a
                    pts_ring_b[:, 1] = cy_b + radius * sin_a
                    pts_ring_b[:, 2] = p_h
                    pts_ring_b[:, 3] = 1.0
                    pts_ring_w = (self.coords.T @ pts_ring_b.T).T[:, :3]
                    center_w = (self.coords.T @ np.array([cx_b, cy_b, p_h, 1.0]))[:3]

                p_color = "#DC143C" if piece in (BLACK_MAN, BLACK_KING) else "#F8F8FF"
                p_edge = "#8B0000" if piece in (BLACK_MAN, BLACK_KING) else "#A9A9A9"

                # Plot the solid thick perimeter ring lying completely flat parallel to the board plane
                self.ax.plot(pts_ring_w[:, 0], pts_ring_w[:, 1], pts_ring_w[:, 2], color=p_color, linewidth=3.5)

                # Scatter a premium pip icon directly in the flat circle origin
                m_size = 90 if piece in (BLACK_KING, WHITE_KING) else 50
                m_style = '^' if piece in (BLACK_KING, WHITE_KING) else 'o'
                self.ax.scatter(center_w[0], center_w[1], center_w[2], color=p_color, edgecolor=p_edge,
                                s=m_size, marker=m_style, depthshade=True, linewidth=1.5)

        self.fig.canvas.draw()

    def _animate_physical_move(self, move: Move):
        """Drive physical digital twin linkage chains through full Pick-and-Place sequence keyframes."""
        if not HAS_MPL:
            self.twin_animating = False
            self._update_state()
            return

        self.twin_animating = True
        self.status_var.set("🤖 Arm Executing AI Trajectory SAFELY...")
        self.root.update_idletasks()

        r_from, c_from = INTERNAL_TO_ROWCOL[move.from_sq]
        r_to, c_to = INTERNAL_TO_ROWCOL[move.to_sq]

        sequence = []

        # 1. Moving checker sequence: transit hover -> approach -> grasp -> approach -> hover -> transit dest -> approach -> place -> approach -> hover
        for wp in [
            self.coords.hover_position(r_from, c_from),
            self.coords.approach_position(r_from, c_from),
            self.coords.grasp_position(r_from, c_from),
            self.coords.approach_position(r_from, c_from),
            self.coords.hover_position(r_from, c_from),
            self.coords.hover_position(r_to, c_to),
            self.coords.approach_position(r_to, c_to),
            self.coords.grasp_position(r_to, c_to),
            self.coords.approach_position(r_to, c_to),
            self.coords.hover_position(r_to, c_to),
        ]:
            sequence.append((wp, move.to_sq))

        # 2. Pick up each captured opponent checker safely and deposit to discard line
        if getattr(move, 'captured_squares', []):
            for idx, cap_sq in enumerate(move.captured_squares):
                r_cap, c_cap = INTERNAL_TO_ROWCOL[cap_sq]
                discard_p = self.coords.discard_position(idx)
                discard_hover = discard_p.copy()
                discard_hover[2] += self.coords.HOVER_HEIGHT

                for wp in [
                    self.coords.hover_position(r_cap, c_cap),
                    self.coords.approach_position(r_cap, c_cap),
                    self.coords.grasp_position(r_cap, c_cap),
                    self.coords.approach_position(r_cap, c_cap),
                    self.coords.hover_position(r_cap, c_cap),
                    discard_hover,
                    discard_p,
                    discard_hover,
                ]:
                    sequence.append((wp, cap_sq))

        # 3. Retract safely back to default home stance
        sequence.append((self.coords.board_center(), None))

        self._interpolate_twin_waypoints(sequence, 0)

    def _interpolate_twin_waypoints(self, sequence: List[Tuple[np.ndarray, Optional[int]]], idx: int):
        """Smoothly step digital twin TCP along successive Cartesian targets in straight lines to forbid table collisions."""
        if idx >= len(sequence):
            self.twin_animating = False
            self.twin_moving_to_sq = None
            self._update_state()
            return

        target_p, active_sq = sequence[idx]
        self.twin_moving_to_sq = active_sq

        # Find starting Cartesian TCP position from forward kinematics
        T_curr, _ = UR5eKinematics.forward_kinematics(self.current_joints)
        start_p = (T_curr @ np.array([0.0, 0.0, 0.15, 1.0]))[:3]
        goal_p = target_p.copy()

        # Enforce strict ground plane compliance bounds
        if goal_p[2] < 0.005:
            goal_p[2] = 0.005

        steps = 12  # Slow, gentle, highly realistic Cartesian linear steps

        def _step(s):
            if s <= steps:
                frac = s / float(steps)
                inter_p = start_p + frac * (goal_p - start_p)
                # Keep absolute clearance above table mesh plane along the entire path
                if inter_p[2] < 0.005:
                    inter_p[2] = 0.005

                sol_q, success = UR5eKinematics.inverse_kinematics(inter_p, self.current_joints)
                self.current_joints = sol_q.copy()
                self._draw_twin_scene()
                self.root.after(35, lambda: _step(s+1))
            else:
                self.root.after(50, lambda: self._interpolate_twin_waypoints(sequence, idx+1))

        _step(1)

    def _on_click(self, event):
        """Map mouse clicks to grid tiles to preview paths or dispatch moves."""
        if self.ai_thinking or self.twin_animating or self.game.current_turn != self.game.human_colour:
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
            return

        # Clicked on a highlighted target destination → EXECUTE MOVE
        if sq in self.valid_targets:
            move = self.valid_targets[sq]
            self.game.apply_move(move)
            self._update_state()
            return

        # Clicked on own piece → SELECT & COMPUTE DESTINATIONS
        piece = self.game.board.b[sq]
        if piece != FREE and (piece & 0x03) == self.game.human_colour:
            self.selected_sq = sq
            self.valid_targets.clear()

            # Map target squares reachable by this specific selected piece
            for m in self.legal_moves_cache:
                if m.from_sq == sq:
                    self.valid_targets[m.to_sq] = m

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
                # Add minimal reading delay so interaction feels incredibly natural
                sleep_needed = max(0.0, 0.4 - elapsed)
                if sleep_needed > 0:
                    time.sleep(sleep_needed)
                self.game.apply_move(stats.best_move)
                self.ai_thinking = False
                self._draw_board()
                self._animate_physical_move(stats.best_move)
            else:
                messagebox.showinfo("Game Over", "AI has no valid responses and resigns!")
                self.ai_thinking = False
                self._update_state()

        self.root.after(10, _apply_ai)

    def _request_hint(self):
        """Query engine for human hint path recommendations."""
        if self.ai_thinking:
            return
        self.status_var.set("💡 Computing best candidate...")
        self.root.update_idletasks()

        # Run synchronously since human hints are lightweight requested lookups
        stats = self.game.search.find_best_move(self.game.board, self.game.human_colour)
        if stats.best_move:
            hint_str = stats.best_move.to_notation()
            self.status_var.set(f"💡 Recommended Path: {hint_str}")
            # Automatically preview recommended source tile
            self.selected_sq = stats.best_move.from_sq
            self.valid_targets = {stats.best_move.to_sq: stats.best_move}
            self._draw_board()
        else:
            self.status_var.set("No viable hints available.")

    def _undo_round(self):
        """Take back previous AI round sequence execution."""
        if self.ai_thinking:
            return
        if self.game.undo_last_round():
            self._update_state()
            self.status_var.set("Round execution rolled back successfully.")

    def _reset_game(self):
        """Reset match configuration cleanly."""
        if self.ai_thinking:
            return
        if messagebox.askyesno("Confirm Reset", "Reset board to standard opening format?"):
            self.game.__init__(human_colour=self.game.human_colour,
                               ai_time=self.game.search.max_time,
                               ai_depth=self.game.search.max_depth)
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
                    game.apply_move(m)
                else:
                    print(f"  {C.RED}Illegal path chosen.{C.RESET}")
        else:
            print(f"  {C.YELLOW}🤖 AI thinking...{C.RESET}")
            stats = game.search.find_best_move(game.board, game.robot_colour)
            if stats.best_move:
                print(f"  🤖 Engine plays: {stats.best_move.to_notation()}")
                game.apply_move(stats.best_move)
            else:
                print("  AI resigns.")
                break


# ─── System Entry Integration Dispatcher ────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="UR5e Premium Checkers Interactive Demo Interface.")
    parser.add_argument('--terminal', action='store_true', help="Launch interactive console mode instead of GUI.")
    parser.add_argument('--colour', '-c', choices=['black', 'red', 'white'], default='black', help="Player alignment.")
    parser.add_argument('--time', '-t', type=float, default=2.0, help="AI search budget duration (seconds).")

    args = parser.parse_args()
    human_colour = CB_BLACK if args.colour in ('black', 'red') else CB_WHITE

    game = CheckersGame(human_colour=human_colour, ai_time=args.time)

    # Launch GUI unless explicit terminal requested or Tk is unavailable
    if not args.terminal and HAS_TKINTER and os.environ.get('DISPLAY', True):
        root = tk.Tk()
        app = CheckersGUI(root, game)
        root.mainloop()
    else:
        run_console_game(game)


if __name__ == '__main__':
    main()
