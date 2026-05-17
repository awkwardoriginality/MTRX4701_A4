#!/usr/bin/env python3
"""
simulate_kinematics.py — Premium UR5e Checkers Virtual Environment Suite.

Provides an advanced educational simulation GUI combining:
    1. Direct Forward Kinematics (FK) visualization using UR5e DH parameters.
    2. Real-time Numerical Inverse Kinematics (IK) via Jacobian pseudo-inverse.
    3. Trajectory path verification mapping checkers coordinates to 3D link poses.
    4. Interactive joint jogging sliders and live Cartesian telemetry dashboards.

Grounds physical robot arm integration directly alongside game logic validation.
"""

from __future__ import annotations
import sys
import os
import math
import numpy as np
from typing import List, Tuple, Optional, Dict

# Add parent directories for module paths
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from checkers_bot.game_engine.board import INTERNAL_TO_STANDARD, STANDARD_TO_INTERNAL, INTERNAL_TO_ROWCOL
from checkers_bot.manipulation.board_coordinates import BoardCoordinates

try:
    import tkinter as tk
    from tkinter import ttk, messagebox
    HAS_TK = True
except ImportError:
    HAS_TK = False

try:
    import matplotlib
    matplotlib.use('TkAgg')
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    from mpl_toolkits.mplot3d import Axes3D
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


# ─── UR5e Kinematics Core Engine ────────────────────────────────────────────

class UR5eKinematics:
    """Standard UR5e robot arm kinematics solver based on DH convention.

    DH Table (standard convention):
        Link | d (m)   | a (m)     | alpha (rad)
        ────────────────────────────────────────
        1    | 0.1625  | 0.0       |  pi/2
        2    | 0.0     | -0.425    |  0.0
        3    | 0.0     | -0.3922   |  0.0
        4    | 0.1333  | 0.0       |  pi/2
        5    | 0.0997  | 0.0       | -pi/2
        6    | 0.0996  | 0.0       |  0.0
    """

    # Denavit-Hartenberg parameters
    d = np.array([0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996])
    a = np.array([0.0, -0.425, -0.3922, 0.0, 0.0, 0.0])
    alpha = np.array([math.pi/2, 0.0, 0.0, math.pi/2, -math.pi/2, 0.0])

    @classmethod
    def dh_matrix(cls, theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        """Construct standard DH homogeneous transformation matrix."""
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        return np.array([
            [ct, -st*ca,  st*sa, a*ct],
            [st,  ct*ca, -ct*sa, a*st],
            [0,   sa,     ca,    d   ],
            [0,   0,      0,     1   ]
        ])

    @classmethod
    def forward_kinematics(cls, joints: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray]]:
        """Compute end-effector pose and intermediate link frame origins.

        Args:
            joints: Array of 6 joint angles in radians.

        Returns:
            T_06: 4x4 matrix representing final tool flange pose.
            origins: List of 3D vectors representing joint connection vertices.
        """
        origins = [np.array([0.0, 0.0, 0.0])]
        T = np.eye(4)

        for i in range(6):
            A_i = cls.dh_matrix(joints[i], cls.d[i], cls.a[i], cls.alpha[i])
            T = T @ A_i
            origins.append(T[:3, 3])

        # Add visual representation of parallel-jaw gripper finger tips
        # TCP offset +Z in tool frame points straight out from flange
        p_jaw1 = T @ np.array([0.0,  0.03, 0.15, 1.0])
        p_jaw2 = T @ np.array([0.0, -0.03, 0.15, 1.0])
        p_tcp  = T @ np.array([0.0,  0.0,  0.15, 1.0])

        origins.append(p_tcp[:3])
        origins.append(p_jaw1[:3])
        origins.append(p_jaw2[:3])

        return T, origins

    @classmethod
    def inverse_kinematics(cls, target_pos: np.ndarray, init_joints: np.ndarray,
                           max_iter: int = 100, tol: float = 1e-4) -> Tuple[np.ndarray, bool]:
        """Numerical Inverse Kinematics solver targeting specific Cartesian coordinates.

        Uses iterative Jacobian pseudo-inverse optimization for smooth path mapping.
        Assumes end-effector maintains downward operational posture.
        """
        # Forbid moving below the physical ground plane
        safe_target = target_pos.copy()
        if safe_target[2] < 0.005:
            safe_target[2] = 0.005

        q = np.array(init_joints, dtype=float).copy()

        for _ in range(max_iter):
            T_curr, _ = cls.forward_kinematics(q)
            # Incorporate TCP tool tip projection offset
            p_curr = (T_curr @ np.array([0.0, 0.0, 0.15, 1.0]))[:3]

            err = safe_target - p_curr
            if np.linalg.norm(err) < tol:
                return q, True

            # Compute numerical positional Jacobian (3x6)
            J = np.zeros((3, 6))
            delta = 1e-5
            for j in range(6):
                q_pert = q.copy()
                q_pert[j] += delta
                T_pert, _ = cls.forward_kinematics(q_pert)
                p_pert = (T_pert @ np.array([0.0, 0.0, 0.15, 1.0]))[:3]
                J[:, j] = (p_pert - p_curr) / delta

            # Damped least squares pseudo-inverse step update
            lam = 0.01
            J_pinv = J.T @ np.linalg.inv(J @ J.T + lam**2 * np.eye(3))
            dq = J_pinv @ err

            q += dq

            # Enforce joint configuration bounds to avoid floor clipping
            T_check, origins_check = cls.forward_kinematics(q)
            for pt in origins_check[1:]:
                if pt[2] < 0.0:
                    # Apply a soft restorative lift perturbation to joints influencing Z
                    q[1] -= 0.01
                    q[2] += 0.01

        return q, False


# ─── Virtual Simulation Visualizer Application ──────────────────────────────

class KinematicsEnvironmentApp:
    """Tkinter-embedded interactive multi-view robotics workspace."""

    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("UR5e Virtual Kinematics Environment — Premium Mechatronics Suite")
        self.root.geometry("1200x800")
        self.root.minsize(1000, 700)

        # UI aesthetics system tokens
        self.bg_main = "#1E1E1E"
        self.bg_sidebar = "#252526"
        self.fg_text = "#D4D4D4"
        self.accent_color = "#00FFCC"

        self.root.configure(bg=self.bg_main)

        # Coordinate mappings generator
        self.coords = BoardCoordinates()
        # Custom calibration displacement placing the board comfortably in workspace front
        tf_custom = np.array([
            [1.0, 0.0, 0.0, 0.35],
            [0.0, 1.0, 0.0, -0.20],
            [0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0]
        ])
        self.coords.set_transform(tf_custom)

        # System posture states
        self.joint_vars: List[tk.DoubleVar] = []
        self.joint_labels: List[tk.Label] = []
        self.current_joints = np.array([0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]) # Safe home config

        self.target_pos = np.array([0.5, 0.0, 0.2])
        self.is_animating = False

        self._setup_layout()
        self._draw_initial_scene()
        self._update_telemetry()

    def _setup_layout(self):
        """Construct side dashboards, embedded Matplotlib viewport, and slider controls."""
        main_split = tk.PanedWindow(self.root, orient=tk.HORIZONTAL, bg=self.bg_main, bd=0)
        main_split.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Left panel: Manual jogging, numerical IK targets, and sequence testing
        left_frame = tk.Frame(main_split, bg=self.bg_sidebar, width=380, bd=1, relief=tk.SUNKEN)
        main_split.add(left_frame, width=380)

        # Heading banner
        tk.Label(left_frame, text="UR5e Arm Controller", font=("Helvetica", 14, "bold"),
                 bg=self.bg_sidebar, fg=self.accent_color, pady=10).pack(fill=tk.X)

        # Joint angle real-time continuous sliders
        slider_box = tk.LabelFrame(left_frame, text="Joint Direct Control (rad)", font=("Helvetica", 10, "bold"),
                                   bg=self.bg_sidebar, fg="#AAAAAA", padx=10, pady=5)
        slider_box.pack(fill=tk.X, padx=10, pady=5)

        j_names = ["Shoulder Pan", "Shoulder Lift", "Elbow", "Wrist 1", "Wrist 2", "Wrist 3"]
        j_bounds = [(-math.pi, math.pi), (-math.pi, 0.0), (0.0, math.pi),
                    (-math.pi, math.pi), (-math.pi, math.pi), (-math.pi, math.pi)]

        for i in range(6):
            row_f = tk.Frame(slider_box, bg=self.bg_sidebar)
            row_f.pack(fill=tk.X, pady=2)
            tk.Label(row_f, text=f"J{i+1}: {j_names[i]}", bg=self.bg_sidebar, fg=self.fg_text, width=15, anchor=tk.W).pack(side=tk.LEFT)

            var = tk.DoubleVar(value=self.current_joints[i])
            s = ttk.Scale(row_f, from_=j_bounds[i][0], to=j_bounds[i][1], variable=var, command=lambda val, idx=i: self._on_slider_move(idx, val))
            s.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)

            val_lbl = tk.Label(row_f, text=f"{var.get():.2f}", bg=self.bg_sidebar, fg=self.accent_color, width=5)
            val_lbl.pack(side=tk.RIGHT)
            self.joint_vars.append(var)
            self.joint_labels.append(val_lbl)

        # Checkers automated coordinate generation panel
        target_box = tk.LabelFrame(left_frame, text="Trajectory Planning Target", font=("Helvetica", 10, "bold"),
                                   bg=self.bg_sidebar, fg="#AAAAAA", padx=10, pady=10)
        target_box.pack(fill=tk.X, padx=10, pady=10)

        tk.Label(target_box, text="Standard Square (1–32):", bg=self.bg_sidebar, fg=self.fg_text).pack(anchor=tk.W)
        self.sq_combo = ttk.Combobox(target_box, values=[str(i) for i in range(1, 33)], state="readonly")
        self.sq_combo.set("15")
        self.sq_combo.pack(fill=tk.X, pady=3)

        tk.Label(target_box, text="Action Posture Profile:", bg=self.bg_sidebar, fg=self.fg_text).pack(anchor=tk.W, pady=(5,0))
        self.profile_combo = ttk.Combobox(target_box, values=["Hover Transit (+80mm)", "Approach Drop (+20mm)", "Grasp Piece Surface", "Bob Over Jump (+25mm)"], state="readonly")
        self.profile_combo.set("Hover Transit (+80mm)")
        self.profile_combo.pack(fill=tk.X, pady=3)

        btn_ik = ttk.Button(target_box, text="🎯 Solve Inverse Kinematics", command=self._solve_selected_target)
        btn_ik.pack(fill=tk.X, pady=(8,2))

        btn_seq = ttk.Button(target_box, text="🚀 Simulate Pick Sequence", command=self._animate_pick_sequence)
        btn_seq.pack(fill=tk.X, pady=2)

        # Telemetry Dashboards
        tele_box = tk.LabelFrame(left_frame, text="Cartesian Space Telemetry", font=("Helvetica", 10, "bold"),
                                 bg=self.bg_sidebar, fg="#AAAAAA", padx=10, pady=10)
        tele_box.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.lbl_tele = tk.Label(tele_box, text="", font=("Consolas", 10), bg="#1E1E1E", fg="#7FFF00", justify=tk.LEFT, anchor=tk.NW, padx=8, pady=8)
        self.lbl_tele.pack(fill=tk.BOTH, expand=True)

        # Right panel: 3D Visualization workspace via Matplotlib embedded layout
        right_frame = tk.Frame(main_split, bg=self.bg_main)
        main_split.add(right_frame, width=800)

        self.fig = plt.figure(figsize=(8, 6), facecolor=self.bg_main)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_facecolor(self.bg_main)

        self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _on_slider_move(self, idx: int, val: str):
        """Handle direct joint manipulation."""
        if self.is_animating:
            return
        self.current_joints[idx] = float(val)
        # Synchronize label
        self.joint_labels[idx].configure(text=f"{self.current_joints[idx]:.2f}")

        self._update_scene()

    def _draw_initial_scene(self):
        """Configure 3D viewport spatial limits and static ambient lighting grids."""
        self._update_scene()

    def _update_scene(self):
        """Redraw robot linkage chains and board boundaries in full 3D space."""
        self.ax.clear()

        # Enforce proportional physical scaling metrics
        self.ax.set_xlim(-0.2, 0.8)
        self.ax.set_ylim(-0.5, 0.5)
        self.ax.set_zlim(0.0, 0.8)

        self.ax.set_xlabel('X Axis (m)', color=self.fg_text)
        self.ax.set_ylabel('Y Axis (m)', color=self.fg_text)
        self.ax.set_zlabel('Z Axis (m)', color=self.fg_text)

        self.ax.tick_params(colors=self.fg_text, labelsize=8)
        self.ax.grid(True, color="#333333", linestyle=':', linewidth=0.5)

        # Customize view angle for intuitive desktop perception
        self.ax.view_init(elev=25, azim=-55)

        # 1. Render calibrated Checkers Board Plane
        sq_m = self.coords.sq
        for r in range(8):
            for c in range(8):
                is_dark = (r + c) % 2 == 0
                c_color = "#5C3A21" if is_dark else "#DEB887"

                # Define vertices in board frame
                pts_b = np.array([
                    [c*sq_m,     r*sq_m,     0.0, 1.0],
                    [(c+1)*sq_m, r*sq_m,     0.0, 1.0],
                    [(c+1)*sq_m, (r+1)*sq_m, 0.0, 1.0],
                    [c*sq_m,     (r+1)*sq_m, 0.0, 1.0]
                ])

                # Map surface segments to calibrated physical workspace floor
                pts_w = (self.coords.T @ pts_b.T).T[:, :3]
                x_surf = pts_w[:, 0]
                y_surf = pts_w[:, 1]
                z_surf = pts_w[:, 2]

                self.ax.plot_trisurf(x_surf, y_surf, z_surf, color=c_color, alpha=0.85 if is_dark else 0.4)

        # Render explicit target marker dot
        self.ax.scatter(self.target_pos[0], self.target_pos[1], self.target_pos[2],
                        color="#FF1493", s=100, marker='*', label="Kinematic Target")

        # 2. Compute and plot Forward Kinematics chains
        T_flange, origins = UR5eKinematics.forward_kinematics(self.current_joints)
        pts = np.array(origins)

        # Draw structural robotic joints and linkage arms
        # Base to Wrist segments
        has_collision = False
        for k in range(len(pts) - 1):
            p1 = pts[k]
            p2 = pts[k+1]
            # Check if any part of the segment goes below the physical table surface
            is_below = p1[2] < -0.005 or p2[2] < -0.005
            if is_below:
                has_collision = True
            seg_color = "#FF0000" if is_below else ("#007ACC" if k < 6 else "#FFA500")
            lw = 5 if k < 6 else 3
            self.ax.plot([p1[0], p2[0]], [p1[1], p2[1]], [p1[2], p2[2]],
                         color=seg_color, linewidth=lw, marker='o' if k < 6 else '',
                         markersize=6, markerfacecolor="#FFFFFF")

        # Draw parallel-jaw gripping fingers
        jaw_color = "#FF0000" if (pts[7, 2] < -0.005 or pts[8, 2] < -0.005 or pts[9, 2] < -0.005) else "#32CD32"
        if jaw_color == "#FF0000":
            has_collision = True
        self.ax.plot([pts[7, 0], pts[8, 0]], [pts[7, 1], pts[8, 1]], [pts[7, 2], pts[8, 2]], color=jaw_color, linewidth=3)
        self.ax.plot([pts[7, 0], pts[9, 0]], [pts[7, 1], pts[9, 1]], [pts[7, 2], pts[9, 2]], color=jaw_color, linewidth=3)

        self.ax.legend(loc='upper right', facecolor=self.bg_sidebar, edgecolor="none", labelcolor=self.fg_text)
        self.fig.canvas.draw()
        self._update_telemetry(has_collision)

    def _update_telemetry(self, has_collision: bool = False):
        """Update side matrix display with forward kinematics evaluations."""
        T_curr, _ = UR5eKinematics.forward_kinematics(self.current_joints)
        p_tcp = (T_curr @ np.array([0.0, 0.0, 0.15, 1.0]))[:3]

        err = np.linalg.norm(self.target_pos - p_tcp) * 1000.0 # mm error

        warn_banner = "\n!!! TABLE PENETRATION FORBIDDEN !!!\n" if has_collision else ""

        txt = (
            f"=== TOOL CENTER POINT (TCP) ===\n"
            f"X: {p_tcp[0]:.4f} m\n"
            f"Y: {p_tcp[1]:.4f} m\n"
            f"Z: {p_tcp[2]:.4f} m\n\n"
            f"=== TARGET OBJECTIVES ===\n"
            f"X: {self.target_pos[0]:.4f} m\n"
            f"Y: {self.target_pos[1]:.4f} m\n"
            f"Z: {self.target_pos[2]:.4f} m\n\n"
            f"Tracking Error: {err:.2f} mm\n{warn_banner}\n"
            f"=== ACTIVE JOINT STATES ===\n"
        )
        for i in range(6):
            txt += f"J{i+1}: {np.rad2deg(self.current_joints[i]):+7.2f}°\n"

        self.lbl_tele.configure(text=txt)

    def _solve_selected_target(self):
        """Extract requested coordinate targets and drive IK solution mapping."""
        if self.is_animating:
            return

        std_sq = int(self.sq_combo.get())
        profile = self.profile_combo.get()

        from checkers_bot.game_engine.board import STANDARD_TO_INTERNAL, INTERNAL_TO_ROWCOL
        sq_int = STANDARD_TO_INTERNAL.get(std_sq)
        if sq_int is None:
            return

        row, col = INTERNAL_TO_ROWCOL[sq_int]

        # Determine target altitude from selection profile
        h_offset = 0.0
        if "Hover" in profile:
            h_offset = self.coords.HOVER_HEIGHT
        elif "Approach" in profile:
            h_offset = self.coords.APPROACH_HEIGHT
        elif "Bob" in profile:
            h_offset = self.coords.BOB_HEIGHT
        else:
            h_offset = self.coords.piece_height

        self.target_pos = self.coords.square_to_cartesian(row, col, h_offset)

        # Execute Inverse Kinematics
        sol, success = UR5eKinematics.inverse_kinematics(self.target_pos, self.current_joints)

        # Synchronize UI states
        self.current_joints = sol.copy()
        for idx in range(6):
            self.joint_vars[idx].set(self.current_joints[idx])
            self.joint_labels[idx].configure(text=f"{self.current_joints[idx]:.2f}")

        self._update_scene()
        if success:
            self.lbl_tele.configure(fg="#7FFF00")
        else:
            self.lbl_tele.configure(fg="#FF4500")

    def _animate_pick_sequence(self):
        """Animate smooth transition through full robotic pick sequences."""
        if self.is_animating:
            return

        std_sq = int(self.sq_combo.get())
        sq_int = STANDARD_TO_INTERNAL.get(std_sq)
        if sq_int is None:
            return

        row, col = INTERNAL_TO_ROWCOL[sq_int]

        # Generate sequence keyframes
        waypoints = [
            self.coords.hover_position(row, col),
            self.coords.approach_position(row, col),
            self.coords.grasp_position(row, col),
            self.coords.approach_position(row, col),
            self.coords.hover_position(row, col)
        ]

        self.is_animating = True
        self._run_waypoint_interpolation(waypoints, 0)

    def _run_waypoint_interpolation(self, waypoints: List[np.ndarray], idx: int):
        """Execute linear workspace step interpolation to guarantee smooth visualization playback."""
        if idx >= len(waypoints):
            self.is_animating = False
            return

        target = waypoints[idx]
        self.target_pos = target.copy()

        # Compute required goal joints
        goal_q, _ = UR5eKinematics.inverse_kinematics(target, self.current_joints)

        # Interpolate between current joints and goal joints in 10 smooth steps
        steps = 8
        start_q = self.current_joints.copy()

        def _step(s):
            if s <= steps:
                frac = s / float(steps)
                self.current_joints = start_q + frac * (goal_q - start_q)

                # Synchronize sliders
                for j in range(6):
                    self.joint_vars[j].set(self.current_joints[j])
                    self.joint_labels[j].configure(text=f"{self.current_joints[j]:.2f}")

                self._update_scene()
                self.root.after(40, lambda: _step(s+1))
            else:
                # Proceed to next target waypoint
                self.root.after(200, lambda: self._run_waypoint_interpolation(waypoints, idx+1))

        _step(1)


# ─── Program Dispatcher ─────────────────────────────────────────────────────

def main():
    if not HAS_TK or not HAS_MPL:
        print("Error: The Virtual Environment requires Tkinter and Matplotlib modules.")
        print("Install missing dependencies using: pip install matplotlib numpy")
        return

    root = tk.Tk()
    app = KinematicsEnvironmentApp(root)
    root.mainloop()


if __name__ == '__main__':
    main()
