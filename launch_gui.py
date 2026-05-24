#!/usr/bin/env python3
"""
launch_gui.py - Cross-platform GUI Dashboard to launch and embed ROS2 nodes.
"""

import os
import tkinter as tk
from tkinter import ttk, messagebox
import subprocess
import signal
import platform
import threading

class LaunchGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("MTRX4701_A4 Launch Dashboard")
        self.root.update_idletasks()
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        # Make the window wider to fit split-screen
        width = int(screen_width * 0.85)  
        self.root.geometry(f"{width}x{screen_height}+0+0")
        
        self.ws_path = os.path.dirname(os.path.abspath(__file__))
        self.setup_cmd = f"cd {self.ws_path} && source /opt/ros/jazzy/setup.bash && source install/setup.bash"
        self.processes = {}

        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.root.bind("<Control-c>", lambda e: self.on_closing())
        self.root.bind("<Control-q>", lambda e: self.on_closing())

        self._create_widgets()

    def on_closing(self):
        """Cleanup all processes before closing the GUI."""
        for name in list(self.processes.keys()):
            self.kill_process(name)
        self.root.destroy()

    def run_embedded(self, command, name, text_widget, custom_setup=None):
        if name in self.processes and self.processes[name].poll() is None:
            messagebox.showwarning("Already Running", f"Process '{name}' is already running.")
            return

        setup = custom_setup if custom_setup is not None else self.setup_cmd
        full_command = f"{setup} && export PYTHONUNBUFFERED=1 && {command}"
        
        if text_widget:
            text_widget.config(state="normal")
            text_widget.delete("1.0", tk.END)
            text_widget.insert(tk.END, f"--- Starting {name} ---\n")
            text_widget.see(tk.END)
            text_widget.config(state="disabled")

        try:
            p = subprocess.Popen(
                ["/bin/bash", "-c", full_command],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid if platform.system() != "Windows" else None
            )
            self.processes[name] = p
            
            if text_widget:
                t = threading.Thread(target=self._read_output, args=(p, text_widget), daemon=True)
                t.start()
        except Exception as e:
            messagebox.showerror("Launch Error", f"Failed to start {name}:\n{e}")

    def _read_output(self, process, text_widget):
        for line in iter(process.stdout.readline, ''):
            if not line:
                break
            self.root.after(0, self._append_text, text_widget, line)
        process.stdout.close()

    def _append_text(self, text_widget, text):
        text_widget.config(state="normal")
        text_widget.insert(tk.END, text)
        text_widget.see(tk.END)
        text_widget.config(state="disabled")

    def kill_process(self, name, text_widget=None):
        if name in self.processes:
            p = self.processes[name]
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)
                    p.wait(timeout=2)
                except Exception as e:
                    pass
            del self.processes[name]
            
            if text_widget:
                text_widget.config(state="normal")
                text_widget.insert(tk.END, f"\n--- Process {name} Killed ---\n")
                text_widget.see(tk.END)
                text_widget.config(state="disabled")

    def create_launch_kill(self, parent, text, launch_func, name, text_widget):
        f = ttk.Frame(parent)
        ttk.Button(f, text=text, command=launch_func).pack(side=tk.LEFT, padx=(0, 2), pady=2)
        ttk.Button(f, text="Kill", command=lambda: self.kill_process(name, text_widget), width=5).pack(side=tk.LEFT, pady=2)
        return f

    def create_text_widget(self, parent, height=6):
        t = tk.Text(parent, height=height, width=80, state="disabled", bg="#1e1e1e", fg="#00ff00", font=("Courier", 10))
        return t

    def _create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        canvas = tk.Canvas(main_frame)
        scrollbar = ttk.Scrollbar(main_frame, orient="vertical", command=canvas.yview)
        self.scrollable_frame = ttk.Frame(canvas)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all"))
        )
        canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Configure columns
        self.scrollable_frame.columnconfigure(0, weight=1, minsize=400)
        self.scrollable_frame.columnconfigure(1, weight=3)

        row = 0

        # --- 1. Robot & Arm Control ---
        ttk.Label(self.scrollable_frame, text="1. Robot & Arm Control", font=("Helvetica", 14, "bold")).grid(row=row, column=0, columnspan=2, pady=(15, 5), sticky="w")
        row += 1

        # Arm Hardware
        f_arm = ttk.Frame(self.scrollable_frame)
        f_arm.grid(row=row, column=0, sticky="nw", pady=5)
        self.t_arm = self.create_text_widget(self.scrollable_frame)
        self.t_arm.grid(row=row, column=1, sticky="nsew", padx=10, pady=5)
        ttk.Label(f_arm, text="Robot IP:").pack(anchor="w")
        self.e_robot_ip = ttk.Entry(f_arm, width=15)
        self.e_robot_ip.insert(0, "192.168.56.101")
        self.e_robot_ip.pack(anchor="w", pady=(0, 10))
        self.create_launch_kill(f_arm, "Launch Arm (Hardware)", self.launch_arm, "arm_hw", self.t_arm).pack(anchor="w")
        row += 1

        # MoveIt
        f_moveit = ttk.Frame(self.scrollable_frame)
        f_moveit.grid(row=row, column=0, sticky="nw", pady=5)
        self.t_moveit = self.create_text_widget(self.scrollable_frame)
        self.t_moveit.grid(row=row, column=1, sticky="nsew", padx=10, pady=5)
        self.create_launch_kill(f_moveit, "Launch MoveIt", self.launch_moveit, "moveit", self.t_moveit).pack(anchor="w")
        row += 1

        # --- 2. Gripper ---
        ttk.Label(self.scrollable_frame, text="2. Gripper", font=("Helvetica", 14, "bold")).grid(row=row, column=0, columnspan=2, pady=(15, 5), sticky="w")
        row += 1

        # Gripper Launch
        f_gripper = ttk.Frame(self.scrollable_frame)
        f_gripper.grid(row=row, column=0, sticky="nw", pady=5)
        self.t_gripper = self.create_text_widget(self.scrollable_frame)
        self.t_gripper.grid(row=row, column=1, sticky="nsew", padx=10, pady=5)
        ttk.Label(f_gripper, text="Port:").pack(anchor="w")
        self.e_port = ttk.Entry(f_gripper, width=15)
        self.e_port.insert(0, "/dev/ttyUSB0")
        self.e_port.pack(anchor="w", pady=(0, 10))
        self.create_launch_kill(f_gripper, "Launch Gripper", self.launch_gripper, "gripper", self.t_gripper).pack(anchor="w")
        row += 1

        # Gripper Command Node
        f_gcmd = ttk.Frame(self.scrollable_frame)
        f_gcmd.grid(row=row, column=0, sticky="nw", pady=5)
        self.t_gripper_cmd = self.create_text_widget(self.scrollable_frame)
        self.t_gripper_cmd.grid(row=row, column=1, sticky="nsew", padx=10, pady=5)
        ttk.Label(f_gcmd, text="Open Width:").grid(row=0, column=0, sticky="w")
        self.e_open_width = ttk.Entry(f_gcmd, width=10)
        self.e_open_width.insert(0, "0.025")
        self.e_open_width.grid(row=0, column=1, padx=5)
        ttk.Label(f_gcmd, text="Closed Width:").grid(row=1, column=0, sticky="w")
        self.e_closed_width = ttk.Entry(f_gcmd, width=10)
        self.e_closed_width.insert(0, "0.019")
        self.e_closed_width.grid(row=1, column=1, padx=5)
        self.create_launch_kill(f_gcmd, "Run Gripper Command Node", self.run_gripper_command, "gripper_cmd", self.t_gripper_cmd).grid(row=2, column=0, columnspan=2, sticky="w", pady=5)
        row += 1

        # --- 3. Camera ---
        ttk.Label(self.scrollable_frame, text="3. Camera", font=("Helvetica", 14, "bold")).grid(row=row, column=0, columnspan=2, pady=(15, 5), sticky="w")
        row += 1

        # Camera
        f_cam = ttk.Frame(self.scrollable_frame)
        f_cam.grid(row=row, column=0, sticky="nw", pady=5)
        self.t_cam = self.create_text_widget(self.scrollable_frame)
        self.t_cam.grid(row=row, column=1, sticky="nsew", padx=10, pady=5)
        self.create_launch_kill(f_cam, "Launch Camera", self.launch_camera, "camera", self.t_cam).pack(anchor="w")
        row += 1

        # RQT
        f_rqt = ttk.Frame(self.scrollable_frame)
        f_rqt.grid(row=row, column=0, sticky="nw", pady=5)
        self.t_rqt = self.create_text_widget(self.scrollable_frame)
        self.t_rqt.grid(row=row, column=1, sticky="nsew", padx=10, pady=5)
        self.create_launch_kill(f_rqt, "Launch RQT", self.launch_rqt, "rqt", self.t_rqt).pack(anchor="w")
        row += 1

        # --- 4. Environment Setup ---
        ttk.Label(self.scrollable_frame, text="4. Environment Setup", font=("Helvetica", 14, "bold")).grid(row=row, column=0, columnspan=2, pady=(15, 5), sticky="w")
        row += 1

        # Bounding Box
        f_bbox = ttk.Frame(self.scrollable_frame)
        f_bbox.grid(row=row, column=0, sticky="nw", pady=5)
        self.t_bbox = self.create_text_widget(self.scrollable_frame)
        self.t_bbox.grid(row=row, column=1, sticky="nsew", padx=10, pady=5)
        ttk.Label(f_bbox, text="Bounding Box:", font=("Helvetica", 10, "bold")).pack(anchor="w", pady=(0, 5))
        f_bbox_params = ttk.Frame(f_bbox)
        f_bbox_params.pack(anchor="w", pady=(0, 10))
        self.bbox_entries = {}
        bbox_params = [("board_x", "-0.075"), ("board_y", "0.20"), ("board_z", "0.0"), 
                       ("front_dist", "0.50"), ("back_dist", "0.50"), ("right_dist", "1.00"), ("left_dist", "0.40")]
        for i, (param, def_val) in enumerate(bbox_params):
            ttk.Label(f_bbox_params, text=f"{param}:").grid(row=i//2, column=(i%2)*2, padx=5, pady=2, sticky="e")
            ent = ttk.Entry(f_bbox_params, width=6)
            ent.insert(0, def_val)
            ent.grid(row=i//2, column=(i%2)*2+1, sticky="w")
            self.bbox_entries[param] = ent
        self.create_launch_kill(f_bbox, "Launch Bounding Box", self.launch_bounding_box, "bbox", self.t_bbox).pack(anchor="w")
        row += 1

        # Checker Board Marker
        f_marker = ttk.Frame(self.scrollable_frame)
        f_marker.grid(row=row, column=0, sticky="nw", pady=5)
        self.t_marker = self.create_text_widget(self.scrollable_frame)
        self.t_marker.grid(row=row, column=1, sticky="nsew", padx=10, pady=5)
        ttk.Label(f_marker, text="Marker:", font=("Helvetica", 10, "bold")).pack(anchor="w", pady=(0, 5))
        f_marker_params = ttk.Frame(f_marker)
        f_marker_params.pack(anchor="w", pady=(0, 10))
        self.marker_entries = {}
        marker_params = [("origin_x", "-0.075"), ("origin_y", "0.020"), ("origin_z", "0.00"), ("square_size", "0.05"), ("rotation_steps", "2")]
        for i, (param, def_val) in enumerate(marker_params):
            ttk.Label(f_marker_params, text=f"{param}:").grid(row=i//2, column=(i%2)*2, padx=5, pady=2, sticky="e")
            ent = ttk.Entry(f_marker_params, width=6)
            ent.insert(0, def_val)
            ent.grid(row=i//2, column=(i%2)*2+1, sticky="w")
            self.marker_entries[param] = ent
        self.create_launch_kill(f_marker, "Publish Checker Board Marker", self.launch_checkerboard_marker, "cb_marker", self.t_marker).pack(anchor="w")
        row += 1

        # Checkerboard Pose Node
        f_pose = ttk.Frame(self.scrollable_frame)
        f_pose.grid(row=row, column=0, sticky="nw", pady=5)
        self.t_pose = self.create_text_widget(self.scrollable_frame)
        self.t_pose.grid(row=row, column=1, sticky="nsew", padx=10, pady=5)
        ttk.Label(f_pose, text="Pose Node:", font=("Helvetica", 10, "bold")).pack(anchor="w", pady=(0, 5))
        f_pose_params = ttk.Frame(f_pose)
        f_pose_params.pack(anchor="w", pady=(0, 10))
        self.pose_entries = {}
        pose_params = [("x", "-0.075"), ("y", "0.20"), ("z", "0.00"), ("square_size", "0.05"), ("rotation_steps", "2"), 
                       ("hover_height", "0.25"), ("descent_height", "0.08"), ("velocity_scaling", "0.08"), 
                       ("acceleration_scaling", "0.05"), ("lift_height", "0.08")]
        for i, (param, def_val) in enumerate(pose_params):
            ttk.Label(f_pose_params, text=f"{param}:").grid(row=i//3, column=(i%3)*2, padx=5, pady=2, sticky="e")
            ent = ttk.Entry(f_pose_params, width=5)
            ent.insert(0, def_val)
            ent.grid(row=i//3, column=(i%3)*2+1, sticky="w")
            self.pose_entries[param] = ent
        self.create_launch_kill(f_pose, "Run Checkerboard Pose Node", self.run_checkerboard_pose, "cb_pose", self.t_pose).pack(anchor="w")
        row += 1

        # --- 5. Controllers ---
        ttk.Label(self.scrollable_frame, text="5. Controllers", font=("Helvetica", 14, "bold")).grid(row=row, column=0, columnspan=2, pady=(15, 5), sticky="w")
        row += 1
        
        # Robot Controller
        f_robot_ctrl = ttk.Frame(self.scrollable_frame)
        f_robot_ctrl.grid(row=row, column=0, sticky="nw", pady=5)
        self.t_robot_ctrl = self.create_text_widget(self.scrollable_frame)
        self.t_robot_ctrl.grid(row=row, column=1, sticky="nsew", padx=10, pady=5)
        self.create_launch_kill(f_robot_ctrl, "Run Robot Controller", self.run_robot_controller, "robot_ctrl", self.t_robot_ctrl).pack(anchor="w")
        row += 1
        
        # Game Controller
        f_game_ctrl = ttk.Frame(self.scrollable_frame)
        f_game_ctrl.grid(row=row, column=0, sticky="nw", pady=5)
        self.t_game_ctrl = self.create_text_widget(self.scrollable_frame)
        self.t_game_ctrl.grid(row=row, column=1, sticky="nsew", padx=10, pady=5)
        self.create_launch_kill(f_game_ctrl, "Run Game Controller", self.run_game_controller, "game_ctrl", self.t_game_ctrl).pack(anchor="w")
        row += 1

        # --- 6. Perception ---
        ttk.Label(self.scrollable_frame, text="6. Perception", font=("Helvetica", 14, "bold")).grid(row=row, column=0, columnspan=2, pady=(15, 5), sticky="w")
        row += 1
        f_perc = ttk.Frame(self.scrollable_frame)
        f_perc.grid(row=row, column=0, sticky="nw", pady=5)
        self.t_perc = self.create_text_widget(self.scrollable_frame)
        self.t_perc.grid(row=row, column=1, sticky="nsew", padx=10, pady=5)
        
        ttk.Label(f_perc, text="YAML Path:").pack(anchor="w")
        self.e_yaml = ttk.Entry(f_perc, width=40)
        self.e_yaml.insert(0, "src/perception/config/checkers_perception.yaml")
        self.e_yaml.pack(anchor="w", pady=(0, 10))
        self.create_launch_kill(f_perc, "Run Checkers Perception", self.run_perception, "perception", self.t_perc).pack(anchor="w")
        row += 1

    # --- Button Callbacks ---
    def launch_arm(self):
        ip = self.e_robot_ip.get()
        cmd = f"ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:={ip}"
        self.run_embedded(cmd, "arm_hw", self.t_arm)

    def launch_moveit(self):
        cmd = "ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true"
        self.run_embedded(cmd, "moveit", self.t_moveit)

    def launch_gripper(self):
        port = self.e_port.get()
        cmd = f"ros2 launch robotiq_hande_driver gripper_controller_preview.launch.py use_fake_hardware:=false tty_port:={port}"
        custom_setup = "cd ~/robotiq-hande && source /opt/ros/jazzy/setup.bash && source install/setup.bash"
        self.run_embedded(cmd, "gripper", self.t_gripper, custom_setup=custom_setup)

    def run_gripper_command(self):
        open_w = self.e_open_width.get()
        closed_w = self.e_closed_width.get()
        cmd = f"ros2 run ur5e_manoeuvring gripper_command_node --ros-args -p arm_model:=ur5e -p open_position:={open_w} -p closed_position:={closed_w}"
        self.run_embedded(cmd, "gripper_cmd", self.t_gripper_cmd)

    def launch_bounding_box(self):
        args = []
        for param, ent in self.bbox_entries.items():
            args.append(f"-p {param}:={ent.get()}")
        cmd = f"ros2 run ur5e_manoeuvring bounding_box_node --ros-args {' '.join(args)}"
        self.run_embedded(cmd, "bbox", self.t_bbox)

    def launch_camera(self):
        cmd = "ros2 launch realsense2_camera rs_launch.py"
        self.run_embedded(cmd, "camera", self.t_cam)

    def launch_checkerboard_marker(self):
        args = []
        for param, ent in self.marker_entries.items():
            args.append(f"-p {param}:={ent.get()}")
        cmd = f"ros2 run ur5e_manoeuvring chessboard_marker_node --ros-args {' '.join(args)}"
        self.run_embedded(cmd, "cb_marker", self.t_marker)

    def run_checkerboard_pose(self):
        args = []
        for param, ent in self.pose_entries.items():
            p_name = param
            if param in ['x', 'y', 'z']:
                p_name = f"origin_{param}"
            args.append(f"-p {p_name}:={ent.get()}")
        cmd = f"ros2 run ur5e_manoeuvring checkerboard_pose_node --ros-args {' '.join(args)}"
        self.run_embedded(cmd, "cb_pose", self.t_pose)

    def run_robot_controller(self):
        cmd = "ros2 run ur5e_manoeuvring ur5e_cartesian_node"
        self.run_embedded(cmd, "robot_ctrl", self.t_robot_ctrl)

    def run_game_controller(self):
        cmd = "ros2 run game_state_machine game_controller"
        self.run_embedded(cmd, "game_ctrl", self.t_game_ctrl)

    def run_perception(self):
        yaml_path = self.e_yaml.get()
        cmd = f"ros2 run perception checkers_perception --ros-args --params-file {yaml_path}"
        self.run_embedded(cmd, "perception", self.t_perc)

    def launch_rqt(self):
        cmd = "ros2 run rqt_image_view rqt_image_view /checkers/warped_view"
        self.run_embedded(cmd, "rqt", self.t_rqt)

if __name__ == "__main__":
    root = tk.Tk()
    app = LaunchGUI(root)
    root.mainloop()
