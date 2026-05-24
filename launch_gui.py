#!/usr/bin/env python3
"""
launch_gui.py - Cross-platform GUI Dashboard to launch ROS2 nodes in the background.
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
        # Position Tkinter GUI on the Left Hand Side (LHS)
        self.root.update_idletasks()
        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        width = int(screen_width * 0.35)  # Takes up 35% of the screen width
        self.root.geometry(f"{width}x{screen_height}+0+0")
        
        self.ws_path = os.path.dirname(os.path.abspath(__file__))
        
        # Base setup commands to run before any ROS command
        self.setup_cmd = f"cd {self.ws_path} && source /opt/ros/jazzy/setup.bash && source install/setup.bash"

        self.processes = {}

        self._create_widgets()

    def run_in_terminal(self, command, name, custom_setup=None):
        """Helper to spawn a new Terminal, execute the ROS command, and name the window."""
        setup = custom_setup if custom_setup is not None else self.setup_cmd
        
        if platform.system() == "Darwin":
            full_command = f"{setup} && {command}"
            applescript = f'''
            tell application "Terminal"
                set newTab to do script "{full_command}"
                set custom title of newTab to "{name}"
                activate
            end tell
            '''
            try:
                subprocess.run(["osascript", "-e", applescript], check=True)
            except Exception as e:
                messagebox.showerror("Launch Error", f"Failed to open terminal:\n{e}")
        else:
            # On Linux, we write the bash PID to a temp file, then `exec` the ROS node 
            # so it inherits that exact PID. This lets us kill it perfectly without wmctrl.
            wrapper = f"echo $$ > /tmp/ros_gui_{name}.pid && {setup} && exec {command}"
            try:
                subprocess.Popen(["gnome-terminal", "--title", name, "--", "bash", "-c", wrapper])
            except Exception as e:
                messagebox.showerror("Launch Error", f"Failed to open terminal:\n{e}")

    def kill_terminal(self, name):
        """Helper to find and kill the foreground terminal window or background process."""
        if name in self.processes:
            p = self.processes[name]
            if p.poll() is None:
                try:
                    os.killpg(os.getpgid(p.pid), signal.SIGINT)
                    p.wait(timeout=2)
                except Exception as e:
                    pass
            del self.processes[name]
            return

        if platform.system() == "Darwin":
            applescript = f'''
            tell application "Terminal"
                set windowList to windows
                repeat with w in windowList
                    set tabList to tabs of w
                    repeat with t in tabList
                        if custom title of t is "{name}" then
                            close w
                        end if
                    end repeat
                end repeat
            end tell
            '''
            try:
                subprocess.run(["osascript", "-e", applescript], check=True)
            except Exception as e:
                messagebox.showerror("Kill Error", f"Failed to close terminal:\n{e}")
        else:
            pid_file = f"/tmp/ros_gui_{name}.pid"
            if os.path.exists(pid_file):
                try:
                    with open(pid_file, "r") as f:
                        pid_str = f.read().strip()
                        if pid_str:
                            pid = int(pid_str)
                            import signal
                            # Send SIGINT to smoothly shut down the ROS 2 node
                            os.kill(pid, signal.SIGINT)
                except ProcessLookupError:
                    pass # Process already exited
                except Exception as e:
                    messagebox.showerror("Kill Error", f"Failed to close '{name}' terminal:\n{e}")
                finally:
                    try:
                        os.remove(pid_file)
                    except:
                        pass

    def create_launch_kill(self, parent, text, launch_func, name):
        f = ttk.Frame(parent)
        ttk.Button(f, text=text, command=launch_func).pack(side=tk.LEFT, padx=(0, 2))
        ttk.Button(f, text="Kill", command=lambda: self.kill_terminal(name), width=5).pack(side=tk.LEFT)
        return f

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

        row = 0

        # --- 1. Robot & Arm Control ---
        ttk.Label(self.scrollable_frame, text="1. Robot & Arm Control", font=("Helvetica", 14, "bold")).grid(row=row, column=0, columnspan=2, pady=(10, 5), sticky="w")
        row += 1

        f_arm = ttk.Frame(self.scrollable_frame)
        f_arm.grid(row=row, column=0, columnspan=2, sticky="w", pady=5)
        ttk.Label(f_arm, text="Robot IP:").pack(side=tk.LEFT, padx=(0,5))
        self.e_robot_ip = ttk.Entry(f_arm, width=15)
        self.e_robot_ip.insert(0, "192.168.56.101")
        self.e_robot_ip.pack(side=tk.LEFT, padx=(0,10))
        self.create_launch_kill(f_arm, "Launch Arm (Hardware)", self.launch_arm, "arm_hw").pack(side=tk.LEFT, padx=5)
        self.create_launch_kill(f_arm, "Launch MoveIt", self.launch_moveit, "moveit").pack(side=tk.LEFT, padx=5)
        row += 1

        # --- 2. Gripper ---
        ttk.Label(self.scrollable_frame, text="2. Gripper", font=("Helvetica", 14, "bold")).grid(row=row, column=0, columnspan=2, pady=(15, 5), sticky="w")
        row += 1

        f_gripper = ttk.Frame(self.scrollable_frame)
        f_gripper.grid(row=row, column=0, columnspan=2, sticky="w")
        ttk.Label(f_gripper, text="Port:").pack(side=tk.LEFT, padx=(0,5))
        self.e_port = ttk.Entry(f_gripper, width=15)
        self.e_port.insert(0, "/dev/ttyUSB0")
        self.e_port.pack(side=tk.LEFT, padx=(0,10))
        self.create_launch_kill(f_gripper, "Launch Gripper", self.launch_gripper, "gripper").pack(side=tk.LEFT)
        row += 1

        f_gripper_cmd = ttk.Frame(self.scrollable_frame)
        f_gripper_cmd.grid(row=row, column=0, columnspan=2, sticky="w", pady=5)
        ttk.Label(f_gripper_cmd, text="Open Width:").pack(side=tk.LEFT, padx=(0,5))
        self.e_open_width = ttk.Entry(f_gripper_cmd, width=10)
        self.e_open_width.insert(0, "0.025")
        self.e_open_width.pack(side=tk.LEFT, padx=(0,10))
        ttk.Label(f_gripper_cmd, text="Closed Width:").pack(side=tk.LEFT, padx=(0,5))
        self.e_closed_width = ttk.Entry(f_gripper_cmd, width=10)
        self.e_closed_width.insert(0, "0.019")
        self.e_closed_width.pack(side=tk.LEFT, padx=(0,10))
        self.create_launch_kill(f_gripper_cmd, "Run Gripper Command Node", self.run_gripper_command, "gripper_cmd").pack(side=tk.LEFT)
        row += 1

        # --- 3. Camera ---
        ttk.Label(self.scrollable_frame, text="3. Camera", font=("Helvetica", 14, "bold")).grid(row=row, column=0, columnspan=2, pady=(15, 5), sticky="w")
        row += 1

        self.create_launch_kill(self.scrollable_frame, "Launch Camera", self.launch_camera, "camera").grid(row=row, column=0, padx=5, pady=5, sticky="w")
        self.create_launch_kill(self.scrollable_frame, "Launch RQT", self.launch_rqt, "rqt").grid(row=row, column=1, padx=5, pady=5, sticky="w")
        row += 1

        # --- 4. Environment Setup (Bounding Box, Marker, Pose Node) ---
        ttk.Label(self.scrollable_frame, text="4. Environment Setup", font=("Helvetica", 14, "bold")).grid(row=row, column=0, columnspan=2, pady=(15, 5), sticky="w")
        row += 1

        # Bounding Box
        f_bbox = ttk.Frame(self.scrollable_frame)
        f_bbox.grid(row=row, column=0, columnspan=2, sticky="w")
        self.bbox_entries = {}
        bbox_params = [
            ("board_x", "-0.075"), 
            ("board_y", "0.20"), 
            ("board_z", "0.0"), 
            ("front_dist", "0.50"), 
            ("back_dist", "0.50"), 
            ("right_dist", "1.00"), 
            ("left_dist", "0.40")
        ]
        for i, (param, def_val) in enumerate(bbox_params):
            ttk.Label(f_bbox, text=f"{param}:").grid(row=i//4, column=(i%4)*2, padx=5, pady=2, sticky="e")
            ent = ttk.Entry(f_bbox, width=8)
            ent.insert(0, def_val)
            ent.grid(row=i//4, column=(i%4)*2+1, padx=5, pady=2, sticky="w")
            self.bbox_entries[param] = ent
        self.create_launch_kill(self.scrollable_frame, "Launch Bounding Box", self.launch_bounding_box, "bbox").grid(row=row+1, column=0, pady=5, sticky="w")
        row += 2

        # Marker
        f_marker = ttk.Frame(self.scrollable_frame)
        f_marker.grid(row=row, column=0, columnspan=2, sticky="w")
        self.marker_entries = {}
        marker_params = [("origin_x", "-0.075"), ("origin_y", "0.020"), ("origin_z", "0.00"), ("square_size", "0.05"), ("rotation_steps", "2")]
        for i, (param, def_val) in enumerate(marker_params):
            ttk.Label(f_marker, text=f"{param}:").grid(row=i//5, column=(i%5)*2, padx=5, pady=2, sticky="e")
            ent = ttk.Entry(f_marker, width=8)
            ent.insert(0, def_val)
            ent.grid(row=i//5, column=(i%5)*2+1, padx=5, pady=2, sticky="w")
            self.marker_entries[param] = ent
        self.create_launch_kill(self.scrollable_frame, "Publish Checker Board Marker", self.launch_checkerboard_marker, "cb_marker").grid(row=row+1, column=0, pady=5, sticky="w")
        row += 2

        # Pose Node
        f_pose = ttk.Frame(self.scrollable_frame)
        f_pose.grid(row=row, column=0, columnspan=2, sticky="w", pady=5)
        self.pose_entries = {}
        pose_params = [
            ("x", "-0.075"), ("y", "0.20"), ("z", "0.00"), 
            ("square_size", "0.05"), ("rotation_steps", "2"), ("hover_height", "0.25"),
            ("descent_height", "0.08"), ("velocity_scaling", "0.08"), 
            ("acceleration_scaling", "0.05"), ("lift_height", "0.08")
        ]
        for i, (param, def_val) in enumerate(pose_params):
            ttk.Label(f_pose, text=f"{param}:").grid(row=i//4, column=(i%4)*2, padx=5, pady=2, sticky="e")
            ent = ttk.Entry(f_pose, width=8)
            ent.insert(0, def_val)
            ent.grid(row=i//4, column=(i%4)*2+1, padx=5, pady=2, sticky="w")
            self.pose_entries[param] = ent
        self.create_launch_kill(self.scrollable_frame, "Run Checkerboard Pose Node", self.run_checkerboard_pose, "cb_pose").grid(row=row+1, column=0, pady=5, sticky="w")
        row += 2

        # --- 5. Controllers ---
        ttk.Label(self.scrollable_frame, text="5. Controllers", font=("Helvetica", 14, "bold")).grid(row=row, column=0, columnspan=2, pady=(15, 5), sticky="w")
        row += 1

        self.create_launch_kill(self.scrollable_frame, "Run Robot Controller", self.run_robot_controller, "robot_ctrl").grid(row=row, column=0, padx=5, pady=5, sticky="w")
        row += 1

        f_game = ttk.Frame(self.scrollable_frame)
        f_game.grid(row=row, column=0, columnspan=2, sticky="w", pady=5)
        self.create_launch_kill(f_game, "Run Game Controller", self.run_game_controller_in_gui, "game_ctrl").pack(anchor="w")
        
        self.t_game_output = tk.Text(f_game, height=10, width=60, state="disabled", bg="black", fg="white", font=("Courier", 10))
        self.t_game_output.pack(pady=5)
        row += 1

        # --- 6. Perception ---
        ttk.Label(self.scrollable_frame, text="6. Perception", font=("Helvetica", 14, "bold")).grid(row=row, column=0, columnspan=2, pady=(15, 5), sticky="w")
        row += 1

        f_perc = ttk.Frame(self.scrollable_frame)
        f_perc.grid(row=row, column=0, columnspan=2, sticky="w", pady=5)
        ttk.Label(f_perc, text="YAML Path:").pack(side=tk.LEFT, padx=(0,5))
        self.e_yaml = ttk.Entry(f_perc, width=40)
        self.e_yaml.insert(0, "src/perception/config/checkers_perception.yaml")
        self.e_yaml.pack(side=tk.LEFT, padx=(0,10))
        self.create_launch_kill(f_perc, "Run Checkers Perception", self.run_perception, "perception").pack(side=tk.LEFT)
        row += 1

    # --- Button Callbacks ---
    def launch_arm(self):
        ip = self.e_robot_ip.get()
        cmd = f"ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:={ip}"
        self.run_in_terminal(cmd, "arm_hw")

    def launch_moveit(self):
        cmd = "ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true"
        self.run_in_terminal(cmd, "moveit")

    def launch_gripper(self):
        port = self.e_port.get()
        cmd = f"ros2 launch robotiq_hande_driver gripper_controller_preview.launch.py use_fake_hardware:=false tty_port:={port}"
        custom_setup = "cd ~/robotiq-hande && source /opt/ros/jazzy/setup.bash && source install/setup.bash"
        self.run_in_terminal(cmd, "gripper", custom_setup=custom_setup)

    def run_gripper_command(self):
        open_w = self.e_open_width.get()
        closed_w = self.e_closed_width.get()
        cmd = f"ros2 run ur5e_manoeuvring gripper_command_node --ros-args -p arm_model:=ur5e -p open_position:={open_w} -p closed_position:={closed_w}"
        self.run_in_terminal(cmd, "gripper_cmd")

    def launch_bounding_box(self):
        args = []
        for param, ent in self.bbox_entries.items():
            args.append(f"-p {param}:={ent.get()}")
        cmd = f"ros2 run ur5e_manoeuvring bounding_box_node --ros-args {' '.join(args)}"
        self.run_in_terminal(cmd, "bbox")

    def launch_camera(self):
        cmd = "ros2 launch realsense2_camera rs_launch.py"
        self.run_in_terminal(cmd, "camera")

    def launch_checkerboard_marker(self):
        args = []
        for param, ent in self.marker_entries.items():
            args.append(f"-p {param}:={ent.get()}")
        cmd = f"ros2 run ur5e_manoeuvring chessboard_marker_node --ros-args {' '.join(args)}"
        self.run_in_terminal(cmd, "cb_marker")

    def run_checkerboard_pose(self):
        args = []
        for param, ent in self.pose_entries.items():
            p_name = param
            if param in ['x', 'y', 'z']:
                p_name = f"origin_{param}"
            args.append(f"-p {p_name}:={ent.get()}")
        cmd = f"ros2 run ur5e_manoeuvring checkerboard_pose_node --ros-args {' '.join(args)}"
        self.run_in_terminal(cmd, "cb_pose")

    def run_robot_controller(self):
        cmd = "ros2 run ur5e_manoeuvring ur5e_cartesian_node"
        self.run_in_terminal(cmd, "robot_ctrl")

    def run_game_controller_in_gui(self):
        name = "game_ctrl"
        if name in self.processes and self.processes[name].poll() is None:
            messagebox.showwarning("Already Running", f"Process '{name}' is already running.")
            return

        command = "ros2 run game_state_machine game_controller"
        full_command = f"{self.setup_cmd} && export PYTHONUNBUFFERED=1 && {command}"
        
        self.t_game_output.config(state="normal")
        self.t_game_output.delete("1.0", tk.END)
        self.t_game_output.config(state="disabled")

        try:
            p = subprocess.Popen(
                ["/bin/bash", "-c", full_command],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                preexec_fn=os.setsid
            )
            self.processes[name] = p
            
            t = threading.Thread(target=self._read_output, args=(p, self.t_game_output), daemon=True)
            t.start()
        except Exception as e:
            messagebox.showerror("Launch Error", f"Failed to start game controller:\n{e}")

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

    def run_perception(self):
        yaml_path = self.e_yaml.get()
        cmd = f"ros2 run perception checkers_perception --ros-args --params-file {yaml_path}"
        self.run_in_terminal(cmd, "perception")

    def launch_rqt(self):
        cmd = "ros2 run rqt_image_view rqt_image_view /checkers/warped_view"
        self.run_in_terminal(cmd, "rqt")


if __name__ == "__main__":
    root = tk.Tk()
    app = LaunchGUI(root)
    root.mainloop()
