#!/usr/bin/env python3
"""
launch_gui.py - GUI Dashboard to launch ROS2 nodes in external terminals.
"""

import os
import shlex
import signal
import subprocess
import tempfile
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk, messagebox


class LaunchGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("MTRX4701_A4 Launch Dashboard")
        self.root.update_idletasks()

        screen_width = self.root.winfo_screenwidth()
        screen_height = self.root.winfo_screenheight()
        width = int(screen_width * 0.45)
        self.root.geometry(f"{width}x{screen_height}+0+0")

        self.ws_path = os.path.dirname(os.path.abspath(__file__))
        self.setup_cmd = (
            f"cd {shlex.quote(self.ws_path)} && "
            "source /opt/ros/jazzy/setup.bash && "
            "source install/setup.bash"
        )

        self.processes = {}
        self._terminal_records = {}
        self.status_queue = queue.Queue()
        self.game_status_var = tk.StringVar(value="Status: not started")
        self._create_widgets()
        self.start_game_status_listener()
        self.poll_game_status_queue()


    def start_game_status_listener(self):
        """Listen to /game/status and show the latest message in the GUI."""
        def worker():
            cmd = (
                f"{self.setup_cmd} && "
                "ros2 topic echo /game/status std_msgs/msg/String --field data"
            )

            while True:
                try:
                    proc = subprocess.Popen(
                        ["bash", "-lc", cmd],
                        stdout=subprocess.PIPE,
                        stderr=subprocess.DEVNULL,
                        text=True,
                        bufsize=1,
                    )

                    for line in proc.stdout:
                        text = line.strip()
                        if not text or text == "---":
                            continue
                        self.status_queue.put(text)

                    proc.wait()
                except Exception:
                    pass

                time.sleep(1.0)

        threading.Thread(target=worker, daemon=True).start()

    def poll_game_status_queue(self):
        try:
            while True:
                status = self.status_queue.get_nowait()
                self.game_status_var.set(f"Status: {status}")
        except queue.Empty:
            pass

        self.root.after(200, self.poll_game_status_queue)

    def _safe_title(self, name):
        return f"MTRX4701 - {name}"

    def _safe_name(self, name):
        return "".join(c if c.isalnum() or c in "_-" else "_" for c in name)

    def _pid_file_for(self, name):
        return os.path.join(tempfile.gettempdir(), f"mtrx4701_{self._safe_name(name)}.pid")

    def _shell_pid_file_for(self, name):
        return os.path.join(tempfile.gettempdir(), f"mtrx4701_{self._safe_name(name)}_terminal_shell.pid")

    def _read_child_pid(self, name):
        record = self._terminal_records.get(name, {})
        pid_file = record.get("pid_file") or self._pid_file_for(name)

        try:
            with open(pid_file, "r", encoding="utf-8") as f:
                return int(f.read().strip())
        except Exception:
            return None

    def open_terminal(self, command, name, custom_setup=None):
        setup = custom_setup if custom_setup is not None else self.setup_cmd
        title = self._safe_title(name)
        pid_file = self._pid_file_for(name)
        shell_pid_file = self._shell_pid_file_for(name)

        # There are two important PIDs:
        #   1. shell_pid_file: the bash running inside the terminal window.
        #      Closing this exits the actual terminal tab/window.
        #   2. pid_file: the ROS command's own process-group leader.
        #      Sending SIGINT to this is equivalent to Ctrl+C for ros2 launch.
        inner_command = f"{setup} && export PYTHONUNBUFFERED=1 && exec {command}"
        wrapper = (
            f"printf '\\033]0;{title}\\007'; "
            f"echo $$ > {shlex.quote(shell_pid_file)}; "
            f"rm -f {shlex.quote(pid_file)}; "
            "trap 'if [ -n \"$child\" ]; then kill -INT -$child 2>/dev/null; sleep 1; kill -TERM -$child 2>/dev/null; fi; exit 0' TERM HUP; "
            f"setsid bash -lc {shlex.quote(inner_command)} & "
            "child=$!; "
            f"echo $child > {shlex.quote(pid_file)}; "
            "wait $child; "
            "status=$?; "
            f"rm -f {shlex.quote(pid_file)}; "
            "echo; "
            f"echo '[{name}] process exited with status '$status'. Terminal left open.'; "
            "exec bash"
        )

        terminals = [
            ["gnome-terminal", f"--title={title}", "--", "bash", "-lc", wrapper],
            ["x-terminal-emulator", "-T", title, "-e", "bash", "-lc", wrapper],
            ["konsole", "--new-tab", "--title", title, "-e", "bash", "-lc", wrapper],
            ["xfce4-terminal", "--title", title, "--command", f"bash -lc {shlex.quote(wrapper)}"],
            ["xterm", "-T", title, "-e", "bash", "-lc", wrapper],
        ]

        for terminal_cmd in terminals:
            try:
                proc = subprocess.Popen(
                    terminal_cmd,
                    preexec_fn=os.setsid,
                )
                self.processes[name] = proc
                self._terminal_records[name] = {
                    "terminal_proc": proc,
                    "pid_file": pid_file,
                    "shell_pid_file": shell_pid_file,
                    "title": title,
                }
                return
            except FileNotFoundError:
                continue
            except Exception as e:
                messagebox.showerror("Terminal Error", f"Failed to open terminal:\n{e}")
                return

        messagebox.showerror(
            "Terminal Error",
            "No supported terminal found. Install gnome-terminal, x-terminal-emulator, konsole, xfce4-terminal, or xterm.",
        )

    # Backwards-compatible name for existing button callbacks.
    def run_embedded(self, command, name, custom_setup=None):
        self.open_terminal(command, name, custom_setup=custom_setup)

    def kill_process(self, name, show_errors=True):
        child_pid = self._read_child_pid(name)

        if child_pid is None:
            if show_errors:
                messagebox.showinfo("Kill", f"No running process PID found for {name}.")
            return False

        try:
            os.killpg(child_pid, signal.SIGINT)
            return True
        except ProcessLookupError:
            return False
        except Exception as e:
            if show_errors:
                messagebox.showerror("Kill Error", f"Failed to send Ctrl+C to {name}:\n{e}")
            return False

    def _read_int_file(self, path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                return int(f.read().strip())
        except Exception:
            return None

    def close_terminal(self, name):
        def worker():
            # First ask ROS to shut down cleanly. The logs you saw mean this
            # part is working: rclcpp received SIGINT/SIGTERM.
            self.kill_process(name, show_errors=False)
            time.sleep(1.0)
            self.kill_process(name, show_errors=False)
            time.sleep(1.0)

            record = self._terminal_records.get(name, {})

            # Now close the actual bash running inside the terminal window.
            # This is more reliable than killing the gnome-terminal launcher
            # process, because that launcher often exits immediately.
            shell_pid = self._read_int_file(record.get("shell_pid_file", ""))
            if shell_pid is not None:
                try:
                    os.kill(shell_pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                except Exception:
                    pass
                time.sleep(0.5)
                try:
                    os.kill(shell_pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                except Exception:
                    pass

            # Fallback for terminals where the above is not enough.
            terminal_proc = record.get("terminal_proc") or self.processes.get(name)
            if terminal_proc is not None:
                try:
                    os.killpg(os.getpgid(terminal_proc.pid), signal.SIGTERM)
                except Exception:
                    try:
                        terminal_proc.terminate()
                    except Exception:
                        pass

            for key in ("pid_file", "shell_pid_file"):
                path = record.get(key)
                if path:
                    try:
                        os.remove(path)
                    except Exception:
                        pass

            self.processes.pop(name, None)
            self._terminal_records.pop(name, None)

        threading.Thread(target=worker, daemon=True).start()

    def create_launch_kill(self, parent, text, launch_func, name):
        frame = ttk.Frame(parent)
        ttk.Button(frame, text=text, command=launch_func).pack(side=tk.LEFT, padx=(0, 2), pady=2)
        ttk.Button(frame, text="Kill", command=lambda: self.kill_process(name), width=5).pack(side=tk.LEFT, padx=(0, 2), pady=2)
        ttk.Button(frame, text="Close", command=lambda: self.close_terminal(name), width=6).pack(side=tk.LEFT, pady=2)
        return frame

    def _create_widgets(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        canvas = tk.Canvas(main_frame)
        scrollbar = ttk.Scrollbar(main_frame, orient="vertical", command=canvas.yview)
        self.scrollable_frame = ttk.Frame(canvas)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: canvas.configure(scrollregion=canvas.bbox("all")),
        )

        canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        canvas.configure(yscrollcommand=scrollbar.set)
        canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        self.scrollable_frame.columnconfigure(0, weight=1)
        row = 0

        ttk.Label(
            self.scrollable_frame,
            text="1. Robot & Arm Control",
            font=("Helvetica", 14, "bold"),
        ).grid(row=row, column=0, pady=(15, 5), sticky="w")
        row += 1

        f_arm = ttk.Frame(self.scrollable_frame)
        f_arm.grid(row=row, column=0, sticky="nw", pady=5)

        ttk.Label(f_arm, text="Robot IP:").pack(anchor="w")
        self.e_robot_ip = ttk.Entry(f_arm, width=15)
        self.e_robot_ip.insert(0, "192.168.56.101")
        self.e_robot_ip.pack(anchor="w", pady=(0, 10))

        self.create_launch_kill(f_arm, "Launch Arm (Hardware)", self.launch_arm, "arm_hw").pack(anchor="w")
        self.create_launch_kill(f_arm, "Launch MoveIt", self.launch_moveit, "moveit").pack(anchor="w")
        row += 1

        ttk.Label(
            self.scrollable_frame,
            text="2. Gripper",
            font=("Helvetica", 14, "bold"),
        ).grid(row=row, column=0, pady=(15, 5), sticky="w")
        row += 1

        f_gripper = ttk.Frame(self.scrollable_frame)
        f_gripper.grid(row=row, column=0, sticky="nw", pady=5)

        ttk.Label(f_gripper, text="Port:").pack(anchor="w")
        self.e_port = ttk.Entry(f_gripper, width=15)
        self.e_port.insert(0, "/dev/ttyUSB0")
        self.e_port.pack(anchor="w", pady=(0, 10))

        self.create_launch_kill(f_gripper, "Launch Gripper", self.launch_gripper, "gripper").pack(anchor="w")

        f_gcmd = ttk.Frame(f_gripper)
        f_gcmd.pack(anchor="w", pady=(10, 0))

        ttk.Label(f_gcmd, text="Open Width:").grid(row=0, column=0, sticky="w")
        self.e_open_width = ttk.Entry(f_gcmd, width=10)
        self.e_open_width.insert(0, "0.025")
        self.e_open_width.grid(row=0, column=1, padx=5)

        ttk.Label(f_gcmd, text="Closed Width:").grid(row=1, column=0, sticky="w")
        self.e_closed_width = ttk.Entry(f_gcmd, width=10)
        self.e_closed_width.insert(0, "0.019")
        self.e_closed_width.grid(row=1, column=1, padx=5)

        self.create_launch_kill(f_gripper, "Run Gripper Command Node", self.run_gripper_command, "gripper_cmd").pack(anchor="w", pady=5)
        row += 1

        ttk.Label(
            self.scrollable_frame,
            text="3. Camera",
            font=("Helvetica", 14, "bold"),
        ).grid(row=row, column=0, pady=(15, 5), sticky="w")
        row += 1

        f_cam = ttk.Frame(self.scrollable_frame)
        f_cam.grid(row=row, column=0, sticky="nw", pady=5)
        self.create_launch_kill(f_cam, "Launch Camera", self.launch_camera, "camera").pack(anchor="w")
        self.create_launch_kill(f_cam, "Launch RQT", self.launch_rqt, "rqt").pack(anchor="w")
        row += 1

        ttk.Label(
            self.scrollable_frame,
            text="4. Environment Setup",
            font=("Helvetica", 14, "bold"),
        ).grid(row=row, column=0, pady=(15, 5), sticky="w")
        row += 1

        f_env = ttk.Frame(self.scrollable_frame)
        f_env.grid(row=row, column=0, sticky="nw", pady=5)

        ttk.Label(f_env, text="Bounding Box:", font=("Helvetica", 10, "bold")).pack(anchor="w", pady=(0, 5))
        f_bbox = ttk.Frame(f_env)
        f_bbox.pack(anchor="w", pady=(0, 10))

        self.bbox_entries = {}
        bbox_params = [
            ("board_x", "-0.075"),
            ("board_y", "0.20"),
            ("board_z", "0.0"),
            ("front_dist", "0.50"),
            ("back_dist", "0.50"),
            ("right_dist", "1.00"),
            ("left_dist", "0.40"),
        ]

        for i, (param, def_val) in enumerate(bbox_params):
            ttk.Label(f_bbox, text=f"{param}:").grid(row=i // 2, column=(i % 2) * 2, padx=5, pady=2, sticky="e")
            ent = ttk.Entry(f_bbox, width=6)
            ent.insert(0, def_val)
            ent.grid(row=i // 2, column=(i % 2) * 2 + 1, sticky="w")
            self.bbox_entries[param] = ent

        self.create_launch_kill(f_env, "Launch Bounding Box", self.launch_bounding_box, "bbox").pack(anchor="w")

        ttk.Label(f_env, text="Marker:", font=("Helvetica", 10, "bold")).pack(anchor="w", pady=(10, 5))
        f_marker = ttk.Frame(f_env)
        f_marker.pack(anchor="w", pady=(0, 10))

        self.marker_entries = {}
        marker_params = [
            ("origin_x", "-0.075"),
            ("origin_y", "0.020"),
            ("origin_z", "0.00"),
            ("square_size", "0.05"),
            ("rotation_steps", "2"),
        ]

        for i, (param, def_val) in enumerate(marker_params):
            ttk.Label(f_marker, text=f"{param}:").grid(row=i // 2, column=(i % 2) * 2, padx=5, pady=2, sticky="e")
            ent = ttk.Entry(f_marker, width=6)
            ent.insert(0, def_val)
            ent.grid(row=i // 2, column=(i % 2) * 2 + 1, sticky="w")
            self.marker_entries[param] = ent

        self.create_launch_kill(f_env, "Publish Checker Board Marker", self.launch_checkerboard_marker, "cb_marker").pack(anchor="w")

        ttk.Label(f_env, text="Pose Node:", font=("Helvetica", 10, "bold")).pack(anchor="w", pady=(10, 5))
        f_pose = ttk.Frame(f_env)
        f_pose.pack(anchor="w", pady=(0, 10))

        self.pose_entries = {}
        pose_params = [
            ("x", "-0.075"),
            ("y", "0.20"),
            ("z", "0.00"),
            ("square_size", "0.05"),
            ("rotation_steps", "2"),
            ("hover_height", "0.25"),
            ("descent_height", "0.08"),
            ("velocity_scaling", "0.08"),
            ("acceleration_scaling", "0.05"),
            ("lift_height", "0.08"),
        ]

        for i, (param, def_val) in enumerate(pose_params):
            ttk.Label(f_pose, text=f"{param}:").grid(row=i // 3, column=(i % 3) * 2, padx=5, pady=2, sticky="e")
            ent = ttk.Entry(f_pose, width=5)
            ent.insert(0, def_val)
            ent.grid(row=i // 3, column=(i % 3) * 2 + 1, sticky="w")
            self.pose_entries[param] = ent

        self.create_launch_kill(f_env, "Run Checkerboard Pose Node", self.run_checkerboard_pose, "cb_pose").pack(anchor="w")
        row += 1

        ttk.Label(
            self.scrollable_frame,
            text="5. Controllers",
            font=("Helvetica", 14, "bold"),
        ).grid(row=row, column=0, pady=(15, 5), sticky="w")
        row += 1

        f_robot_ctrl = ttk.Frame(self.scrollable_frame)
        f_robot_ctrl.grid(row=row, column=0, sticky="nw", pady=5)
        self.create_launch_kill(f_robot_ctrl, "Run Robot Controller", self.run_robot_controller, "robot_ctrl").pack(anchor="w")
        row += 1

        f_game_ctrl = ttk.Frame(self.scrollable_frame)
        f_game_ctrl.grid(row=row, column=0, sticky="nw", pady=5)
        game_button_frame = self.create_launch_kill(
            f_game_ctrl,
            "Run Game Controller",
            self.run_game_controller,
            "game_ctrl",
        )
        game_button_frame.pack(side=tk.LEFT, anchor="w")
        ttk.Label(
            f_game_ctrl,
            textvariable=self.game_status_var,
            wraplength=420,
            foreground="blue",
        ).pack(side=tk.LEFT, padx=(10, 0), anchor="w")
        row += 1

        ttk.Label(
            self.scrollable_frame,
            text="6. Perception",
            font=("Helvetica", 14, "bold"),
        ).grid(row=row, column=0, pady=(15, 5), sticky="w")
        row += 1

        f_perc = ttk.Frame(self.scrollable_frame)
        f_perc.grid(row=row, column=0, sticky="nw", pady=5)

        ttk.Label(f_perc, text="YAML Path:").pack(anchor="w")
        self.e_yaml = ttk.Entry(f_perc, width=40)
        self.e_yaml.insert(0, "src/perception/config/checkers_perception.yaml")
        self.e_yaml.pack(anchor="w", pady=(0, 10))

        self.create_launch_kill(f_perc, "Run Checkers Perception", self.run_perception, "perception").pack(anchor="w")
        row += 1

    def launch_arm(self):
        ip = self.e_robot_ip.get()
        cmd = f"ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:={shlex.quote(ip)}"
        self.run_embedded(cmd, "arm_hw")

    def launch_moveit(self):
        cmd = "ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e launch_rviz:=true"
        self.run_embedded(cmd, "moveit")

    def launch_gripper(self):
        port = self.e_port.get()
        cmd = f"ros2 launch robotiq_hande_driver gripper_controller_preview.launch.py use_fake_hardware:=false tty_port:={shlex.quote(port)}"
        custom_setup = "cd ~/robotiq-hande && source /opt/ros/jazzy/setup.bash && source install/setup.bash"
        self.run_embedded(cmd, "gripper", custom_setup=custom_setup)

    def run_gripper_command(self):
        open_w = self.e_open_width.get()
        closed_w = self.e_closed_width.get()
        cmd = (
            "ros2 run ur5e_manoeuvring gripper_command_node --ros-args "
            f"-p arm_model:=ur5e -p open_position:={shlex.quote(open_w)} -p closed_position:={shlex.quote(closed_w)}"
        )
        self.run_embedded(cmd, "gripper_cmd")

    def launch_bounding_box(self):
        args = []
        for param, ent in self.bbox_entries.items():
            args.append(f"-p {param}:={shlex.quote(ent.get())}")
        cmd = f"ros2 run ur5e_manoeuvring bounding_box_node --ros-args {' '.join(args)}"
        self.run_embedded(cmd, "bbox")

    def launch_camera(self):
        cmd = "ros2 launch realsense2_camera rs_launch.py"
        self.run_embedded(cmd, "camera")

    def launch_checkerboard_marker(self):
        args = []
        for param, ent in self.marker_entries.items():
            args.append(f"-p {param}:={shlex.quote(ent.get())}")
        cmd = f"ros2 run ur5e_manoeuvring chessboard_marker_node --ros-args {' '.join(args)}"
        self.run_embedded(cmd, "cb_marker")

    def run_checkerboard_pose(self):
        args = []
        for param, ent in self.pose_entries.items():
            p_name = f"origin_{param}" if param in ["x", "y", "z"] else param
            args.append(f"-p {p_name}:={shlex.quote(ent.get())}")
        cmd = f"ros2 run ur5e_manoeuvring checkerboard_pose_node --ros-args {' '.join(args)}"
        self.run_embedded(cmd, "cb_pose")

    def run_robot_controller(self):
        cmd = "ros2 run ur5e_manoeuvring ur5e_cartesian_node"
        self.run_embedded(cmd, "robot_ctrl")

    def run_game_controller(self):
        cmd = "ros2 run game_state_machine game_controller"
        self.run_embedded(cmd, "game_ctrl")

    def run_perception(self):
        yaml_path = self.e_yaml.get()
        cmd = f"ros2 run perception checkers_perception --ros-args --params-file {shlex.quote(yaml_path)}"
        self.run_embedded(cmd, "perception")

    def launch_rqt(self):
        cmd = "ros2 run rqt_image_view rqt_image_view /checkers/warped_view"
        self.run_embedded(cmd, "rqt")


if __name__ == "__main__":
    root = tk.Tk()
    app = LaunchGUI(root)
    root.mainloop()
