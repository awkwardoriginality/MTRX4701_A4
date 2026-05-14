# MTRX4701_A4 — UR5e Checkers Robot Simulation

This repository contains the full production-ready source code for an English checkers-playing UR5e robot. It includes a pure-Python checkers engine (ported from SimpleCh), a ROS2 perception and control architecture, and a premium standalone GUI for interactive play and digital twin simulation.

## 🚀 Quick Start (Standalone GUI)

The fastest way to test the checkers simulation is using the standalone `play_checkers.py` script. This requires only standard Python dependencies and does not require a full ROS2 installation for the GUI mode.

```bash
# Install dependencies
pip install -r requirements.txt

# Launch the premium point-and-click GUI
python3 src/checkers_bot/play_checkers.py
```

## 🎮 Playbook: How to Play

1.  **Selection**: Click on a Red piece (your side). Valid target moves will be highlighted with chartreuse dots.
2.  **Execution**: Click a target dot to execute the move. The digital twin UR5e will automatically compute a collision-free trajectory to perform the physical move.
3.  **Rules**: The engine enforces standard English checkers rules, including **mandatory captures** (forced jumps). If a jump is available, you must take it.
4.  **AI Hint**: If you're stuck, click the "💡 AI Hint" button to see the recommended path.
5.  **Multi-Jumps**: The simulation handles multi-jump sequences with a realistic "bobbing" motion for the robotic arm.

## 🤖 ROS2 Package Usage

To use the full ROS2 architecture with a real UR5e and Realsense camera:

### 1. Build the Workspace
```bash
colcon build --packages-select checkers_bot
source install/setup.bash
```

### 2. Launch Perception
```bash
# Launch camera
ros2 launch realsense2_camera rs_launch.py

# Run perception node
ros2 run checkers_bot perception --ros-args \
  -p input_mode:=ros \
  -p image_topic:=/camera/camera/color/image_raw
```

### 3. Run Game Manager
```bash
ros2 run checkers_bot game_manager
```

## 🛠 Prerequisites & Dependencies

-   **Python 3.10+**
-   **OpenCV** (for perception)
-   **Matplotlib** (for 3D digital twin)
-   **Tkinter** (for GUI)
-   **ROS2 Humble** (optional, for hardware integration)

## 📁 Repository Structure

-   `src/checkers_bot/checkers_bot/game_engine/`: Pure Python checkers logic (Rules, Move Gen, Search).
-   `src/checkers_bot/checkers_bot/manipulation/`: Kinematics and coordinate mapping.
-   `src/checkers_bot/checkers_bot/nodes/`: ROS2 nodes (Perception, Manipulation, Game Manager).
-   `src/checkers_bot/play_checkers.py`: The main entry point for the interactive simulation.
-   `src/checkers_bot/simulate_kinematics.py`: Dedicated 3D robot workspace visualizer.
