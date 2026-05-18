"""
cartesian_bridge_node.py — Bridges ManipulationGoal commands to ur5e Cartesian waypoints.

Subscribes to /checkers/manipulation_goal, expands each command into a sequence
of Cartesian waypoints using PickPlace/BoardCoordinates, and publishes them one
at a time to /ur5e_cartesian_goal. Waits for /ur5e_motion_status confirmation
between waypoints, then signals completion back to the game manager.

Run this instead of manipulation_node when driving the ur5e_cartesian_node.
"""

from __future__ import annotations

import json
import logging
import math
import os
import time
from typing import List, Optional, Tuple

import numpy as np

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String, Bool
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

try:
    import yaml
    from ament_index_python.packages import get_package_share_directory
    HAS_AMENT = True
except ImportError:
    HAS_AMENT = False

from ..protocol import ManipulationFeedback, ManipulationGoal, ManipulationResult
from ..manipulation.board_coordinates import BoardCoordinates
from ..manipulation.pick_place import MotionCommand, MotionType, PickPlace

logger = logging.getLogger(__name__)


# ─── Constants ───────────────────────────────────────────────────────────────

# Fallback transform — derived from chessboard_marker_node params:
#   origin_x=0.30, origin_y=-0.20, square_size=0.05, rotation_steps=1 (90 deg)
#   x_robot = origin_x + 8*sq - y_board = 0.70 - y_board
#   y_robot = origin_y + x_board        = -0.20 + x_board
_DEFAULT_TF = np.array([
    [ 0.0, -1.0, 0.0,  0.70],
    [ 1.0,  0.0, 0.0, -0.20],
    [ 0.0,  0.0, 1.0,  0.00],
    [ 0.0,  0.0, 0.0,  1.00],
], dtype=float)

GRIPPER_OPEN  = 0.025   # metres — jaw open
GRIPPER_CLOSE = 0.0     # metres — jaw closed

# End-effector orientation: gripper pointing straight down
EE_ROLL  = math.pi
EE_PITCH = 0.0
EE_YAW   = 0.0

# TCP z-offset: distance from tool0 (robot flange) down to the gripper tip.
# BoardCoordinates computes positions for the gripper tip, so we shift z up
# by this amount before sending to ur5e_cartesian_node (which plans for tool0).
_DEFAULT_TCP_Z = 0.05   # metres — from robot_params.yaml tcp_offset.z

# Safe home position in robot base frame (above the board centre)
HOME_POSITION = np.array([0.4, 0.0, 0.35])

# Motion durations by speed band (seconds)
_TRANSIT_DURATION  = 3.0   # speed >= 0.25 m/s
_BOB_DURATION      = 3.5   # speed >= 0.12 m/s
_APPROACH_DURATION = 4.0   # slow precision approach

# ur5e_motion_status values that mean the arm/gripper step finished
_ARM_DONE  = 'ARM_MOVE_DONE'
_GRIP_DONE = 'GRIPPER_GOAL_ACCEPTED'

# ur5e_motion_status values that indicate a goal failed
_STATUS_ERROR = frozenset({
    'IK_REQUEST_FAILED', 'IK_SERVICE_NOT_AVAILABLE', 'IK_FAILED',
    'ARM_ACTION_NOT_AVAILABLE', 'ARM_GOAL_REJECTED', 'ARM_MOVE_FAILED',
    'GRIPPER_GOAL_REJECTED',
})


# ─── Helpers ─────────────────────────────────────────────────────────────────

def _load_robot_params() -> tuple[np.ndarray, float]:
    """Load board transform and TCP z-offset from robot_params.yaml.

    Returns (board_to_robot_tf, tcp_z_offset).
    The TCP z-offset is the distance from tool0 (flange) down to the gripper tip.
    All z targets from BoardCoordinates are at the gripper tip, so we add this
    offset before sending to ur5e_cartesian_node which plans for tool0.
    """
    if not HAS_AMENT:
        return _DEFAULT_TF, _DEFAULT_TCP_Z
    try:
        pkg_dir = get_package_share_directory('checkers_bot')
        cfg_path = os.path.join(pkg_dir, 'config', 'robot_params.yaml')
        with open(cfg_path) as f:
            cfg = yaml.safe_load(f)
        tf = np.array(cfg['robot']['board_to_robot_tf'], dtype=float)
        tcp_z = float(cfg['robot']['tcp_offset']['z'])
        return tf, tcp_z
    except Exception as e:
        logger.warning(f"Could not load robot_params.yaml ({e}), using defaults")
        return _DEFAULT_TF, _DEFAULT_TCP_Z


def _motion_duration(cmd: MotionCommand) -> float:
    if cmd.speed >= 0.25:
        return _TRANSIT_DURATION
    if cmd.speed >= 0.12:
        return _BOB_DURATION
    return _APPROACH_DURATION


# ─── Node ────────────────────────────────────────────────────────────────────

class CartesianBridgeNode:
    """Translates high-level checkers ManipulationGoals into Cartesian waypoints.

    Each goal (PICK, PLACE, BOB, etc.) is expanded into a list of MotionCommands
    by PickPlace, then sent one at a time to /ur5e_cartesian_goal. The node waits
    for /ur5e_motion_status to confirm each waypoint before sending the next.
    Once all waypoints for a goal are done it publishes back on
    /checkers/manipulation_result and /checkers/manipulation_done.
    """

    def __init__(self):
        if not HAS_ROS2:
            logger.warning("ROS2 not available — CartesianBridgeNode cannot run")
            return

        rclpy.init()
        self.node = Node('cartesian_bridge_node')

        tf, self._tcp_z = _load_robot_params()
        self.coords = BoardCoordinates(board_to_robot_tf=tf)
        self.pick_place = PickPlace(self.coords)
        logger.info(f"TCP z-offset: {self._tcp_z:.3f} m")

        # ─── Subscribers ─────────────────────────────────────────────────
        self.node.create_subscription(
            String, '/checkers/manipulation_goal', self._goal_callback, 10,
        )
        self.node.create_subscription(
            String, '/ur5e_motion_status', self._status_callback, 10,
        )

        # ─── Publishers ──────────────────────────────────────────────────
        self._cartesian_pub = self.node.create_publisher(
            String, '/ur5e_cartesian_goal', 10,
        )
        self._done_pub = self.node.create_publisher(
            Bool, '/checkers/manipulation_done', 10,
        )
        self._result_pub = self.node.create_publisher(
            String, '/checkers/manipulation_result', 10,
        )
        self._feedback_pub = self.node.create_publisher(
            String, '/checkers/manipulation_feedback', 10,
        )

        # ─── Execution state ─────────────────────────────────────────────
        self._goal_queue: List[Tuple[ManipulationGoal, List[MotionCommand]]] = []
        self._active_goal: Optional[ManipulationGoal] = None
        self._active_motions: List[MotionCommand] = []
        self._motion_idx: int = 0
        self._awaiting_for: Optional[str] = None  # 'arm' | 'gripper' | None
        self._wait_until: float = 0.0
        self._current_gripper: float = GRIPPER_OPEN
        self._last_xyz: List[float] = HOME_POSITION.tolist()

        self.node.create_timer(0.05, self._tick)  # 20 Hz
        logger.info("CartesianBridgeNode initialised")

    # ─── Callbacks ───────────────────────────────────────────────────────

    def _goal_callback(self, msg: String):
        try:
            goal = ManipulationGoal.from_json(msg.data)
        except (TypeError, ValueError, json.JSONDecodeError) as e:
            logger.error(f"Invalid manipulation goal JSON: {e}")
            return

        motions = self._expand_goal(goal)
        if motions is None:
            logger.warning(f"Unsupported command type: {goal.command_type}")
            self._signal_done(goal, success=False, detail="Unsupported command type")
            return

        self._goal_queue.append((goal, motions))
        logger.info(
            f"Queued {goal.command_id}: {goal.command_type} "
            f"row={goal.row} col={goal.col} → {len(motions)} waypoints"
        )

    def _status_callback(self, msg: String):
        # Strip any trailing code (e.g. "ARM_MOVE_FAILED: -3" → "ARM_MOVE_FAILED")
        status = msg.data.split(':')[0].strip()

        done = (
            (self._awaiting_for == 'arm'     and status == _ARM_DONE) or
            (self._awaiting_for == 'gripper' and status == _GRIP_DONE)
        )
        if done:
            self._awaiting_for = None
            logger.info(f"  ✓ {msg.data}")

        elif status in _STATUS_ERROR:
            logger.error(f"  Motion error: {msg.data}")
            if self._awaiting_for is not None and self._active_goal:
                self._awaiting_for = None
                goal = self._active_goal
                self._reset_active_goal()
                self._signal_done(goal, success=False, detail=msg.data)

    # ─── 20 Hz tick ──────────────────────────────────────────────────────

    def _tick(self):
        if self._awaiting_for is not None:
            return
        if time.monotonic() < self._wait_until:
            return

        # Pick up the next queued goal if none is active
        if self._active_goal is None:
            if not self._goal_queue:
                return
            self._active_goal, self._active_motions = self._goal_queue.pop(0)
            self._motion_idx = 0
            self._publish_feedback(self._active_goal, stage='executing')
            logger.info(f"Starting {self._active_goal.command_id}: {self._active_goal.command_type}")

        # All waypoints done — signal completion and clear state
        if self._motion_idx >= len(self._active_motions):
            goal = self._active_goal
            self._reset_active_goal()
            self._signal_done(goal, success=True, detail="Complete")
            return

        self._send_motion(self._active_motions[self._motion_idx])

    # ─── Motion dispatch ─────────────────────────────────────────────────

    def _send_motion(self, cmd: MotionCommand):
        if cmd.motion_type == MotionType.MOVE_TO:
            xyz = cmd.position.tolist()
            self._last_xyz = xyz
            self._publish_cartesian(xyz, _motion_duration(cmd))
            self._awaiting_for = 'arm'
            self._motion_idx += 1
            logger.info(f"  MOVE_TO [{cmd.label}] → {[f'{v:.3f}' for v in xyz]}")

        elif cmd.motion_type == MotionType.GRASP:
            self._set_gripper(GRIPPER_CLOSE, "GRASP — closing gripper")

        elif cmd.motion_type == MotionType.RELEASE:
            self._set_gripper(GRIPPER_OPEN, "RELEASE — opening gripper")

        elif cmd.motion_type == MotionType.WAIT:
            self._wait_until = time.monotonic() + cmd.wait_duration
            self._motion_idx += 1

        else:
            logger.warning(f"  Skipping unhandled motion type: {cmd.motion_type}")
            self._motion_idx += 1

    def _set_gripper(self, value: float, label: str):
        self._current_gripper = value
        self._publish_cartesian(self._last_xyz, duration=1.5)
        self._awaiting_for = 'gripper'
        self._motion_idx += 1
        logger.info(f"  {label}")

    def _reset_active_goal(self):
        self._active_goal = None
        self._active_motions = []
        self._motion_idx = 0

    def _publish_cartesian(self, xyz: List[float], duration: float):
        # xyz is the desired gripper-tip position; tool0 must be tcp_z above it
        payload = {
            'x':       xyz[0],
            'y':       xyz[1],
            'z':       xyz[2] + self._tcp_z,
            'roll':    EE_ROLL,
            'pitch':   EE_PITCH,
            'yaw':     EE_YAW,
            'gripper': self._current_gripper,
            'time':    duration,
        }
        msg = String()
        msg.data = json.dumps(payload)
        self._cartesian_pub.publish(msg)

    # ─── Goal expansion ──────────────────────────────────────────────────

    def _expand_goal(self, goal: ManipulationGoal) -> Optional[List[MotionCommand]]:
        ct = goal.command_type

        if ct == 'PICK':
            return self.pick_place.pick_piece(goal.row, goal.col, goal.is_king_stack)

        elif ct == 'PLACE':
            if goal.square == -1:
                return self._discard_waypoints(goal.metadata.get('discard_index', 0))
            return self.pick_place.place_piece(goal.row, goal.col, goal.is_king_stack)

        elif ct == 'BOB':
            return self.pick_place.bob_over_square(goal.row, goal.col)

        elif ct == 'FINGER_WAG':
            return self.pick_place.finger_wag()

        elif ct == 'MOVE_HOME':
            return [MotionCommand(
                motion_type=MotionType.MOVE_TO,
                position=HOME_POSITION.copy(),
                speed=PickPlace.TRANSIT_SPEED,
                label="Move to home",
            )]

        elif ct == 'GRASP':
            return [MotionCommand(motion_type=MotionType.GRASP, label="Grasp")]

        elif ct == 'RELEASE':
            return [MotionCommand(motion_type=MotionType.RELEASE, label="Release")]

        return None

    def _discard_waypoints(self, discard_index: int) -> List[MotionCommand]:
        """Waypoints to lower the held piece into the discard pile and retreat."""
        pos = self.coords.discard_position(discard_index)
        hover = pos.copy()
        hover[2] += BoardCoordinates.HOVER_HEIGHT

        return [
            MotionCommand(
                MotionType.MOVE_TO, hover,
                speed=PickPlace.TRANSIT_SPEED,
                label=f"Hover over discard [{discard_index}]",
            ),
            MotionCommand(
                MotionType.MOVE_TO, pos,
                speed=PickPlace.APPROACH_SPEED,
                label=f"Lower to discard [{discard_index}]",
            ),
            MotionCommand(MotionType.RELEASE, label="Release to discard"),
            MotionCommand(MotionType.WAIT, wait_duration=0.2, label="Wait"),
            MotionCommand(
                MotionType.MOVE_TO, hover,
                speed=PickPlace.APPROACH_SPEED,
                label="Retreat from discard",
            ),
        ]

    # ─── Completion signalling ────────────────────────────────────────────

    def _signal_done(self, goal: ManipulationGoal, success: bool, detail: str):
        result_msg = String()
        result_msg.data = ManipulationResult(
            command_id=goal.command_id,
            command_type=goal.command_type,
            success=success,
            detail=detail,
        ).to_json()
        self._result_pub.publish(result_msg)

        done_msg = Bool()
        done_msg.data = success
        self._done_pub.publish(done_msg)

        self._publish_feedback(goal, stage='done' if success else 'failed', detail=detail)
        logger.info(f"  → {goal.command_id} {'done' if success else 'FAILED'}: {detail}")

    def _publish_feedback(self, goal: ManipulationGoal, stage: str, detail: str = ""):
        msg = String()
        msg.data = ManipulationFeedback(
            command_id=goal.command_id,
            command_type=goal.command_type,
            stage=stage,
            detail=detail,
        ).to_json()
        self._feedback_pub.publish(msg)

    # ─── Lifecycle ───────────────────────────────────────────────────────

    def run(self):
        if HAS_ROS2:
            logger.info("CartesianBridgeNode spinning...")
            rclpy.spin(self.node)

    def shutdown(self):
        if HAS_ROS2:
            self.node.destroy_node()
            rclpy.shutdown()


def main():
    logging.basicConfig(level=logging.INFO)
    node = CartesianBridgeNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
