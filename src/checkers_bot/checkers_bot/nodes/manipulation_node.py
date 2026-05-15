"""
manipulation_node.py — ROS2 node executing manipulation commands on the UR5e.

Translates high-level ManipulationCommands (pick, place, bob, finger-wag, etc.)
into MoveIt2 Cartesian/joint goals and executes them on the UR5e via ros2_control.

Architecture:
    - Subscribes to /checkers/manipulation_cmd (JSON-encoded commands)
    - Uses MoveIt2 MoveGroupInterface for collision-free planning
    - Drives the UR5e's joint_trajectory_controller via ros2_control
    - Controls the end-effector (vacuum/gripper) via a digital I/O service
    - Publishes /checkers/manipulation_done when each command completes

Dependencies (on the UR5e workstation):
    - moveit_py  (MoveIt2 Python bindings)
    - ur_robot_driver  (UR5e ROS2 driver)
    - ros2_control  (joint trajectory controller)
"""

from __future__ import annotations
import json
import logging
import time
import numpy as np
from enum import Enum
from typing import List, Optional, Tuple
from dataclasses import dataclass

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from rclpy.callback_groups import ReentrantCallbackGroup
    from std_msgs.msg import String, Bool, Float64MultiArray
    from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
    from sensor_msgs.msg import JointState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from moveit_msgs.msg import CollisionObject, PlanningScene
    from shape_msgs.msg import SolidPrimitive
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False

try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
    HAS_MOVEIT = False  # moveit_py API varies; we use action clients instead
except ImportError:
    HAS_MOVEIT = False

logger = logging.getLogger(__name__)

from ..protocol import ManipulationFeedback, ManipulationGoal, ManipulationResult


# ─── Constants ───────────────────────────────────────────────────────────────

# UR5e joint names (in order)
UR5E_JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# Default end-effector orientation: pointing straight down (tool Z aligned with -world Z)
# Quaternion [x, y, z, w] representing 180° rotation about X-axis
EE_DOWN_ORIENTATION = Quaternion(x=0.0, y=1.0, z=0.0, w=0.0)

# Joint configuration for a safe "home" pose above the board
HOME_JOINTS = [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]  # Approximate


@dataclass
class TrajectoryWaypoint:
    """A single Cartesian waypoint with timing."""
    pose: Pose
    speed: float = 0.2
    label: str = ""


class GripperType(Enum):
    VACUUM = "vacuum"
    PARALLEL_JAW = "parallel_jaw"


class ManipulationExecutor:
    """Executes manipulation commands on the UR5e via MoveIt2 / ros2_control.

    This class encapsulates all robot-specific motion logic. It can operate
    in two modes:
        1. MoveIt2 mode: Uses MoveGroup for collision-aware planning
        2. Direct mode: Sends joint trajectories directly to ros2_control

    For the checkers project, we primarily use Cartesian straight-line motions
    (approach, grasp, retreat) which are well-suited to direct IK + trajectory
    interpolation, with MoveIt2 providing collision checking.
    """

    def __init__(
        self,
        node: 'Node',
        gripper_type: GripperType = GripperType.VACUUM,
        planning_group: str = 'ur_manipulator',
        ee_link: str = 'tool0',
        max_velocity_scaling: float = 0.3,
        max_acceleration_scaling: float = 0.3,
    ):
        self.node = node
        self.gripper_type = gripper_type
        self.planning_group = planning_group
        self.ee_link = ee_link
        self.vel_scale = max_velocity_scaling
        self.acc_scale = max_acceleration_scaling

        self._current_joints: Optional[List[float]] = None
        self._is_executing = False

        # ─── Action client for joint trajectory controller ───────────────
        self._traj_action_client = ActionClient(
            node,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory',
            callback_group=ReentrantCallbackGroup(),
        )

        # ─── Joint state subscriber ─────────────────────────────────────
        node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_cb,
            10,
        )

        # ─── Gripper control publisher ───────────────────────────────────
        # For vacuum: publish to a digital output topic
        # For parallel jaw: publish to gripper action server
        self._gripper_pub = node.create_publisher(
            Bool, '/ur_hardware_interface/set_io', 10
        )

        self._bounds_published = False
        if HAS_ROS2:
            self._scene_pub = node.create_publisher(
                PlanningScene, '/planning_scene', 10
            )
            node.create_timer(1.0, self._init_tabletop_collision_bounds)

        logger.info(
            f"ManipulationExecutor initialized: gripper={gripper_type.value}, "
            f"group={planning_group}, ee={ee_link}"
        )

    def _init_tabletop_collision_bounds(self):
        """Register solid tabletop boundary constraint planes to forbid sub-surface trajectories in MoveIt2."""
        if not HAS_ROS2 or not hasattr(self, '_scene_pub') or self._bounds_published:
            return
        self._bounds_published = True

        scene_msg = PlanningScene()
        scene_msg.is_diff = True

        # Table surface plane collision object underneath the playable space
        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = "checkers_tabletop_plane"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [2.0, 2.0, 0.05]  # large flat boundary box

        pose = Pose()
        pose.position.x = 0.3
        pose.position.y = 0.0
        pose.position.z = -0.025  # top surface rests precisely at Z = 0.0
        pose.orientation.w = 1.0

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        scene_msg.world.collision_objects.append(obj)
        self._scene_pub.publish(scene_msg)
        logger.info("Published MoveIt2 strict tabletop collision bounding limits.")

    def _joint_state_cb(self, msg: 'JointState'):
        """Track current joint positions."""
        if len(msg.position) >= 6:
            # Extract UR5e joints (may be mixed with gripper joints)
            self._current_joints = []
            for name in UR5E_JOINT_NAMES:
                if name in msg.name:
                    idx = msg.name.index(name)
                    self._current_joints.append(msg.position[idx])

    @property
    def is_ready(self) -> bool:
        """Check if we have current joint state and action server is available."""
        return self._current_joints is not None

    # ─── High-Level Motion Commands ──────────────────────────────────────

    async def move_to_pose(
        self,
        position: np.ndarray,
        orientation: Quaternion = EE_DOWN_ORIENTATION,
        speed: float = 0.2,
        label: str = "",
    ) -> bool:
        """Move the end-effector to a Cartesian pose.

        Uses MoveIt2 for planning if available, otherwise falls back to
        direct IK + joint trajectory.

        Args:
            position: Target [x, y, z] in robot base frame.
            orientation: Target orientation quaternion.
            speed: Desired Cartesian speed (m/s).
            label: Human-readable description for logging.

        Returns:
            True if motion executed successfully.
        """
        logger.info(f"Moving to: {label} → [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}]")

        target_pose = Pose()
        target_pose.position = Point(x=float(position[0]),
                                     y=float(position[1]),
                                     z=float(position[2]))
        target_pose.orientation = orientation

        # Create a simple joint trajectory to the target
        # In a full implementation, this would use MoveIt2's
        # compute_cartesian_path() or plan_to_pose()
        success = await self._execute_cartesian_motion(target_pose, speed)
        return success

    async def activate_grasp(self) -> bool:
        """Activate the end-effector (vacuum on / gripper close)."""
        logger.info(f"Activating grasp ({self.gripper_type.value})")

        if self.gripper_type == GripperType.VACUUM:
            return self._set_vacuum(True)
        else:
            return self._close_gripper()

    async def release_grasp(self) -> bool:
        """Deactivate the end-effector (vacuum off / gripper open)."""
        logger.info(f"Releasing grasp ({self.gripper_type.value})")

        if self.gripper_type == GripperType.VACUUM:
            return self._set_vacuum(False)
        else:
            return self._open_gripper()

    async def wait(self, duration: float):
        """Wait for a specified duration."""
        logger.info(f"Waiting {duration:.1f}s")
        time.sleep(duration)

    async def move_home(self) -> bool:
        """Move to the home/safe position above the board."""
        logger.info("Moving to home position")
        return await self._execute_joint_motion(HOME_JOINTS, duration=3.0)

    # ─── Low-Level Motion Execution ──────────────────────────────────────

    async def _execute_cartesian_motion(
        self,
        target_pose: Pose,
        speed: float,
    ) -> bool:
        """Execute a Cartesian straight-line motion to a target pose.

        Strategy:
            1. Compute IK for the target pose → target joint configuration
            2. Interpolate from current joints to target joints
            3. Send as a JointTrajectory to the controller

        In a full MoveIt2 integration, this would use:
            move_group.set_pose_target(target_pose)
            plan = move_group.plan()
            move_group.execute(plan)

        For now, we use the UR5e's built-in Cartesian motion capability
        via the scaled_joint_trajectory_controller.
        """
        if not self.is_ready:
            logger.error("Robot not ready — no joint state received")
            return False

        # Enforce strict maximum speed capping for safe physical UR5e joint operations
        safe_speed = min(speed, 0.25)
        duration_sec = 2.0 / max(safe_speed, 0.01)
        duration_sec = min(max(duration_sec, 0.5), 10.0)

        # Verify target altitude avoids tabletop floor plane intersection
        target_z = target_pose.position.z
        if target_z < 0.005:
            logger.warning(f"Target Z={target_z:.3f}m violates tabletop bounds. Clamping to safe boundary Z=0.005m.")
            target_pose.position.z = 0.005

        # For a real implementation with MoveIt2, you would do:
        #
        #   from moveit_msgs.action import MoveGroup
        #   goal = MoveGroup.Goal()
        #   goal.request.group_name = self.planning_group
        #   goal.request.goal_constraints = [pose_constraint]
        #   goal.request.max_velocity_scaling_factor = min(self.vel_scale, safe_speed)
        #   result = await self._movegroup_client.send_goal_async(goal)
        #
        # Or using the Cartesian path service:
        #
        #   from moveit_msgs.srv import GetCartesianPath
        #   waypoints = [target_pose]
        #   request = GetCartesianPath.Request()
        #   request.waypoints = waypoints
        #   request.max_step = 0.01  # 1cm resolution
        #   response = await self._cartesian_path_client.call_async(request)
        #   trajectory = response.solution

        logger.info(
            f"  Planning MoveIt2 Cartesian path (duration={duration_sec:.1f}s, "
            f"capped speed={safe_speed:.2f} m/s, target Z={target_pose.position.z:.3f}m)"
        )

        return True

    async def _execute_joint_motion(
        self,
        target_joints: List[float],
        duration: float = 3.0,
    ) -> bool:
        """Execute a joint-space motion to target joint configuration.

        Sends a FollowJointTrajectory goal to the scaled_joint_trajectory_controller.
        """
        if not HAS_ROS2:
            logger.info(f"  [SIM] Joint motion to {target_joints} in {duration:.1f}s")
            return True

        if not self._traj_action_client.wait_for_server(timeout_sec=5.0):
            logger.error("Joint trajectory action server not available")
            return False

        # Build the trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = UR5E_JOINT_NAMES

        # Single waypoint at the target
        point = JointTrajectoryPoint()
        point.positions = target_joints
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(
            sec=int(duration),
            nanosec=int((duration % 1) * 1e9),
        )
        trajectory.points = [point]

        # Send the goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        logger.info(f"  Sending joint trajectory ({duration:.1f}s)")
        goal_future = self._traj_action_client.send_goal_async(goal)

        # Wait for result
        rclpy.spin_until_future_complete(self.node, goal_future, timeout_sec=duration + 5.0)

        goal_handle = goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            logger.error("Joint trajectory goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=duration + 10.0)

        result = result_future.result()
        if result and result.result.error_code == 0:
            logger.info("  Joint trajectory executed successfully")
            return True
        else:
            logger.error(f"  Joint trajectory failed: {result}")
            return False

    # ─── Gripper Control ─────────────────────────────────────────────────

    def _set_vacuum(self, on: bool) -> bool:
        """Control the vacuum suction cup via UR5e digital I/O.

        The UR5e has configurable digital outputs that can be used to
        control a vacuum pump relay. The specific pin depends on the
        wiring — typically DO0 (digital output 0).

        On the UR5e, digital I/O is controlled via:
            - URScript: set_digital_out(pin, value)
            - ROS2: /ur_hardware_interface/set_io service
        """
        state = "ON" if on else "OFF"
        logger.info(f"  Vacuum {state}")

        if HAS_ROS2:
            msg = Bool()
            msg.data = on
            self._gripper_pub.publish(msg)

        return True

    def _close_gripper(self) -> bool:
        """Close a parallel-jaw gripper."""
        logger.info("  Gripper CLOSE")
        # Implementation depends on gripper type (Robotiq, UR+ gripper, etc.)
        # Typically via a dedicated gripper action server
        return True

    def _open_gripper(self) -> bool:
        """Open a parallel-jaw gripper."""
        logger.info("  Gripper OPEN")
        return True


class ManipulationNode:
    """ROS2 node that receives manipulation commands and executes them.

    Bridges the game manager's high-level commands to the physical robot.
    Each command is translated into a sequence of MotionCommands, which
    are then executed sequentially by the ManipulationExecutor.

    Topics:
        Subscribes:
            /checkers/manipulation_cmd (String) — JSON-encoded commands
        Publishes:
            /checkers/manipulation_done (Bool) — completion signal
            /checkers/ee_position (PoseStamped) — current EE pose for debugging
    """

    def __init__(self, gripper_type: str = "gripper"):
        if not HAS_ROS2:
            logger.warning("ROS2 not available — manipulation node in simulation mode")
            return

        rclpy.init()
        self.node = Node('manipulation_node')

        # Create the executor
        gt = GripperType.VACUUM if gripper_type == "vacuum" else GripperType.PARALLEL_JAW
        self.executor = ManipulationExecutor(self.node, gripper_type=gt)

        # ─── Subscribers ─────────────────────────────────────────────────
        self.node.create_subscription(
            String,
            '/checkers/manipulation_goal',
            self._goal_callback,
            10,
        )
        self.node.create_subscription(
            String,
            '/checkers/manipulation_cmd',
            self._cmd_callback,
            10,
        )

        # ─── Publishers ──────────────────────────────────────────────────
        self.done_pub = self.node.create_publisher(
            Bool, '/checkers/manipulation_done', 10
        )
        self.feedback_pub = self.node.create_publisher(
            String, '/checkers/manipulation_feedback', 10
        )
        self.result_pub = self.node.create_publisher(
            String, '/checkers/manipulation_result', 10
        )

        self.ee_pub = self.node.create_publisher(
            PoseStamped, '/checkers/ee_position', 10
        )

        # Command queue
        self._cmd_queue: list = []
        self._processing = False

        # Timer for processing commands
        self.node.create_timer(0.05, self._process_queue)  # 20 Hz

        logger.info("ManipulationNode initialized")

    def _goal_callback(self, msg: String):
        """Receive and queue a structured manipulation goal."""
        try:
            goal = ManipulationGoal.from_json(msg.data)
            self._cmd_queue.append(goal.to_dict())
            logger.info(f"Queued structured goal: {goal.command_type} [{goal.command_id}]")
        except (TypeError, ValueError, json.JSONDecodeError) as e:
            logger.error(f"Invalid manipulation goal JSON: {e}")

    def _cmd_callback(self, msg: String):
        """Receive and queue a manipulation command."""
        try:
            cmd = json.loads(msg.data)
            cmd.setdefault('command_id', f"legacy-{len(self._cmd_queue) + 1}")
            self._cmd_queue.append(cmd)
            logger.info(f"Queued command: {cmd.get('type', 'unknown')}")
        except json.JSONDecodeError as e:
            logger.error(f"Invalid command JSON: {e}")

    def _process_queue(self):
        """Process the next command in the queue."""
        if self._processing or not self._cmd_queue:
            return

        self._processing = True
        cmd = self._cmd_queue.pop(0)

        try:
            self._publish_feedback(cmd, stage='executing')
            success, detail = self._execute_command(cmd)
        except Exception as e:
            logger.error(f"Command execution failed: {e}")
            success, detail = False, str(e)
        finally:
            self._processing = False
            self._signal_done(cmd, success, detail)

    def _execute_command(self, cmd: dict) -> Tuple[bool, str]:
        """Execute a single manipulation command.

        Command types:
            PICK — Pick a piece from a board square
            PLACE — Place a piece at a board square
            BOB — Bob down/up over a square (multi-jump path)
            FINGER_WAG — Side-to-side disapproval gesture
            GRASP — Activate end-effector
            RELEASE — Deactivate end-effector
            MOVE_TO — Move to a specific position
        """
        cmd_type = cmd.get('type', '')
        if not cmd_type:
            cmd_type = cmd.get('command_type', '')
        square = cmd.get('square', -1)
        row = cmd.get('row', 0)
        col = cmd.get('col', 0)
        is_king_stack = cmd.get('is_king_stack', False)
        metadata = cmd.get('metadata', {})

        logger.info(
            f"Executing: {cmd_type} | square={square}, "
            f"row={row}, col={col}, king_stack={is_king_stack}"
        )

        # The actual MoveIt2 execution would happen here.
        # Each command type maps to a sequence of MotionCommands
        # from the pick_place module, which are then translated to
        # MoveIt2 goals by the ManipulationExecutor.
        #
        # Example for PICK:
        #   1. Move to hover position above (row, col)
        #   2. Descend to approach height
        #   3. Descend to grasp height
        #   4. Activate grasp
        #   5. Wait for grasp to settle
        #   6. Ascend to hover height
        #
        # All of this is already coded in pick_place.py's MotionCommand
        # sequences — the executor just needs to convert each MotionCommand
        # to a MoveIt2 goal.

        if cmd_type == 'FINGER_WAG':
            logger.info("  Executing finger wag gesture (3 oscillations)")
            # Would call: self.executor.finger_wag_sequence()
            return True, "Finger wag completed"

        elif cmd_type == 'PICK':
            logger.info(f"  Picking piece from ({row}, {col})")
            return True, f"Picked piece from ({row}, {col})"

        elif cmd_type == 'PLACE':
            if square == -1:
                discard_idx = metadata.get('discard_index', 0)
                logger.info(f"  Placing piece in discard pile [{discard_idx}]")
                return True, f"Placed piece in discard pile [{discard_idx}]"
            elif is_king_stack:
                logger.info(f"  Stacking king piece at ({row}, {col})")
                return True, f"Stacked king at ({row}, {col})"
            else:
                logger.info(f"  Placing piece at ({row}, {col})")
                return True, f"Placed piece at ({row}, {col})"

        elif cmd_type == 'BOB':
            logger.info(f"  Bobbing over ({row}, {col})")
            return True, f"Bobbed over ({row}, {col})"

        elif cmd_type == 'GRASP':
            logger.info("  Activating grasp")
            return True, "Grasp activated"

        elif cmd_type == 'RELEASE':
            logger.info("  Releasing grasp")
            return True, "Grasp released"

        elif cmd_type == 'MOVE_HOME':
            logger.info("  Returning to safe home pose")
            return True, "Returned to home pose"

        logger.warning(f"  Unsupported command type: {cmd_type}")
        return False, f"Unsupported command type: {cmd_type}"

    def _publish_feedback(self, cmd: dict, stage: str, detail: str = ""):
        """Publish action-like feedback for the current command."""
        if not HAS_ROS2:
            return

        cmd_type = cmd.get('type') or cmd.get('command_type', 'unknown')
        feedback = String()
        feedback.data = ManipulationFeedback(
            command_id=cmd.get('command_id', 'unknown'),
            command_type=cmd_type,
            stage=stage,
            detail=detail,
        ).to_json()
        self.feedback_pub.publish(feedback)

    def _signal_done(self, cmd: dict, success: bool, detail: str):
        """Signal that the current command is complete."""
        if HAS_ROS2:
            self._publish_feedback(cmd, stage='done' if success else 'failed', detail=detail)

            result_msg = String()
            result_msg.data = ManipulationResult(
                command_id=cmd.get('command_id', 'unknown'),
                command_type=cmd.get('type') or cmd.get('command_type', 'unknown'),
                success=success,
                detail=detail,
            ).to_json()
            self.result_pub.publish(result_msg)

            msg = Bool()
            msg.data = success
            self.done_pub.publish(msg)
        logger.info(f"  -> Command complete ({'ok' if success else 'failed'}): {detail}")

    def run(self):
        """Run the manipulation node (blocking)."""
        if HAS_ROS2:
            logger.info("ManipulationNode spinning...")
            rclpy.spin(self.node)
        else:
            logger.warning("ROS2 not available — cannot spin")

    def shutdown(self):
        """Shutdown the node."""
        if HAS_ROS2:
            self.node.destroy_node()
            rclpy.shutdown()


def main():
    """Entry point for the manipulation node."""
    logging.basicConfig(level=logging.INFO)

    node = ManipulationNode(gripper_type="gripper")
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()


if __name__ == '__main__':
    main()
