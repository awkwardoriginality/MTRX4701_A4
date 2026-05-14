"""
calibration.py — Hand-Eye Calibration module for the UR5e Checkers Bot.

Implements the Tsai-Lenz (AX=XB) hand-eye calibration algorithm to solve for the
transform between the robot base frame and the camera/checkerboard frame.

Supports:
    - Eye-to-Hand setup (Camera fixed above the workspace)
    - Eye-in-Hand setup (Camera mounted on the robot wrist)
    - Computing transformation matrices using OpenCV's calibrateHandEye
    - Updating robot_params.yaml automatically with the calibrated matrix

References:
    Tsai, R. Y., & Lenz, R. K. (1989). A new technique for fully autonomous
    and efficient 3D robotics hand/eye calibration. IEEE Transactions on
    Robotics and Automation.
"""

from __future__ import annotations
import os
import logging
import numpy as np
from typing import List, Tuple, Optional

try:
    import cv2
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

logger = logging.getLogger(__name__)


class HandEyeCalibrator:
    """Computes the hand-eye calibration transformation matrix.

    Solves AX = XB where:
        A = Robot base to end-effector relative motions
        B = Camera to calibration target relative motions
        X = Unknown transform (Camera to Base for eye-to-hand)
    """

    def __init__(self, setup_type: str = "eye_to_hand"):
        """
        Args:
            setup_type: "eye_to_hand" (fixed camera) or "eye_in_hand" (wrist mounted).
        """
        self.setup_type = setup_type
        self.R_gripper2base: List[np.ndarray] = []
        self.t_gripper2base: List[np.ndarray] = []
        self.R_target2cam: List[np.ndarray] = []
        self.t_target2cam: List[np.ndarray] = []

    def add_sample(
        self,
        T_gripper2base: np.ndarray,
        T_target2cam: np.ndarray,
    ):
        """Add a paired sample of robot pose and camera target pose.

        Args:
            T_gripper2base: 4x4 homogeneous matrix of EE in robot base frame.
            T_target2cam: 4x4 homogeneous matrix of target in camera frame.
        """
        assert T_gripper2base.shape == (4, 4) and T_target2cam.shape == (4, 4)

        self.R_gripper2base.append(T_gripper2base[:3, :3])
        self.t_gripper2base.append(T_gripper2base[:3, 3])

        self.R_target2cam.append(T_target2cam[:3, :3])
        self.t_target2cam.append(T_target2cam[:3, 3])

    def compute_calibration(self, method: int = getattr(cv2, 'CALIB_HAND_EYE_TSAI', 0)) -> Optional[np.ndarray]:
        """Solve for the unknown 4x4 transformation matrix.

        Returns:
            T_cam2base (for Eye-to-Hand) or T_cam2gripper (for Eye-in-Hand).
        """
        if not HAS_CV2 or len(self.R_gripper2base) < 3:
            logger.warning("Insufficient samples or OpenCV unavailable for calibration")
            return None

        # Convert lists to NumPy arrays expected by OpenCV
        R_g2b = np.array(self.R_gripper2base)
        t_g2b = np.array(self.t_gripper2base)
        R_t2c = np.array(self.R_target2cam)
        t_t2c = np.array(self.t_target2cam)

        if self.setup_type == "eye_to_hand":
            # For Eye-to-Hand, calibrateHandEye expects poses of base in gripper frame
            # T_base2gripper = inv(T_gripper2base)
            R_b2g = []
            t_b2g = []
            for r, t in zip(self.R_gripper2base, self.t_gripper2base):
                r_inv = r.T
                t_inv = -r_inv @ t
                R_b2g.append(r_inv)
                t_b2g.append(t_inv)

            R_robot = np.array(R_b2g)
            t_robot = np.array(t_b2g)
        else:
            R_robot = R_g2b
            t_robot = t_g2b

        try:
            R_cam2base, t_cam2base = cv2.calibrateHandEye(
                R_robot, t_robot, R_t2c, t_t2c, method=method
            )

            T_result = np.eye(4)
            T_result[:3, :3] = R_cam2base
            T_result[:3, 3] = t_cam2base.flatten()
            return T_result

        except Exception as e:
            logger.error(f"Hand-eye calibration failed: {e}")
            return None

    @staticmethod
    def update_yaml_config(yaml_path: str, T_matrix: np.ndarray, param_name: str = "board_to_robot_tf"):
        """Update the configuration YAML file with the calibrated matrix."""
        assert T_matrix.shape == (4, 4)

        if not os.path.exists(yaml_path):
            logger.error(f"Config YAML not found: {yaml_path}")
            return

        with open(yaml_path, 'r') as f:
            lines = f.readlines()

        out_lines = []
        skip_matrix = False
        matrix_written = False

        for line in lines:
            if skip_matrix:
                if line.strip().startswith("-") or (line.startswith(" ") and "[" in line):
                    continue
                else:
                    skip_matrix = False

            if line.strip().startswith(f"{param_name}:"):
                out_lines.append(line)
                # Write the 4 rows formatted neatly
                for row in T_matrix:
                    row_str = ", ".join([f"{val:10.5f}" for val in row])
                    out_lines.append(f"    - [{row_str}]\n")
                skip_matrix = True
                matrix_written = True
            else:
                out_lines.append(line)

        if matrix_written:
            with open(yaml_path, 'w') as f:
                f.writelines(out_lines)
            logger.info(f"Updated {yaml_path} successfully with calibrated matrix.")
