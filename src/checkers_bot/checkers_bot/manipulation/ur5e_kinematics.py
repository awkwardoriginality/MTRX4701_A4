"""Shared UR5e kinematics helpers used by standalone simulation tools."""

from __future__ import annotations

import math
from typing import List, Tuple

import numpy as np


class UR5eKinematics:
    """Standard UR5e robot arm kinematics solver based on DH convention."""

    d = np.array([0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996])
    a = np.array([0.0, -0.425, -0.3922, 0.0, 0.0, 0.0])
    alpha = np.array([math.pi / 2, 0.0, 0.0, math.pi / 2, -math.pi / 2, 0.0])

    @classmethod
    def dh_matrix(cls, theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        """Construct a standard DH homogeneous transformation matrix."""
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        return np.array([
            [ct, -st * ca, st * sa, a * ct],
            [st, ct * ca, -ct * sa, a * st],
            [0, sa, ca, d],
            [0, 0, 0, 1],
        ])

    @classmethod
    def forward_kinematics(cls, joints: np.ndarray) -> Tuple[np.ndarray, List[np.ndarray]]:
        """Compute the flange pose and intermediate link origins."""
        origins = [np.array([0.0, 0.0, 0.0])]
        transform = np.eye(4)

        for i in range(6):
            transform = transform @ cls.dh_matrix(joints[i], cls.d[i], cls.a[i], cls.alpha[i])
            origins.append(transform[:3, 3])

        # Include a simple TCP and jaw-tip visual model for the arm simulation.
        jaw_1 = transform @ np.array([0.0, 0.03, 0.15, 1.0])
        jaw_2 = transform @ np.array([0.0, -0.03, 0.15, 1.0])
        tcp = transform @ np.array([0.0, 0.0, 0.15, 1.0])

        origins.append(tcp[:3])
        origins.append(jaw_1[:3])
        origins.append(jaw_2[:3])

        return transform, origins

    @classmethod
    def inverse_kinematics(
        cls,
        target_pos: np.ndarray,
        init_joints: np.ndarray,
        max_iter: int = 100,
        tol: float = 1e-4,
    ) -> Tuple[np.ndarray, bool]:
        """Solve a position-only IK target using a damped least-squares update."""
        safe_target = target_pos.copy()
        if safe_target[2] < 0.005:
            safe_target[2] = 0.005

        joints = np.array(init_joints, dtype=float).copy()

        for _ in range(max_iter):
            transform, _ = cls.forward_kinematics(joints)
            current_pos = (transform @ np.array([0.0, 0.0, 0.15, 1.0]))[:3]

            error = safe_target - current_pos
            if np.linalg.norm(error) < tol:
                return joints, True

            jacobian = np.zeros((3, 6))
            delta = 1e-5
            for joint_index in range(6):
                perturbed = joints.copy()
                perturbed[joint_index] += delta
                perturbed_transform, _ = cls.forward_kinematics(perturbed)
                perturbed_pos = (perturbed_transform @ np.array([0.0, 0.0, 0.15, 1.0]))[:3]
                jacobian[:, joint_index] = (perturbed_pos - current_pos) / delta

            lam = 0.01
            jacobian_pinv = jacobian.T @ np.linalg.inv(jacobian @ jacobian.T + lam ** 2 * np.eye(3))
            joints += jacobian_pinv @ error

            _, origins = cls.forward_kinematics(joints)
            for point in origins[1:]:
                if point[2] < 0.0:
                    joints[1] -= 0.01
                    joints[2] += 0.01

        return joints, False
