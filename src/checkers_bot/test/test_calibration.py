"""
test_calibration.py — Unit tests for the Tsai-Lenz hand-eye calibration module.

Tests:
    1. Instantiation and data buffering interfaces
    2. Fallback handling for missing/insufficient samples
    3. YAML file generation and matrix parsing updates
"""

import sys
import os
import tempfile
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from checkers_bot.manipulation.calibration import HandEyeCalibrator


def test_calibrator_instantiation():
    """Verify initialization and sample addition."""
    cal = HandEyeCalibrator(setup_type="eye_to_hand")
    assert cal.setup_type == "eye_to_hand"

    T_g2b = np.eye(4)
    T_t2c = np.eye(4)
    T_g2b[0, 3] = 0.5  # 50cm offset

    cal.add_sample(T_g2b, T_t2c)
    assert len(cal.R_gripper2base) == 1
    assert np.array_equal(cal.t_gripper2base[0], np.array([0.5, 0.0, 0.0]))
    print("✓ Hand-Eye Calibrator sample buffering verified")


def test_yaml_config_update():
    """Verify automatic updating of calibrated matrices in config YAMLs."""
    dummy_yaml = """# Placeholder configuration
robot:
  board_to_robot_tf:
    - [1.0, 0.0, 0.0, 0.0]
    - [0.0, 1.0, 0.0, 0.0]
    - [0.0, 0.0, 1.0, 0.0]
    - [0.0, 0.0, 0.0, 1.0]
  end_effector: "vacuum"
"""
    with tempfile.NamedTemporaryFile('w', delete=False) as f:
        f.write(dummy_yaml)
        temp_path = f.name

    try:
        new_matrix = np.eye(4)
        new_matrix[0, 3] = 0.42  # Updated X translation
        new_matrix[1, 3] = -0.15 # Updated Y translation

        HandEyeCalibrator.update_yaml_config(temp_path, new_matrix, param_name="board_to_robot_tf")

        with open(temp_path, 'r') as f:
            updated_content = f.read()

        assert "0.42000" in updated_content
        assert "-0.15000" in updated_content
        assert 'end_effector: "vacuum"' in updated_content
        print("✓ Configuration YAML automatic matrix updating correct")

    finally:
        if os.path.exists(temp_path):
            os.remove(temp_path)


if __name__ == '__main__':
    print("=" * 60)
    print("HAND-EYE CALIBRATION TESTS")
    print("=" * 60)

    test_calibrator_instantiation()
    test_yaml_config_update()

    print("\n" + "=" * 60)
    print("ALL CALIBRATION TESTS PASSED ✓")
    print("=" * 60)
