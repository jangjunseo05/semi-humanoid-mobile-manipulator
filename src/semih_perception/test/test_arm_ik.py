import math

import pytest

from semih_perception.arm_reach_node import solve_ik


# Same defaults as arm_reach_node's declared parameters / semih.urdf.xacro properties.
GEOM = dict(
    base_height=0.15,
    arm_link1_len=0.20,
    arm_link2_len=0.25,
    arm_link3_len=0.15,
    joint1_limit=3.14,
    joint23_limit=1.57,
)


def test_reachable_target_is_not_clamped():
    # (0.3, 0.0, 0.35): confirmed against a real Gazebo run on 2026-08-04 --
    # arm_reach_node computed joints=(0, 43, -90) deg and the arm physically
    # moved to that pose (see semiH_troubleshooting_log.md and session notes).
    joint1, joint2, joint3, was_clamped = solve_ik(0.3, 0.0, 0.35, **GEOM)

    assert was_clamped is False
    assert math.degrees(joint1) == pytest.approx(0.0, abs=1.0)
    assert math.degrees(joint2) == pytest.approx(43.0, abs=1.0)
    assert math.degrees(joint3) == pytest.approx(-90.0, abs=1.0)


def test_out_of_range_target_gets_clamped():
    # 2m away is far beyond the 0.40m max reach (arm_link2_len + arm_link3_len).
    _, joint2, joint3, was_clamped = solve_ik(2.0, 0.0, 0.35, **GEOM)

    assert was_clamped is True
    # A clamped target should saturate the elbow toward full extension.
    assert joint3 == pytest.approx(-GEOM['joint23_limit'], abs=0.05) or joint3 == pytest.approx(0.0, abs=0.05)


def test_yaw_points_toward_target():
    joint1, _, _, _ = solve_ik(0.3, 0.3, 0.275, **GEOM)
    assert math.degrees(joint1) == pytest.approx(45.0, abs=0.5)

    joint1, _, _, _ = solve_ik(0.3, -0.3, 0.275, **GEOM)
    assert math.degrees(joint1) == pytest.approx(-45.0, abs=0.5)


def test_joint_limits_are_respected():
    # Even a reachable-distance target placed at an extreme angle must not
    # exceed the declared joint limits.
    joint1, joint2, joint3, _ = solve_ik(0.01, 0.01, 5.0, **GEOM)

    assert abs(joint1) <= GEOM['joint1_limit'] + 1e-9
    assert abs(joint2) <= GEOM['joint23_limit'] + 1e-9
    assert abs(joint3) <= GEOM['joint23_limit'] + 1e-9
