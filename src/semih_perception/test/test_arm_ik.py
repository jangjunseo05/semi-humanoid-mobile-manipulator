import math

import pytest

from semih_perception.arm_reach_node import solve_ik


# Same defaults as arm_reach_node's declared parameters / semih.urdf.xacro properties.
GEOM = dict(
    base_height=0.15,
    arm_link1_len=0.20,
    arm_link2_len=0.25,
    arm_link3_len=0.21,
    joint1_limit=3.14,
    joint23_limit=1.57,
)


def forward_kinematics(joint1, joint2, joint3, g=GEOM):
    """Tip position in base_link, from semih.urdf.xacro: arm_joint1 yaws about z at the
    top of the base, arm_joint2/3 pitch about +y, every link extends along its +z."""
    l2, l3 = g['arm_link2_len'], g['arm_link3_len']
    r = l2 * math.sin(joint2) + l3 * math.sin(joint2 + joint3)
    z = g['base_height'] / 2 + g['arm_link1_len'] + l2 * math.cos(joint2) + l3 * math.cos(joint2 + joint3)
    return r * math.cos(joint1), r * math.sin(joint1), z


@pytest.mark.parametrize('target', [
    (0.35, 0.0, 0.35),
    (0.30, 0.15, 0.30),
    (0.25, -0.20, 0.45),
    (0.38, 0.0, 0.22),
    (0.15, 0.30, 0.40),
])
def test_tip_reaches_target(target):
    # The previous regression-only test checked joint numbers from an earlier run and
    # never ran FK, so an elbow sign error (tip ~30cm off every target) went unnoticed.
    joint1, joint2, joint3, was_clamped = solve_ik(*target, **GEOM)
    assert was_clamped is False
    tip = forward_kinematics(joint1, joint2, joint3)
    assert math.dist(tip, target) < 0.01


def test_out_of_range_target_gets_clamped():
    # 2m away is far beyond the 0.46m max reach (arm_link2_len + arm_link3_len).
    _, joint2, joint3, was_clamped = solve_ik(2.0, 0.0, 0.35, **GEOM)

    assert was_clamped is True
    # A clamped target should saturate the elbow toward full extension.
    assert joint3 == pytest.approx(0.0, abs=0.05)


@pytest.mark.parametrize('r', [0.31, 0.33, 0.35])
def test_backpack_height_reachable_at_standoff(r):
    # Measured backpack target was base_link z=0.10 (2026-11-16 run). target_approach_node
    # stops 0.33m away (horizontal); the tip must reach that height around the standoff.
    target = (r, 0.0, 0.10)
    joint1, joint2, joint3, was_clamped = solve_ik(*target, **GEOM)
    assert was_clamped is False
    assert math.dist(forward_kinematics(joint1, joint2, joint3), target) < 0.01


def test_backpack_from_old_detection_distance_is_flagged():
    # The 2026-11-16 run reached for (0.54, 0, 0.10) directly; still beyond reach.
    *_, was_clamped = solve_ik(0.54, 0.0, 0.10, **GEOM)
    assert was_clamped is True


def test_below_joint_limit_target_is_flagged():
    # Within max reach but needs joint2 past +90 deg (elbow below the shoulder): used to
    # be clamped silently and reported as reachable.
    *_, was_clamped = solve_ik(0.25, 0.0, 0.0, **GEOM)
    assert was_clamped is True


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
