"""Gravity feed-forward torques for the B601-RS (Seeed reBot Arm RS) arm.

``compensation_torques(q_raw)`` takes the six arm joint positions in raw
Motorbridge motor radians (``robstride_get_param_f32(0x7019)``, the same value
``send_mit`` takes as its position) and returns the six MIT feed-forward
torques in N*m, raw motor frame, to pass as the ``tau`` of
``send_mit(pos, vel, kp, kd, tau)``. The gripper motor is not modelled as a
joint: send it 0. Its fingers (2 x 0.075 kg) are carried as a load at the
URDF zero opening. They slide symmetrically in opposite directions, so their
travel barely moves the load's centre of mass.

This is a static rigid-body model, g(q) of the URDF below, with a gravity
vector of -9.81 m/s^2 along base z. That equals Pinocchio's
``computeGeneralizedGravity`` that Seeed's controller uses. It assumes the base
is mounted level and the arm carries no payload. The PD loop must stay in
command: this feed-forward covers neither friction (0.2-0.5 N*m per joint,
measured by Seeed) nor contact.

Frames. On the RS build the raw motor frame IS the URDF joint frame: same
zeros (folded rest pose = all zeros), same positive directions, motor torque
positive about the URDF joint axis. Seeed's stack applies no sign or offset
anywhere: ``RebotArm.get_positions`` returns mechPos/state unchanged, and
``GravityCompensation`` sends ``g(q)`` as MIT tau with ``joint_direction``
defaulting to 1. Their calibration (``docs/gravity_calibration_rs_2026-07-17.md``)
states it explicitly and fits a positive scale near 1 on every joint whose
gravity signal clears friction. The LeRobot "native" frame (raw = -native) is
a different frame. Do not pass native angles here.

The kinematic and inertial numbers are vendored, so the installed wheel needs
neither the URDF nor numpy, from Seeed's reBotArm_control_py
(https://github.com/Seeed-Projects/reBotArm_control_py) at commit 6415d43:
    urdf/RS/urdf/ReBot_Arm_RS.urdf
    sha256 3838ca8f9d586be22a89b33c7cbdd4abd43d4a92a2de377b294eef4c5bd7654d
Byte-identical to Rebot_Arm_description/RS/urdf/ReBot_Arm_RS.urdf in
https://github.com/Seeed-Projects/reBot-DevArm. tests/test_gravity.py rechecks
every number against a checkout named by $REBOTARM_CONTROL_PY.
"""

from __future__ import annotations

import math
from collections.abc import Iterable

Vec = tuple[float, ...]  # a 3-vector
Mat = tuple[Vec, ...]  # a 3x3 rotation, row-major

URDF_SOURCE = "urdf/RS/urdf/ReBot_Arm_RS.urdf"  # within reBotArm_control_py 6415d43
URDF_SHA256 = "3838ca8f9d586be22a89b33c7cbdd4abd43d4a92a2de377b294eef4c5bd7654d"
GRAVITY = 9.81  # m/s^2, Seeed's EARTH_GRAVITY = (0, 0, -9.81)

# Per joint: name, motor, sign, scale, clamp.
#   q_urdf = sign * q_raw          (no zero offset)
#   tau_raw = clamp(sign * scale * g_urdf(q_urdf), +-clamp)
# Fits below are of Seeed's contact-free PD sweeps
# (tools/gravity_calibration/data/pdsweep_*.json, tau_est = kp*err - kd*v),
# made with this model as tau_est = k*g + fric*dir, with offset and bias held
# at 0, as applied here.
# sign: +1 on every joint. On joints 2-4, a flipped sign fits 2-8x worse.
#   Seeed's joint5 k = -0.805 is not a sign flip. Joint5's axis is within 6
#   degrees of vertical in that sweep pose, so the model's g5 there is under
#   0.075 N*m, against 0.21 N*m friction, and a flipped sign fits identically.
#   The sweep carries no information on joint5.
# scale: joint2 0.98 (fit 0.980, = Seeed's shipped tau_scale). Joint3 0.95
#   (fit 0.945; Seeed's clean fit 0.952). Joint4 0.98 (fit 0.98-1.01).
#   Seeed's 0.948 / 0.885 for joints 2 / 4 come with a fitted offset and bias
#   that this module does not apply. Joint4's +0.26 rad offset is not
#   identifiable: rms is flat (0.140-0.159 N*m) for offsets from -0.3 to
#   +0.3 rad, while k moves from 1.36 to 0.84. Seeed's README retracts its
#   first-pass zero offsets as contact artifacts. Joint1 (vertical axis,
#   g = 0 on a level base), joint5 (no usable data) and joint6 (load centred
#   on its axis, |g| < 0.01 N*m) keep the model's 1.0.
# clamp: a guard against model or sign errors: 12 N*m on rs-06, 4 N*m on
#   rs-00. joint2's model torque reaches 15 N*m when the arm is stretched
#   out, so there the clamp binds and the PD loop carries the rest.
JOINT_TABLE = (
    ("joint1", "rs-06", 1, 1.00, 12.0),
    ("joint2", "rs-06", 1, 0.98, 12.0),
    ("joint3", "rs-06", 1, 0.95, 12.0),
    ("joint4", "rs-00", 1, 0.98, 4.0),
    ("joint5", "rs-00", 1, 1.00, 4.0),
    ("joint6", "rs-00", 1, 1.00, 4.0),
)

# Vendored URDF data. Joints: origin xyz (m), origin rpy (rad), axis.
_ARM_JOINTS = (
    (
        (-0.00034283, -0.00098683, 0.075),
        (0.0, 0.0, 0.0),
        (0.0, 0.0, -1.0),
    ),  # joint1 base_link->link1
    ((0.020343, 0.027237, 0.07), (-1.5708, 0.0, 0.0), (0.0, 0.0, 1.0)),  # joint2 link1->link2
    ((-0.236, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0)),  # joint3 link2->link3
    ((0.228, -0.072746, 0.0045), (0.0, 0.0, 0.0), (0.0, 0.0, -1.0)),  # joint4 link3->link4
    ((0.087, -0.048, -0.03075), (-1.5708, 0.0, 0.0), (0.0, 0.0, -1.0)),  # joint5 link4->link5
    ((0.0365, 0.0, 0.048), (0.0, 1.5708, 0.0), (0.0, 0.0, -1.0)),  # joint6 link5->link6
)
# Links: mass (kg), centre of mass xyz (m) in the link frame. All inertial rpy
# are 0. base_link (1.1774 kg) is fixed and loads no joint.
_LINK1 = (0.4655, (0.00817652280320541, 0.00122584211922395, 0.0331419658967293))
_LINK2 = (1.552, (-0.117864812717456, -0.00342226413942967, -0.0271551938501484))
_LINK3 = (1.252, (0.103427154487799, -0.0664137912104635, -0.0271068687716704))
_LINK4 = (0.46, (0.0475544540775559, -0.0441113470158517, -0.0314111924918381))
_LINK5 = (0.20120457182895, (0.0114400622356175, 4.7907739316217e-06, 0.0380487481818926))
_LINK6 = (0.1, (1.80196022567525e-05, 2.48334592051007e-08, 0.00434805764057547))
_GRIPPER_END = (0.65, (-0.112751692515958, 0.000128443070436966, -5.72602449749882e-06))
_GRIPPER_LEFT = (0.0752, (-0.00218008703135955, 0.0187090355934181, 0.00932188568692145))
_GRIPPER_RIGHT = (0.0752, (0.00218008703135955, -0.018709035593418, 0.00932188568692151))
# Fixed j_gripper_end (link6 -> gripper_end), and the prismatic finger joints
# gripper_joint1/2 (gripper_end -> gripper_left/right) at q = 0.
_J_GRIPPER_END = ((0.0, 0.0, 0.16621), (3.1416, -1.5708, 0.0))
_GRIPPER_JOINT1 = ((-0.041939, -7.3385e-05, 0.0), (1.5708, -1.5708, 0.0))
_GRIPPER_JOINT2 = ((-0.041939, 7.3385e-05, 0.0), (-1.5708, -1.5708, 0.0))


def _matmul(a: Mat, b: Mat) -> Mat:
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)) for i in range(3)
    )


def _apply(r: Mat, v: Vec) -> Vec:
    return tuple(r[i][0] * v[0] + r[i][1] * v[1] + r[i][2] * v[2] for i in range(3))


def _add(a: Vec, b: Vec) -> Vec:
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def _rpy(roll: float, pitch: float, yaw: float) -> Mat:
    """URDF fixed-axis rpy: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _axis_angle(axis: Vec, angle: float) -> Mat:
    x, y, z = axis
    c, s = math.cos(angle), math.sin(angle)
    t = 1.0 - c
    return (
        (t * x * x + c, t * x * y - s * z, t * x * z + s * y),
        (t * x * y + s * z, t * y * y + c, t * y * z - s * x),
        (t * x * z - s * y, t * y * z + s * x, t * z * z + c),
    )


def _carried_by_link6() -> tuple[float, Vec]:
    """Lump link6, gripper_end and both fingers into one link6-frame body."""
    end_r = _rpy(*_J_GRIPPER_END[1])
    end_p = _J_GRIPPER_END[0]
    bodies: list[tuple[float, Vec]] = [
        _LINK6,
        (_GRIPPER_END[0], _add(end_p, _apply(end_r, _GRIPPER_END[1]))),
    ]
    for (xyz, rpy), (mass, com) in (
        (_GRIPPER_JOINT1, _GRIPPER_LEFT),
        (_GRIPPER_JOINT2, _GRIPPER_RIGHT),
    ):
        r = _matmul(end_r, _rpy(*rpy))
        p = _add(end_p, _apply(end_r, xyz))
        bodies.append((mass, _add(p, _apply(r, com))))
    mass = sum(m for m, _ in bodies)
    return mass, tuple(sum(m * c[i] for m, c in bodies) / mass for i in range(3))


# Per arm joint: origin xyz, origin rotation, axis, and the (mass, com) of the link it moves.
_CHAIN: tuple[tuple[Vec, Mat, Vec, tuple[float, Vec]], ...] = tuple(
    (xyz, _rpy(*rpy), axis, body)
    for (xyz, rpy, axis), body in zip(
        _ARM_JOINTS, (_LINK1, _LINK2, _LINK3, _LINK4, _LINK5, _carried_by_link6()), strict=False
    )
)


def _six_numbers(values: Iterable[float], label: str) -> tuple[float, ...]:
    if isinstance(values, (str, bytes)):
        raise ValueError(f"{label} must be 6 numbers")
    try:
        values = tuple(values)
    except TypeError:
        raise ValueError(f"{label} must be 6 numbers") from None
    if len(values) != 6:
        raise ValueError(f"{label} must be 6 numbers, got {len(values)}")
    out = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{label} must be 6 numbers")
        value = float(value)
        if not math.isfinite(value):
            raise ValueError(f"{label} must be finite")
        out.append(value)
    return tuple(out)


def urdf_gravity(q_urdf: Iterable[float]) -> tuple[float, ...]:
    """Return g(q) in the URDF frame: the joint torques that hold the arm still.

    Unscaled and unclamped, this equals Pinocchio's computeGeneralizedGravity
    on the vendored URDF with the fingers at 0. It is the negative of the
    gravity torque about each joint axis.
    """
    angles = _six_numbers(q_urdf, "q_urdf")
    rotation: Mat = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    position: Vec = (0.0, 0.0, 0.0)
    frames: list[tuple[Vec, Vec, float, Vec]] = []
    for (xyz, origin, axis, (mass, com)), angle in zip(_CHAIN, angles, strict=False):
        position = _add(position, _apply(rotation, xyz))
        rotation = _matmul(rotation, origin)
        axis_world = _apply(rotation, axis)
        rotation = _matmul(rotation, _axis_angle(axis, angle))
        frames.append((position, axis_world, mass, _add(position, _apply(rotation, com))))
    torques = [0.0] * 6
    total_mass = 0.0
    moment_sum: Vec = (0.0, 0.0, 0.0)  # sums of m and of m*com over the distal links
    for i in range(5, -1, -1):
        position, axis_world, mass, com = frames[i]
        total_mass += mass
        moment_sum = _add(moment_sum, (mass * com[0], mass * com[1], mass * com[2]))
        lever_x = moment_sum[0] - total_mass * position[0]
        lever_y = moment_sum[1] - total_mass * position[1]
        # Holding torque = -axis . (lever x m*g), with g = (0, 0, -GRAVITY).
        torques[i] = GRAVITY * (axis_world[0] * lever_y - axis_world[1] * lever_x)
    return tuple(torques)


def gripper_tip(q_raw: Iterable[float]) -> tuple[float, float, float, float]:
    """Return the gripper tip (x, y, z in m, pitch in degrees) in the base frame.

    The tip is the URDF's gripper_end origin; x points forward, y left and z up from the
    base_link origin. Pitch is the jaws' (and wrist camera's) angle above horizontal, so a
    negative pitch looks down.
    """
    angles = _six_numbers(q_raw, "q_raw")
    rotation: Mat = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
    position: Vec = (0.0, 0.0, 0.0)
    for (xyz, origin, axis, _body), (_name, _model, sign, _scale, _limit), angle in zip(
        _CHAIN, JOINT_TABLE, angles, strict=False
    ):
        position = _add(position, _apply(rotation, xyz))
        rotation = _matmul(_matmul(rotation, origin), _axis_angle(axis, sign * angle))
    tip = _add(position, _apply(rotation, _J_GRIPPER_END[0]))
    reach = tuple(t - w for t, w in zip(tip, position, strict=False))
    pitch = math.degrees(
        math.asin(max(-1.0, min(1.0, reach[2] / math.sqrt(sum(v * v for v in reach)))))
    )
    return tip[0], tip[1], tip[2], pitch


def compensation_torques(q_raw: Iterable[float]) -> tuple[float, ...]:
    """Return the 6 MIT feed-forward torques (N*m, raw motor frame) cancelling gravity.

    ``q_raw`` is the six arm joint positions in raw motor radians (mechPos
    0x7019). Each torque is scaled per ``JOINT_TABLE`` and clamped to its
    joint's guard limit. Raises ValueError unless given 6 finite numbers.
    """
    angles = _six_numbers(q_raw, "q_raw")
    model = urdf_gravity(
        tuple(sign * q for (_, _, sign, _, _), q in zip(JOINT_TABLE, angles, strict=False))
    )
    return tuple(
        max(-limit, min(limit, sign * scale * tau))
        for (_, _, sign, scale, limit), tau in zip(JOINT_TABLE, model, strict=False)
    )
