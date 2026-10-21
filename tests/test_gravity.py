"""Hardware-free tests for the B601-RS gravity feed-forward model."""

import ast
import hashlib
import json
import math
import os
import random
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from unittest.mock import patch

from inspect_robots_b601.hardware import gravity
from inspect_robots_b601.hardware.gravity import JOINT_TABLE, compensation_torques, urdf_gravity

# A checkout of https://github.com/Seeed-Projects/reBotArm_control_py at 6415d43, if any.
REBOTARM = Path(os.environ.get("REBOTARM_CONTROL_PY", "/nonexistent"))
URDF = REBOTARM / gravity.URDF_SOURCE
SEEED_DATA = REBOTARM / "tools/gravity_calibration/data"
# Seeed's README drops joint2 samples below this as near-table contact
# (tools/gravity_calibration/README.md, "fit_sweeps.py ... --qmin 0.25").
JOINT2_CONTACT_FREE_Q = 0.25


def scaled(q):
    return tuple(scale * tau for (_, _, _, scale, _), tau in zip(JOINT_TABLE, urdf_gravity(q)))


class ReferencePoseTests(unittest.TestCase):
    # Gravity torque about each axis from an independent numpy FK of the same
    # URDF. urdf_gravity returns the holding torque, its negative.
    REFERENCES = (
        ((0, 0, 0, 0, 0, 0), {1: -1.54, 2: -6.76, 3: -2.00, 0: 0.0, 4: 0.0, 5: 0.0}),
        ((0, 0, 1.1, 0, 0, 0), {1: -7.16, 2: -1.15, 3: -0.71}),
        ((0, 0.4, 0.2, 0, 0, 0), {1: -0.58, 2: -7.06}),
    )

    def test_reference_poses(self):
        for q, expected in self.REFERENCES:
            holding = urdf_gravity(q)
            for joint, gravity_torque in expected.items():
                with self.subTest(q=q, joint=joint + 1):
                    self.assertAlmostEqual(-holding[joint], gravity_torque, delta=0.006)

    def test_compensation_is_scaled_model_in_identity_frame(self):
        self.assertEqual([sign for _, _, sign, _, _ in JOINT_TABLE], [1] * 6)
        for q, _ in self.REFERENCES:
            for got, want in zip(compensation_torques(q), scaled(q)):
                self.assertAlmostEqual(got, want, places=12)
        # Folded rest (raw = URDF = 0): the motors must push up, i.e. positive.
        rest = compensation_torques([0] * 6)
        for joint, holding in ((1, 1.545), (2, 6.764), (3, 2.001)):
            self.assertAlmostEqual(rest[joint], JOINT_TABLE[joint][3] * holding, delta=0.005)

    def test_base_yaw_does_not_change_gravity(self):
        base = urdf_gravity((0, 0.7, 1.1, -0.3, 0.4, 0.2))
        turned = urdf_gravity((1.9, 0.7, 1.1, -0.3, 0.4, 0.2))
        for a, b in zip(base, turned):
            self.assertAlmostEqual(a, b, places=9)
        self.assertAlmostEqual(base[0], 0.0, places=9)


@unittest.skipUnless(URDF.exists(), "set REBOTARM_CONTROL_PY to a reBotArm_control_py checkout")
class VendoredUrdfTests(unittest.TestCase):
    def test_source_hash(self):
        self.assertEqual(hashlib.sha256(URDF.read_bytes()).hexdigest(), gravity.URDF_SHA256)

    def test_matches_potential_energy_gradient_of_the_urdf(self):
        """Every vendored number, checked as dU/dq of the URDF read afresh."""
        root = ET.parse(URDF).getroot()
        numbers = lambda text: [float(v) for v in text.split()]
        masses = {link.get("name"): (float(link.find("inertial/mass").get("value")),
                                     numbers(link.find("inertial/origin").get("xyz")))
                  for link in root.findall("link")}
        joints = {joint.find("child").get("link"): joint for joint in root.findall("joint")}

        def mul(a, b):
            return [[sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]

        def turn(axis, angle):  # elementary rotation about x (0), y (1) or z (2)
            c, s = math.cos(angle), math.sin(angle)
            i, j = [(1, 2), (2, 0), (0, 1)][axis]
            m = [[1.0 if r == k else 0.0 for k in range(3)] for r in range(3)]
            m[i][i], m[i][j], m[j][i], m[j][j] = c, -s, s, c
            return m

        def world(link, q):
            if link not in joints:
                return turn(2, 0.0), [0.0, 0.0, 0.0]
            joint = joints[link]
            rotation, position = world(joint.find("parent").get("link"), q)
            xyz = numbers(joint.find("origin").get("xyz"))
            position = [p + sum(rotation[i][k] * xyz[k] for k in range(3))
                        for i, p in enumerate(position)]
            roll, pitch, yaw = numbers(joint.find("origin").get("rpy"))
            rotation = mul(rotation, mul(turn(2, yaw), mul(turn(1, pitch), turn(0, roll))))
            if joint.get("type") == "revolute":
                axis = numbers(joint.find("axis").get("xyz"))
                self.assertIn(axis, ([0, 0, 1], [0, 0, -1]))
                rotation = mul(rotation, turn(2, axis[2] * q[int(joint.get("name")[-1]) - 1]))
            return rotation, position

        def potential(q):
            total = 0.0
            for link, (mass, com) in masses.items():
                rotation, position = world(link, q)
                height = position[2] + sum(rotation[2][k] * com[k] for k in range(3))
                total += mass * 9.81 * height
            return total

        rng = random.Random(601)
        for _ in range(20):
            q = [rng.uniform(-3.0, 3.0) for _ in range(6)]
            model = urdf_gravity(q)
            for i in range(6):
                step = [0.0] * 6
                step[i] = 1e-6
                plus = potential([a + b for a, b in zip(q, step)])
                minus = potential([a - b for a, b in zip(q, step)])
                self.assertAlmostEqual(model[i], (plus - minus) / 2e-6, delta=1e-5)


class Joint2SweepTests(unittest.TestCase):
    """Seeed's joint2 PD sweep in the elbow-up L pose (PD only, no feed-forward)."""

    def setUp(self):
        paths = sorted(SEEED_DATA.glob("pdsweep_joint2_Lpose_*.json"))
        if not paths:
            self.skipTest("Seeed sweep data is not in this tree")
        samples = json.loads(paths[0].read_text())["samples"]
        self.samples = [s for s in samples if s["q"] > JOINT2_CONTACT_FREE_Q]

    def residuals(self, model):
        out = {1: [], -1: []}
        for sample in self.samples:
            out[sample["dir"]].append(sample["tau_est"] - model(sample["q6"]))
        return out

    @staticmethod
    def rms(values):
        return math.sqrt(sum(v * v for v in values) / len(values))

    def test_compensation_matches_holding_torque_within_friction(self):
        residuals = self.residuals(lambda q6: compensation_torques(q6)[1])
        up, down = residuals[1], residuals[-1]
        self.assertGreater(len(up), 300)
        self.assertGreater(len(down), 300)
        # Measured: up rms 0.53, mean +0.42; down rms 0.73, mean -0.64.
        self.assertLess(self.rms(up), 0.8)
        self.assertLess(self.rms(down), 0.8)
        mean_up, mean_down = sum(up) / len(up), sum(down) / len(down)
        # Coulomb friction adds torque moving up and removes it moving down.
        self.assertTrue(0.0 < mean_up < 0.8, mean_up)
        self.assertTrue(-0.8 < mean_down < 0.0, mean_down)
        friction = (mean_up - mean_down) / 2
        self.assertTrue(0.3 < friction < 0.8, friction)  # Seeed's fit: 0.53
        without_friction = [r - friction for r in up] + [r + friction for r in down]
        self.assertLess(self.rms(without_friction), 0.45)  # measured 0.35

    def test_sweep_rejects_a_flipped_joint2_sign(self):
        def flipped(q6):
            q = list(q6)
            q[1] = -q[1]
            return -JOINT_TABLE[1][3] * urdf_gravity(q)[1]
        residuals = self.residuals(flipped)
        self.assertGreater(self.rms(residuals[1]), 2.0)
        self.assertGreater(self.rms(residuals[-1]), 2.0)

    def test_holding_torque_profile(self):
        pose = list(self.samples[0]["q6"])
        def at(q2):
            pose[1] = q2
            return compensation_torques(pose)[1]
        self.assertAlmostEqual(at(0.3), 4.3, delta=0.3)
        self.assertGreater(at(0.7), 0.0)
        self.assertLess(at(0.85), 0.0)


class ClampTests(unittest.TestCase):
    def test_outstretched_arm_hits_the_rs06_guard(self):
        model = urdf_gravity((0, 0, 2.8, 0, 0, 0))
        self.assertGreater(JOINT_TABLE[1][3] * model[1], 12.0)
        self.assertEqual(compensation_torques((0, 0, 2.8, 0, 0, 0))[1], 12.0)

    def test_guards_are_per_motor_and_symmetric(self):
        self.assertEqual([limit for *_, limit in JOINT_TABLE], [12.0] * 3 + [4.0] * 3)
        loud = tuple((name, model, sign, 100.0, limit)
                     for name, model, sign, _, limit in JOINT_TABLE)
        with patch.object(gravity, "JOINT_TABLE", loud):
            folded = compensation_torques((0, 0, 0, 0, 0, 0))
            forward = compensation_torques((0, 2.967, 0, 0, 0, 0))
        self.assertEqual(folded[1:4], (12.0, 12.0, 4.0))
        self.assertEqual(forward[1:4], (-12.0, -12.0, -4.0))
        for tau, (*_, limit) in zip(folded + forward, JOINT_TABLE * 2):
            self.assertLessEqual(abs(tau), limit)


class InputTests(unittest.TestCase):
    def test_accepts_six_finite_numbers(self):
        self.assertEqual(len(compensation_torques([0, 0.5, 1, -0.2, 0.1, 0.3])), 6)
        self.assertEqual(compensation_torques(iter((0.0,) * 6)), compensation_torques((0,) * 6))
        self.assertTrue(all(isinstance(tau, float) for tau in compensation_torques((0,) * 6)))

    def test_rejects_anything_else(self):
        bad = [
            (0.0,) * 5, (0.0,) * 7, (), None, 0.0, "012345",
            (0, 0, 0, 0, 0, "0"), (0, 0, 0, 0, 0, None), (0, 0, 0, 0, 0, True),
            (0, 0, 0, 0, 0, math.nan), (0, 0, 0, 0, 0, math.inf), (0, 0, 0, 0, 0, -math.inf),
        ]
        for q in bad:
            with self.subTest(q=q), self.assertRaises(ValueError):
                compensation_torques(q)
            with self.subTest(urdf=q), self.assertRaises(ValueError):
                urdf_gravity(q)


class PurityTests(unittest.TestCase):
    def test_imports_only_the_standard_library(self):
        # No numpy or URDF tooling: the vendored model must run in any install.
        tree = ast.parse(Path(gravity.__file__).read_text())
        imported = {alias.name for node in ast.walk(tree) if isinstance(node, ast.Import)
                    for alias in node.names}
        imported |= {node.module for node in ast.walk(tree) if isinstance(node, ast.ImportFrom)}
        self.assertEqual(imported, {"math", "__future__", "collections.abc"})


if __name__ == "__main__":
    unittest.main()
