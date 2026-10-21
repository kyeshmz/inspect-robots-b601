"""The Inspect Robots adapter guide's checks for the b601 embodiment; nothing touches hardware.

https://docs.inspectrobots.org/guide/adapters/
"""

import inspect
import unittest

from inspect_robots.conformance import DeviceSlot, assert_embodiment_conformant
from inspect_robots.registry import registered

from inspect_robots_b601 import B601Embodiment

CAN_SLOT = DeviceSlot(arg="channel", kind="can", label="B601-RS arm CAN channel")


class AdapterGuideTests(unittest.TestCase):
    def test_embodiment_is_conformant(self):
        for robot in (B601Embodiment(), B601Embodiment(camera="realsense+depth")):
            with self.subTest(cameras=robot.info.observation_space.cameras):
                assert_embodiment_conformant(robot.info)
                self.assertIsNotNone(robot.info.control_hz)
                self.assertEqual(robot.info.action_space.semantics.control_mode, "joint_pos")

    def test_registry_constructs_it_with_no_arguments(self):
        factory = registered("embodiment")["b601"]
        self.assertIs(factory, B601Embodiment)
        robot = factory()
        self.assertEqual(robot.info.name, "b601")
        self.assertEqual(robot.info.action_space.shape, (7,))
        self.assertIsNone(robot._session)

    def test_runtime_requirements_point_at_the_motorbridge_extra(self):
        self.assertEqual(B601Embodiment.RUNTIME_REQUIREMENTS,
                         {"motorbridge": "python -m pip install 'inspect-robots-b601[motorbridge]'"})

    def test_constructor_kwargs_are_scalars_for_the_cli(self):
        for parameter in inspect.signature(B601Embodiment).parameters.values():
            with self.subTest(parameter=parameter.name):
                self.assertEqual(parameter.kind, parameter.KEYWORD_ONLY)
                self.assertIsInstance(parameter.default, (str, bool, int, float))

    def test_setup_wizard_can_fill_the_can_channel(self):
        self.assertEqual(B601Embodiment.DEVICE_SLOTS, (CAN_SLOT,))
        self.assertEqual(B601Embodiment(channel="can3").profile["channel"], "can3")
        self.assertEqual(B601Embodiment().profile["channel"], "can0")
        with self.assertRaises(ValueError):
            B601Embodiment(channel="../can0")

    def test_setup_wizard_can_turn_motion_on_only_with_every_answer(self):
        self.assertEqual([slot.arg for slot in B601Embodiment.OPTION_SLOTS],
                         ["allow_motion", "operator_supported_arm", "activate_on_reset"])
        self.assertTrue(all(slot.default is False for slot in B601Embodiment.OPTION_SLOTS))
        with self.assertRaises(ValueError):
            B601Embodiment(activate_on_reset=True)
        with self.assertRaises(ValueError):
            B601Embodiment(allow_motion=True, activate_on_reset=True)
        B601Embodiment(allow_motion=True, operator_supported_arm=True, activate_on_reset=True)

    def test_context_manager_closes(self):
        with B601Embodiment() as robot:
            self.assertIsInstance(robot, B601Embodiment)
        self.assertIsNone(robot._session)


if __name__ == "__main__":
    unittest.main()
