"""Unit tests for the machine config loader."""
import math
import os
import sys
import unittest

HERE = os.path.dirname(__file__)
sys.path.insert(0, os.path.join(HERE, ".."))

from machine import Axis, load_machine, to_klipper_cfg  # noqa: E402

IR3V2 = os.path.join(HERE, "..", "machines", "ideaformer_ir3v2.json")


class TestAxis(unittest.TestCase):
    def test_ungeared_effective_equals_rotation_distance(self):
        a = Axis("x", rotation_distance=40, gear_ratio=(1, 1), microsteps=32)
        self.assertAlmostEqual(a.effective_rotation_distance, 40)
        # 200 * 32 / 40 = 160 steps/mm
        self.assertAlmostEqual(a.steps_per_mm, 200 * 32 / 40)

    def test_geared_effective(self):
        # 50:17 reduction applied to a 22mm ungeared rotation_distance.
        a = Axis("e", rotation_distance=22.0, gear_ratio=(50, 17))
        self.assertAlmostEqual(a.ratio, 50 / 17)
        self.assertAlmostEqual(a.effective_rotation_distance, 22.0 / (50 / 17))

    def test_invalid_gear_ratio(self):
        with self.assertRaises(ValueError):
            Axis("x", rotation_distance=40, gear_ratio=(0, 1))

    def test_invalid_rotation_distance(self):
        with self.assertRaises(ValueError):
            Axis("x", rotation_distance=0)

    def test_invalid_microsteps(self):
        with self.assertRaises(ValueError):
            Axis("x", rotation_distance=40, microsteps=0)


class TestIR3V2Config(unittest.TestCase):
    def setUp(self):
        self.m = load_machine(IR3V2)

    def test_core_values(self):
        self.assertEqual(self.m.gantry_angle_deg, 45.0)
        self.assertEqual(self.m.kinematics, "corexy")
        self.assertEqual(self.m.nozzle_diameter, 0.4)
        self.assertEqual(self.m.motion["max_velocity"], 400)

    def test_axis_rotation_distances(self):
        self.assertEqual(self.m.axes["x"].rotation_distance, 40)
        self.assertEqual(self.m.axes["y"].rotation_distance, 40)
        self.assertEqual(self.m.axes["z"].rotation_distance, 3.7)
        self.assertEqual(self.m.axes["extruder"].rotation_distance, 4.4)

    def test_postprocess_defaults(self):
        self.assertTrue(self.m.scale_z)
        self.assertTrue(self.m.scale_feedrate)
        self.assertEqual(self.m.begin_marker, "; nelox:begin")
        self.assertEqual(self.m.end_marker, "; nelox:end")
        self.assertEqual(self.m.belt_axis, "z")  # IR3 V2 drives the belt as Z

    def test_x_steps_per_mm(self):
        # 200 full steps * 32 microsteps / 40 mm = 160 steps/mm
        self.assertAlmostEqual(self.m.axes["x"].steps_per_mm, 160.0)

    def test_extra_keys_preserved(self):
        # Extra (non-Axis) keys round-trip through loading. y is the gantry rail
        # (lift), so is_belt_feed is False; the belt is z (position_max 99999).
        self.assertIs(self.m.axes["y"].extra.get("is_belt_feed"), False)
        self.assertEqual(self.m.axes["y"].extra.get("position_max"), 354)
        self.assertEqual(self.m.axes["z"].extra.get("position_max"), 99999)

    def test_klipper_cfg_snippet(self):
        cfg = to_klipper_cfg(self.m)
        self.assertIn("[stepper_x]", cfg)
        self.assertIn("[extruder]", cfg)
        self.assertIn("rotation_distance: 3.7", cfg)
        self.assertIn("rotation_distance: 4.4", cfg)


if __name__ == "__main__":
    unittest.main(verbosity=2)
