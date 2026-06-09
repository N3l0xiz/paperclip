"""Unit tests for the Nelox Belt transform and G-code processing.

Run with:  python3 -m unittest discover -s beltprinter/tests
or simply: python3 beltprinter/tests/test_nelox_belt.py
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from nelox_belt import Transform, transform_gcode  # noqa: E402

SQRT2 = math.sqrt(2)


def run(lines, **kwargs):
    return list(transform_gcode(lines, **kwargs))


class TestTransform(unittest.TestCase):
    def test_shift_and_lift_45(self):
        t = Transform(angle_deg=45.0)
        self.assertAlmostEqual(t.shift(5.0, 2.0), 7.0)        # y + z*cot(45)
        self.assertAlmostEqual(t.lift(2.0), 2.0 * SQRT2)      # z / sin(45)

    def test_belt_axis_z_is_default(self):
        # IR3 V2: belt is machine Z (gets shift), gantry rail is machine Y (gets lift).
        t = Transform(angle_deg=45.0)
        self.assertEqual(t.belt_axis, "z")
        x, y, z = t.machine(10.0, 5.0, 2.0)
        self.assertAlmostEqual(x, 10.0)
        self.assertAlmostEqual(y, 2.0 * SQRT2)   # lift
        self.assertAlmostEqual(z, 7.0)           # shift

    def test_belt_axis_y_swaps_outputs(self):
        # CR-30: belt is machine Y (gets shift), rail is machine Z (gets lift).
        t = Transform(angle_deg=45.0, belt_axis="y")
        x, y, z = t.machine(10.0, 5.0, 2.0)
        self.assertAlmostEqual(y, 7.0)           # shift
        self.assertAlmostEqual(z, 2.0 * SQRT2)   # lift

    def test_no_scale_z_affects_lift_only(self):
        t = Transform(angle_deg=45.0, scale_z=False)
        self.assertAlmostEqual(t.lift(2.0), 2.0)     # unscaled
        self.assertAlmostEqual(t.shift(5.0, 2.0), 7.0)

    def test_general_angle_30(self):
        t = Transform(angle_deg=30.0)
        self.assertAlmostEqual(t.shift(0.0, 3.0), 3.0 / math.tan(math.radians(30)))
        self.assertAlmostEqual(t.lift(3.0), 3.0 / math.sin(math.radians(30)))

    def test_invalid_angles(self):
        for bad in (0, 90, -5, 120):
            with self.assertRaises(ValueError):
                Transform(angle_deg=bad)

    def test_invalid_belt_axis(self):
        with self.assertRaises(ValueError):
            Transform(angle_deg=45, belt_axis="x")


class TestGcodeProcessingBeltZ(unittest.TestCase):
    """Default IR3 V2 convention: belt on machine Z."""

    def test_absolute_move(self):
        out = run(["G1 X10 Y5 Z2 E1.0 F1800\n"], transform=Transform(45.0), scale_feedrate=False)
        line = out[0]
        self.assertIn("X10", line)
        self.assertIn("Y2.8284", line)   # lift = 2*sqrt2 on machine Y
        self.assertIn("Z7", line)        # shift = 5+2 on machine Z
        self.assertIn("E1.0", line)      # extrusion untouched
        self.assertIn("F1800", line)

    def test_extrusion_and_comment_preserved(self):
        out = run(["G1 X1 Y1 Z1 E0.5 ; outer wall\n"], transform=Transform(45.0))
        self.assertIn("E0.5", out[0])
        self.assertIn("; outer wall", out[0])

    def test_relative_positioning(self):
        lines = ["G91\n", "G1 Z2 Y1\n", "G1 Z2\n"]
        out = run(lines, transform=Transform(45.0))
        # dz=2, dy=1 -> machine Y=lift(2)=2.8284, machine Z=shift(1,2)=3
        self.assertIn("Y2.8284", out[1])
        self.assertIn("Z3", out[1])
        # dz=2, dy=0 -> Y=2.8284, Z=shift(0,2)=2
        self.assertIn("Y2.8284", out[2])
        self.assertIn("Z2", out[2])

    def test_z_only_move_emits_both_axes(self):
        # A Z-only model move changes both lift (Y) and shift (Z).
        out = run(["G1 Z2\n"], transform=Transform(45.0))
        self.assertIn("Y2.8284", out[0])
        self.assertIn("Z2", out[0])

    def test_pure_retraction_untouched(self):
        out = run(["G1 E-2 F2400\n"], transform=Transform(45.0))
        self.assertEqual(out[0], "G1 E-2 F2400\n")

    def test_g92_resets_real_frame(self):
        lines = ["G92 Z0\n", "G1 Z2 Y0\n"]
        out = run(lines, transform=Transform(45.0))
        self.assertEqual(out[0], "G92 Z0\n")
        self.assertIn("Z2", out[1])        # shift(0,2)=2
        self.assertIn("Y2.8284", out[1])   # lift(2)

    def test_feedrate_scaling_pure_z(self):
        # Model Z move dz=1: machine delta has lift=sqrt2 and shift=1, len=sqrt(3).
        out = run(["G1 Z1 F1000\n"], transform=Transform(45.0), scale_feedrate=True)
        f_val = float(out[0].split("F")[1].split()[0])
        self.assertAlmostEqual(f_val, 1000 * math.sqrt(3), places=0)


class TestGcodeProcessingBeltY(unittest.TestCase):
    """CR-30 convention: belt on machine Y."""

    def test_absolute_move(self):
        out = run(["G1 X10 Y5 Z2\n"], transform=Transform(45.0, belt_axis="y"), scale_feedrate=False)
        self.assertIn("Y7", out[0])        # shift on Y
        self.assertIn("Z2.8284", out[0])   # lift on Z


class TestMarkersAndModes(unittest.TestCase):
    def test_arc_passthrough_and_warning(self):
        lines = ["G2 X5 Y5 I1 J1\n"]
        out = run(lines, transform=Transform(45.0))
        self.assertEqual(out[0], lines[0])
        stats = transform_gcode.last_stats
        self.assertEqual(stats.arc_moves, 1)
        self.assertTrue(any("arc" in w for w in stats.warnings))

    def test_begin_marker_gates_transform(self):
        lines = ["G1 X1 Y1 Z1\n", "; nelox:begin\n", "G1 X1 Y1 Z1\n"]
        out = run(lines, transform=Transform(45.0), begin_marker="; nelox:begin")
        self.assertEqual(out[0], "G1 X1 Y1 Z1\n")  # before marker: untouched
        self.assertIn("Z2", out[2])                # after marker: shift=1+1 on Z

    def test_end_marker_stops_transform(self):
        lines = ["G1 X1 Y1 Z1\n", "; nelox:end\n", "G1 X1 Y1 Z1\n"]
        out = run(lines, transform=Transform(45.0), end_marker="; nelox:end")
        self.assertIn("Z2", out[0])
        self.assertEqual(out[2], "G1 X1 Y1 Z1\n")

    def test_g90_g91_tracking_does_not_emit_changes(self):
        out = run(["G90\n", "M82\n"], transform=Transform(45.0))
        self.assertEqual(out, ["G90\n", "M82\n"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
