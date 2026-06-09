"""Unit tests for the Nelox Belt transform and G-code processing.

Run with:  python3 -m unittest discover -s nelox-belt/tests
or simply: python3 nelox-belt/tests/test_nelox_belt.py
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from nelox_belt import Transform, transform_gcode  # noqa: E402


def run(lines, **kwargs):
    return list(transform_gcode(lines, **kwargs))


class TestTransform(unittest.TestCase):
    def test_45_degrees(self):
        t = Transform(angle_deg=45.0)
        x, y, z = t.point(10.0, 5.0, 2.0)
        self.assertAlmostEqual(x, 10.0)
        self.assertAlmostEqual(y, 5.0 + 2.0)            # y + z*cot(45)=y+z
        self.assertAlmostEqual(z, 2.0 * math.sqrt(2))   # z/sin(45)

    def test_x_is_identity(self):
        t = Transform(angle_deg=45.0)
        self.assertAlmostEqual(t.point(123.4, 0, 0)[0], 123.4)

    def test_no_scale_z(self):
        t = Transform(angle_deg=45.0, scale_z=False)
        self.assertAlmostEqual(t.z(2.0), 2.0)
        # Y shift is unaffected by the scale_z flag.
        self.assertAlmostEqual(t.y(5.0, 2.0), 7.0)

    def test_general_angle_30(self):
        t = Transform(angle_deg=30.0)
        z = 3.0
        self.assertAlmostEqual(t.y(0.0, z), z / math.tan(math.radians(30)))
        self.assertAlmostEqual(t.z(z), z / math.sin(math.radians(30)))

    def test_invalid_angles(self):
        for bad in (0, 90, -5, 120):
            with self.assertRaises(ValueError):
                Transform(angle_deg=bad)


class TestGcodeProcessing(unittest.TestCase):
    def test_absolute_move(self):
        out = run(["G1 X10 Y5 Z2 E1.0 F1800\n"], transform=Transform(45.0), scale_feedrate=False)
        line = out[0]
        self.assertIn("X10", line)
        self.assertIn("Y7", line)            # 5 + 2
        self.assertIn("Z2.8284", line)       # 2 * sqrt(2)
        self.assertIn("E1.0", line)          # extrusion untouched
        self.assertIn("F1800", line)

    def test_extrusion_and_comment_preserved(self):
        out = run(["G1 X1 Y1 Z1 E0.5 ; outer wall\n"], transform=Transform(45.0))
        self.assertIn("E0.5", out[0])
        self.assertIn("; outer wall", out[0])

    def test_relative_positioning(self):
        lines = ["G91\n", "G1 Z2 Y1\n", "G1 Z2\n"]
        out = run(lines, transform=Transform(45.0))
        # First relative Z move: dz=2 -> Y'=1+2=3, Z'=2*sqrt2
        self.assertIn("Y3", out[1])
        self.assertIn("Z2.8284", out[1])
        # Second: cumulative real z is now 4, but as a relative delta dz=2 again.
        self.assertIn("Y2", out[2])
        self.assertIn("Z2.8284", out[2])

    def test_pure_retraction_untouched(self):
        out = run(["G1 E-2 F2400\n"], transform=Transform(45.0))
        self.assertEqual(out[0], "G1 E-2 F2400\n")

    def test_g92_resets_real_frame(self):
        lines = ["G92 Z0\n", "G1 Z2 Y0\n"]
        out = run(lines, transform=Transform(45.0))
        self.assertEqual(out[0], "G92 Z0\n")  # passed through
        self.assertIn("Z2.8284", out[1])

    def test_arc_passthrough_and_warning(self):
        lines = ["G2 X5 Y5 I1 J1\n"]
        out = run(lines, transform=Transform(45.0))
        self.assertEqual(out[0], lines[0])  # untouched
        stats = transform_gcode.last_stats
        self.assertEqual(stats.arc_moves, 1)
        self.assertTrue(any("arc" in w for w in stats.warnings))

    def test_begin_marker_gates_transform(self):
        lines = ["G1 X1 Y1 Z1\n", "; nelox:begin\n", "G1 X1 Y1 Z1\n"]
        out = run(lines, transform=Transform(45.0), begin_marker="; nelox:begin")
        self.assertEqual(out[0], "G1 X1 Y1 Z1\n")  # before marker: untouched
        self.assertIn("Y2", out[2])                # after marker: transformed

    def test_end_marker_stops_transform(self):
        lines = ["G1 X1 Y1 Z1\n", "; nelox:end\n", "G1 X1 Y1 Z1\n"]
        out = run(lines, transform=Transform(45.0), end_marker="; nelox:end")
        self.assertIn("Y2", out[0])
        self.assertEqual(out[2], "G1 X1 Y1 Z1\n")

    def test_feedrate_scaling_pure_z(self):
        # A pure real-Z move of dz=1: machine delta = (0, 1, sqrt2), len=sqrt(3).
        # real len = 1, so F scales by sqrt(3).
        out = run(["G1 Z1 F1000\n"], transform=Transform(45.0), scale_feedrate=True)
        expected = 1000 * math.sqrt(3)
        # Parse F back out.
        f_val = float(out[0].split("F")[1].split()[0])
        self.assertAlmostEqual(f_val, expected, places=0)

    def test_g90_g91_tracking_does_not_emit_changes(self):
        out = run(["G90\n", "M82\n"], transform=Transform(45.0))
        self.assertEqual(out, ["G90\n", "M82\n"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
