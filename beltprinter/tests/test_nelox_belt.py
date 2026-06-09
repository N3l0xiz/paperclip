"""Unit tests for the Nelox Belt transform and G-code processing.

Run with:  python3 -m unittest discover -s beltprinter/tests
or simply: python3 beltprinter/tests/test_nelox_belt.py
"""
import math
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from nelox_belt import Transform, main, transform_gcode  # noqa: E402

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

    def test_g92_rewritten_to_machine_frame(self):
        # G92 X/Y/Z must be rewritten into machine coords so the firmware frame stays
        # in sync. At y=0,z=0 the machine equivalent is Y0 Z0.
        lines = ["G92 Z0\n", "G1 Z2 Y0\n"]
        out = run(lines, transform=Transform(45.0))
        self.assertEqual(out[0], "G92 Y0 Z0\n")
        self.assertIn("Z2", out[1])        # shift(0,2)=2
        self.assertIn("Y2.8284", out[1])   # lift(2)

    def test_g92_nonzero_context_no_desync(self):
        # Regression for the belt-motion desync bug: a G92 Z0 while the belt (Z) is
        # physically advanced must re-label Z to the current shift, not to 0.
        lines = ["G1 X0 Y10 Z0\n", "G92 Z0\n", "G1 X0 Y10 Z5\n"]
        out = run(lines, transform=Transform(45.0))
        # After "G1 ... Y10 Z0": belt Z = shift(10,0) = 10.
        self.assertIn("Z10", out[0])
        # G92 Z0 relabels to machine Z = shift(10,0) = 10 (NOT 0).
        self.assertIn("Z10", out[1])
        # Final move belt Z = shift(10,5) = 15; true belt progression 15-10 = 5mm.
        self.assertIn("Z15", out[2])

    def test_g92_e0_passes_through(self):
        out = run(["G92 E0\n"], transform=Transform(45.0))
        self.assertEqual(out[0], "G92 E0\n")

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


class TestParsingRobustness(unittest.TestCase):
    def test_scientific_notation_no_phantom_extrusion(self):
        # X1e3 must parse as one word (X=1000), not X1 + a phantom E3.
        out = run(["G1 X1e3 Z2\n"], transform=Transform(45.0))
        self.assertIn("X1000", out[0])
        self.assertNotIn("E3", out[0])

    def test_leading_plus_sign(self):
        out = run(["G1 X+5 Z2\n"], transform=Transform(45.0))
        self.assertIn("X5", out[0])

    def test_no_space_command_is_transformed(self):
        # "G1X10Z2" must be recognized as a move, not passed through verbatim.
        out = run(["G1X10Z2\n"], transform=Transform(45.0))
        self.assertIn("X10", out[0])
        self.assertIn("Z2", out[0])     # shift(0,2)=2
        self.assertNotEqual(out[0], "G1X10Z2\n")

    def test_lowercase_command(self):
        out = run(["g1 x10 z2\n"], transform=Transform(45.0))
        self.assertIn("X10", out[0])

    def test_negative_z_warns(self):
        run(["G1 Z-1\n"], transform=Transform(45.0))
        stats = transform_gcode.last_stats
        self.assertTrue(any("below-belt" in w for w in stats.warnings))


class TestModalFeedrate(unittest.TestCase):
    def test_modal_f_reemitted_on_fless_z_move(self):
        # F set once, then an F-less Z move (layer change) must get a scaled F so the
        # rail/belt axis isn't run at the wrong speed.
        lines = ["G1 X0 Y0 Z0 F1000\n", "G1 Z1\n"]
        out = run(lines, transform=Transform(45.0), scale_feedrate=True)
        # Second move scales F by the machine/real ratio (sqrt3 for a pure-z move).
        self.assertIn("F", out[1])
        f_val = float(out[1].split("F")[1].split()[0])
        self.assertAlmostEqual(f_val, 1000 * math.sqrt(3), places=0)

    def test_feedrate_restored_after_scaled_move(self):
        # in-layer move (scale 1) after a scaled z move should restore F to 1000.
        lines = ["G1 X0 Y0 Z0 F1000\n", "G1 Z1\n", "G1 X10\n"]
        out = run(lines, transform=Transform(45.0), scale_feedrate=True)
        f_val = float(out[2].split("F")[1].split()[0])
        self.assertAlmostEqual(f_val, 1000.0, places=0)

    def test_max_velocity_clamp(self):
        # F1000 is fine, but the z-move scales it to ~1732 > 20 mm/s (1200 mm/min).
        lines = ["G1 X0 Y0 Z0 F1000\n", "G1 Z1\n"]
        out = run(lines, transform=Transform(45.0), scale_feedrate=True, max_velocity=20)
        f_val = float(out[1].split("F")[1].split()[0])
        self.assertAlmostEqual(f_val, 20 * 60, places=0)  # clamped to 1200 mm/min
        self.assertTrue(any("clamped" in w for w in transform_gcode.last_stats.warnings))

    def test_no_scaling_passes_f_through(self):
        out = run(["G1 X10 Z2 F1800\n"], transform=Transform(45.0), scale_feedrate=False)
        self.assertIn("F1800", out[0])


class TestMarkersAndModes(unittest.TestCase):
    def test_arc_passthrough_and_warning(self):
        lines = ["G2 X5 Y5 I1 J1\n"]
        out = run(lines, transform=Transform(45.0))
        self.assertEqual(out[0], lines[0])
        stats = transform_gcode.last_stats
        self.assertEqual(stats.arc_moves, 1)
        self.assertTrue(any("arc" in w for w in stats.warnings))
        self.assertIs(stats.fatal_arcs, True)

    def test_arc_in_transformed_region_main_returns_2_and_writes_nothing(self):
        src = tempfile.NamedTemporaryFile(
            mode="w", suffix=".gcode", delete=False, encoding="utf-8"
        )
        out_path = src.name + ".out"
        try:
            src.write("G1 X0 Y0 Z0\nG2 X10 Y0 I5 J0\n")
            src.close()
            rc = main(["--angle", "45", "-o", out_path, src.name])
            self.assertEqual(rc, 2)
            self.assertFalse(os.path.exists(out_path))
            self.assertIs(transform_gcode.last_stats.fatal_arcs, True)
        finally:
            os.unlink(src.name)
            if os.path.exists(out_path):
                os.unlink(out_path)

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
