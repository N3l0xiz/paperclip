#!/usr/bin/env python3
"""
Nelox Belt — belt-printer post-processor for OrcaSlicer / PrusaSlicer.

Turns ordinary "upright" G-code (sliced as if on a flat bed with vertical Z)
into belt-printer G-code for a tilted-gantry conveyor printer such as the
IdeaFormer IR3 V2 (45° gantry, Klipper firmware).

How it works
------------
A belt printer's gantry is tilted by the gantry angle theta (45° on the IR3 V2).
The firmware (mainline Klipper has no belt kinematics) expects G-code already
expressed in the *sheared* machine frame. We slice the model upright in Orca and
then apply the geometric transform that maps a real-world point (x, y, z) — where
z is true height and y runs along the belt — into machine coordinates:

    X' = x
    Y' = y + z / tan(theta)      # higher layers are pushed forward along the belt
    Z' = z / sin(theta)          # the 45° rail travels a longer distance than the height

For theta = 45°:  Y' = y + z,  Z' = z * sqrt(2).

The transform is linear in (y, z) with no constant term, so it applies
identically to absolute coordinates and to relative deltas (G91).

Usage (Orca/Prusa post-processing script — edits the file in place):
    python3 nelox_belt.py "path/to/sliced.gcode"

Manual / testing:
    python3 nelox_belt.py --angle 45 -o out.gcode in.gcode
    python3 nelox_belt.py --angle 45 --dry-run in.gcode     # report, write nothing

Run `python3 nelox_belt.py --help` for all options.

This is a v1 geometric transform. See README.md for the roadmap (infinite-length
re-ordering) and the firmware caveats around Z-scaling and arc moves.
"""
from __future__ import annotations

import argparse
import math
import re
import sys
from dataclasses import dataclass, field

BRAND = "Nelox Belt"
VERSION = "0.4.0"

# Matches a G-code word like X12.34, Y-5, Z+0.2, E1.5e-3, F1800 — a letter followed
# by a signed number with an optional exponent. The exponent is consumed as part of
# the number so it is never mis-parsed as a separate word (e.g. X1e3 -> one word).
_WORD = re.compile(r"([A-Za-z])\s*([-+]?\d*\.?\d+(?:[eE][-+]?\d+)?)")

# Leading command token of a line, e.g. "G1", "G92", "M82" — tolerant of no space
# before the parameters ("G1X10") and of lowercase ("g1").
_CMD = re.compile(r"\s*([A-Za-z]\d+)")


@dataclass
class Transform:
    """Geometric belt transform for a tilted-gantry printer.

    The transform produces two derived coordinates from an upright model point:

      shift = y + z * cot(theta)   # belt-progression: how far along the belt
      lift  = z / sin(theta)       # gantry-rail travel for the model height

    Which physical machine axis carries each depends on the printer's convention,
    selected by `belt_axis`:

      belt_axis="z"  (IdeaFormer IR3 V2 "infinite Z", the default):
          X_machine = x,  Y_machine = lift,  Z_machine = shift
      belt_axis="y"  (CR-30 style "infinite Y"):
          X_machine = x,  Y_machine = shift, Z_machine = lift

    This was confirmed against hardware-validated belt slicers and the IR3 V2's
    own Klipper config (its belt is the Z axis, position_max ~infinite).
    """

    angle_deg: float = 45.0
    scale_z: bool = True  # scale the lift (rail) axis by 1/sin; off if firmware compensates
    belt_axis: str = "z"  # "z" = IR3 V2 (default), "y" = CR-30

    def __post_init__(self) -> None:
        if not 0 < self.angle_deg < 90:
            raise ValueError(f"gantry angle must be between 0 and 90 degrees, got {self.angle_deg}")
        if self.belt_axis not in ("y", "z"):
            raise ValueError(f"belt_axis must be 'y' or 'z', got {self.belt_axis!r}")
        theta = math.radians(self.angle_deg)
        self._cot = 1.0 / math.tan(theta)          # belt progression per unit height
        self._inv_sin = 1.0 / math.sin(theta)      # rail scaling for height

    def shift(self, y: float, z: float) -> float:
        """Belt-progression coordinate (depends on model y and z)."""
        return y + z * self._cot

    def lift(self, z: float) -> float:
        """Gantry-rail coordinate (depends on model z)."""
        return z * self._inv_sin if self.scale_z else z

    def machine(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        """Map an upright model point (or delta) to machine X/Y/Z."""
        s, l = self.shift(y, z), self.lift(z)
        if self.belt_axis == "z":
            return x, l, s
        return x, s, l


@dataclass
class State:
    """Tracks the modal G-code machine state needed to transform moves correctly."""

    # Real-frame position as the slicer intended it (pre-transform).
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    absolute_xyz: bool = True  # G90 (default) vs G91
    absolute_e: bool = True     # M82 (default) vs M83 (relative extrusion)
    transforming: bool = False  # gated by the begin/end markers
    feed_real: float | None = None      # last modal feedrate the slicer intended (mm/min)
    feed_emitted: float | None = None   # feedrate the firmware currently holds (machine frame)


@dataclass
class Stats:
    moves_transformed: int = 0
    arc_moves: int = 0
    lines: int = 0
    fatal_arcs: bool = False
    warnings: list[str] = field(default_factory=list)


def _fmt(value: float, decimals: int) -> str:
    """Format a coordinate, trimming trailing zeros so the file stays tidy."""
    s = f"{value:.{decimals}f}"
    if "." in s:
        s = s.rstrip("0").rstrip(".")
    return s or "0"


def _split_comment(line: str) -> tuple[str, str]:
    idx = line.find(";")
    if idx == -1:
        return line, ""
    return line[:idx], line[idx:]


def transform_gcode(
    lines,
    transform: Transform,
    begin_marker: str | None = None,
    end_marker: str | None = None,
    scale_feedrate: bool = True,
    scale_extrusion: bool = True,
    decimals: int = 4,
    max_velocity: float | None = None,
):
    """Yield transformed G-code lines. `lines` is any iterable of strings.

    If `begin_marker` is given, transformation only starts after a line containing
    it (case-insensitive substring); otherwise it starts immediately. `end_marker`
    stops it again — useful for end G-code that homes/parks in machine coordinates.
    """
    state = State(transforming=begin_marker is None)
    stats = Stats()

    for raw in lines:
        stats.lines += 1
        line = raw.rstrip("\n")

        # Marker handling (markers are comments, passed through untouched).
        if begin_marker and begin_marker.lower() in line.lower():
            state.transforming = True
            yield raw
            continue
        if end_marker and end_marker.lower() in line.lower():
            state.transforming = False
            yield raw
            continue

        code, comment = _split_comment(line)
        stripped = code.strip()

        if not stripped:
            yield raw
            continue

        # Identify the leading command (tolerant of "G1X10" and lowercase "g1").
        cmd_match = _CMD.match(stripped)
        if not cmd_match:
            yield raw
            continue
        gword = cmd_match.group(1)
        cmd = gword.upper()
        rest = stripped[cmd_match.end():]

        # Track positioning mode regardless of whether we're transforming yet.
        if cmd == "G90":
            state.absolute_xyz = True
            yield raw
            continue
        if cmd == "G91":
            state.absolute_xyz = False
            yield raw
            continue
        if cmd == "M82":
            state.absolute_e = True
            yield raw
            continue
        if cmd == "M83":
            state.absolute_e = False
            yield raw
            continue

        # Which machine axes to (re-)emit for a given set of present model axes.
        def _axis_emits(has_x: bool, has_y: bool, has_z: bool):
            shift_changed = has_y or has_z  # belt progression depends on model y and z
            lift_changed = has_z            # rail lift depends on model z
            if transform.belt_axis == "z":
                return has_x, lift_changed, shift_changed
            return has_x, shift_changed, lift_changed

        if cmd == "G92":
            words = _WORD.findall(rest)
            present = {letter.upper(): value for letter, value in words}
            has_xyz = any(L in present for L in ("X", "Y", "Z"))
            if not (has_xyz and state.transforming):
                # G92 E0 and resets in untransformed (machine-frame) regions: leave be.
                yield raw
                continue
            # Redefine the real-frame origin, then re-emit the equivalent machine
            # coordinates so the firmware's frame stays in sync with ours.
            if "X" in present:
                state.x = float(present["X"])
            if "Y" in present:
                state.y = float(present["Y"])
            if "Z" in present:
                state.z = float(present["Z"])
            mx, my, mz = transform.machine(state.x, state.y, state.z)
            emit_x, emit_y, emit_z = _axis_emits("X" in present, "Y" in present, "Z" in present)
            parts = ["G92"]
            if emit_x:
                parts.append("X" + _fmt(mx, decimals))
            if emit_y:
                parts.append("Y" + _fmt(my, decimals))
            if emit_z:
                parts.append("Z" + _fmt(mz, decimals))
            for letter, value in words:  # keep E and any other reset words verbatim
                if letter.upper() not in ("X", "Y", "Z"):
                    parts.append(letter + value)
            rebuilt = " ".join(parts) + (" " + comment if comment else "")
            yield rebuilt + "\n"
            continue

        is_linear = cmd in ("G0", "G1")
        is_arc = cmd in ("G2", "G3")

        if is_arc and state.transforming:
            # A shear turns a circle into an ellipse — it can't be re-expressed as
            # G2/G3. This is a hard error: emitting wrong-geometry arcs to a real
            # belt printer risks a toolhead crash. Flag it fatal so main() refuses
            # to write output. We still advance state to the arc endpoint (parsing
            # X/Y/Z like a linear move) so downstream coordinates never desync.
            stats.arc_moves += 1
            stats.fatal_arcs = True
            if "arc-fitting" not in " ".join(stats.warnings):
                stats.warnings.append(
                    "arc-fitting move (G2/G3) found and left untransformed — "
                    "disable 'Arc fitting' in the slicer for correct belt geometry"
                )
            arc_words = _WORD.findall(rest)
            arc_present = {letter.upper(): value for letter, value in arc_words}
            if state.absolute_xyz:
                if "X" in arc_present:
                    state.x = float(arc_present["X"])
                if "Y" in arc_present:
                    state.y = float(arc_present["Y"])
                if "Z" in arc_present:
                    state.z = float(arc_present["Z"])
            else:  # relative — deltas
                state.x += float(arc_present.get("X", 0.0))
                state.y += float(arc_present.get("Y", 0.0))
                state.z += float(arc_present.get("Z", 0.0))
            yield raw
            continue

        if not (is_linear and state.transforming):
            yield raw
            continue

        # --- Transform a linear move -------------------------------------------------
        words = _WORD.findall(rest)
        present = {letter.upper(): value for letter, value in words}  # raw value strings

        # Track the modal feedrate the slicer intends, in the real frame.
        if "F" in present:
            state.feed_real = float(present["F"])

        if not any(L in present for L in ("X", "Y", "Z")):
            # Pure E/F move (e.g. retraction or a bare "G1 F1800"): no geometry. The
            # firmware adopts any F here verbatim, so mirror that into feed_emitted.
            if "F" in present:
                state.feed_emitted = state.feed_real
            yield raw
            continue

        # Capture pre-move real position (for feedrate scaling), then advance it.
        prev = (state.x, state.y, state.z)
        if state.absolute_xyz:
            if "X" in present:
                state.x = float(present["X"])
            if "Y" in present:
                state.y = float(present["Y"])
            if "Z" in present:
                state.z = float(present["Z"])
            dx = dy = dz = None  # unused in absolute mode
        else:  # relative — deltas
            dx = float(present.get("X", 0.0))
            dy = float(present.get("Y", 0.0))
            dz = float(present.get("Z", 0.0))
            state.x += dx
            state.y += dy
            state.z += dz
        new = (state.x, state.y, state.z)

        # Safety: a model point below the belt would drive the toolhead into it.
        if state.z < -1e-6 and "below-belt" not in " ".join(stats.warnings):
            stats.warnings.append(
                f"below-belt move: model z = {state.z:g} < 0 — the toolhead would dive "
                "below the belt surface; check model placement / start G-code"
            )

        # Machine/real path-length ratio for this move. Drives both feedrate scaling
        # and extrusion compensation, so compute it whenever either is enabled.
        f_scale = 1.0
        if scale_feedrate or scale_extrusion:
            real_d = math.dist(prev, new)
            if real_d > 1e-9:
                f_scale = math.dist(transform.machine(*prev), transform.machine(*new)) / real_d

        emit_x, emit_my, emit_mz = _axis_emits("X" in present, "Y" in present, "Z" in present)

        # Compute machine coordinates (absolute) or deltas (relative) in one call.
        if state.absolute_xyz:
            mx, my, mz = transform.machine(state.x, state.y, state.z)
        else:
            mx, my, mz = transform.machine(dx, dy, dz)

        # Rebuild in canonical order: command, X, Y, Z, E, extras, F.
        parts = [gword]
        if emit_x:
            parts.append("X" + _fmt(mx, decimals))
        if emit_my:
            parts.append("Y" + _fmt(my, decimals))
        if emit_mz:
            parts.append("Z" + _fmt(mz, decimals))

        # Extrusion compensation. The shear is non-orthonormal, so on moves whose
        # machine path length differs from the model length (z changes while extruding
        # — vase mode / continuous-Z) the deposited volume needs E * f_scale to stay
        # constant. This is a no-op for normal in-layer moves (f_scale == 1). Absolute
        # E (M82) can't be rescaled per-move without rewriting the whole accumulator,
        # so we warn and recommend relative E there.
        if "E" in present:
            if scale_extrusion and abs(f_scale - 1.0) > 1e-9:
                if state.absolute_e:
                    parts.append("E" + present["E"])
                    if "absolute-E extrusion" not in " ".join(stats.warnings):
                        stats.warnings.append(
                            "absolute-E extrusion not compensated on length-changing "
                            "(vase-mode) moves — enable relative E (M83) for exact volume"
                        )
                else:  # relative E: scale the per-move extrusion delta
                    parts.append("E" + _fmt(float(present["E"]) * f_scale, 5))
            else:
                parts.append("E" + present["E"])  # unchanged (normal layer-by-layer)

        for letter, value in words:  # passthrough for any non-geometry words (e.g. S)
            if letter.upper() not in ("X", "Y", "Z", "E", "F"):
                parts.append(letter + value)

        # Feedrate emission. With scaling on, F is modal: re-emit the scaled machine
        # feedrate whenever it differs from what the firmware currently holds (so the
        # rail/belt axis isn't run at the wrong speed on F-less modal moves), and clamp
        # to the machine's max velocity to avoid over-speeding into the belt.
        if scale_feedrate and state.feed_real is not None:
            desired = state.feed_real * f_scale
            if max_velocity is not None and desired > max_velocity * 60.0:
                desired = max_velocity * 60.0
                if "feedrate clamped" not in " ".join(stats.warnings):
                    stats.warnings.append(
                        f"feedrate clamped to max_velocity ({max_velocity} mm/s) on belt moves"
                    )
            if state.feed_emitted is None or abs(desired - state.feed_emitted) > 0.5:
                parts.append("F" + _fmt(desired, 1))
                state.feed_emitted = desired
        elif "F" in present:  # scaling off: pass the original feedrate through
            parts.append("F" + present["F"])

        rebuilt = " ".join(parts)
        if comment:
            rebuilt += " " + comment
        stats.moves_transformed += 1
        yield rebuilt + "\n"

    transform_gcode.last_stats = stats  # type: ignore[attr-defined]


def _header(transform: Transform, scale_feedrate: bool, scale_extrusion: bool = True) -> str:
    shift, lift = "y+z*cot(a)", "z/sin(a)"
    if transform.belt_axis == "z":
        ymap, zmap = lift, shift
    else:
        ymap, zmap = shift, lift
    return (
        f"; Processed by {BRAND} v{VERSION}\n"
        f";   gantry angle = {transform.angle_deg} deg, belt_axis = {transform.belt_axis}, "
        f"scale_z = {transform.scale_z}, scale_feedrate = {scale_feedrate}, "
        f"scale_extrusion = {scale_extrusion}\n"
        f";   transform: X'=x  Y'={ymap}  Z'={zmap}\n"
    )


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        prog="nelox_belt",
        description=f"{BRAND} — belt-printer G-code post-processor (IdeaFormer IR3 V2 and similar).",
    )
    p.add_argument("input", help="input G-code file (edited in place when no -o is given)")
    p.add_argument("-o", "--output", help="write to this file instead of editing in place")
    p.add_argument(
        "-m",
        "--machine",
        help="machine JSON (e.g. machines/ideaformer_ir3v2.json) supplying gantry angle "
        "and post-process defaults; explicit flags below override it",
    )
    p.add_argument("-a", "--angle", type=float, default=None, help="gantry angle in degrees (default: 45)")
    p.add_argument(
        "--belt-axis",
        choices=("y", "z"),
        default=None,
        help="which machine axis is the belt: 'z' = IdeaFormer IR3 V2 (default), 'y' = CR-30",
    )
    p.add_argument(
        "--no-scale-z",
        action="store_true",
        help="do not scale the gantry-rail axis by 1/sin(angle) — use only if firmware compensates the tilt",
    )
    p.add_argument(
        "--no-scale-feedrate",
        action="store_true",
        help="leave F (feedrate) values untouched",
    )
    p.add_argument(
        "--no-scale-extrusion",
        action="store_true",
        help="leave E (extrusion) untouched on length-changing (vase-mode) moves",
    )
    p.add_argument("--begin-marker", help="only transform after a line containing this comment")
    p.add_argument("--end-marker", help="stop transforming at a line containing this comment")
    p.add_argument(
        "--max-velocity",
        type=float,
        default=None,
        help="clamp belt/rail feedrate to this many mm/s (default: from --machine, else none)",
    )
    p.add_argument("--decimals", type=int, default=4, help="coordinate precision (default: 4)")
    p.add_argument("--dry-run", action="store_true", help="analyze and report, but write nothing")
    p.add_argument("--version", action="version", version=f"{BRAND} {VERSION}")
    args = p.parse_args(argv)

    # Layer machine-config defaults under the explicit CLI flags.
    m_angle, m_scale_z, m_scale_f, m_begin, m_end, m_belt, m_maxv, m_scale_e = (
        None, True, True, None, None, "z", None, True,
    )
    if args.machine:
        try:
            from machine import load_machine  # local module, no third-party deps
        except ImportError:
            import os
            sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
            from machine import load_machine
        try:
            mc = load_machine(args.machine)
        except Exception as exc:  # noqa: BLE001 — surface any config error cleanly
            print(f"{BRAND}: cannot load machine {args.machine}: {exc}", file=sys.stderr)
            return 2
        m_angle, m_scale_z, m_scale_f = mc.gantry_angle_deg, mc.scale_z, mc.scale_feedrate
        m_begin, m_end, m_belt = mc.begin_marker, mc.end_marker, mc.belt_axis
        m_maxv, m_scale_e = mc.motion.get("max_velocity"), mc.scale_extrusion

    angle = args.angle if args.angle is not None else (m_angle if m_angle is not None else 45.0)
    belt_axis = args.belt_axis if args.belt_axis is not None else m_belt
    max_velocity = args.max_velocity if args.max_velocity is not None else m_maxv
    # store_true flags can only force OFF; the machine config sets the base value.
    scale_z = (m_scale_z if args.machine else True) and not args.no_scale_z
    scale_feedrate = (m_scale_f if args.machine else True) and not args.no_scale_feedrate
    scale_extrusion = (m_scale_e if args.machine else True) and not args.no_scale_extrusion
    begin_marker = args.begin_marker if args.begin_marker is not None else m_begin
    end_marker = args.end_marker if args.end_marker is not None else m_end

    try:
        transform = Transform(angle_deg=angle, scale_z=scale_z, belt_axis=belt_axis)
    except ValueError as exc:
        print(f"{BRAND}: {exc}", file=sys.stderr)
        return 2

    try:
        with open(args.input, "r", encoding="utf-8", errors="surrogateescape") as fh:
            source = fh.readlines()
    except OSError as exc:
        print(f"{BRAND}: cannot read {args.input}: {exc}", file=sys.stderr)
        return 1

    result = list(
        transform_gcode(
            source,
            transform,
            begin_marker=begin_marker,
            end_marker=end_marker,
            scale_feedrate=scale_feedrate,
            scale_extrusion=scale_extrusion,
            decimals=args.decimals,
            max_velocity=max_velocity,
        )
    )
    stats = transform_gcode.last_stats  # type: ignore[attr-defined]

    for w in stats.warnings:
        print(f"{BRAND}: warning: {w}", file=sys.stderr)

    summary = (
        f"{BRAND}: {stats.moves_transformed} moves transformed, "
        f"{stats.arc_moves} arc moves passed through, {stats.lines} lines total."
    )
    print(summary, file=sys.stderr)

    if args.dry_run:
        return 0

    if stats.fatal_arcs:
        print(
            f"{BRAND}: error: arc moves (G2/G3) found in the transformed region — "
            "disable 'Arc fitting' in the slicer; refusing to write belt G-code that "
            "could crash the toolhead",
            file=sys.stderr,
        )
        return 2

    out_path = args.output or args.input
    payload = _header(transform, scale_feedrate, scale_extrusion) + "".join(result)
    try:
        with open(out_path, "w", encoding="utf-8", errors="surrogateescape") as fh:
            fh.write(payload)
    except OSError as exc:
        print(f"{BRAND}: cannot write {out_path}: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
