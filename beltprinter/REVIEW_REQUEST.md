# Independent review request — Nelox Belt (OrcaSlicer belt-printer post-processor)

You are reviewing a Python post-processing script that converts ordinary
"upright-sliced" G-code from OrcaSlicer into belt-printer G-code for a 45°
tilted-gantry conveyor printer (IdeaFormer IR3 V2, Klipper firmware, CoreXY).

**This output drives real hardware.** Bad geometry can crash the toolhead into the
belt or fling the gantry. Review accordingly — correctness and safety first.

This is **v0.4.0**, already hardened by one prior independent review. Please both
re-verify the core and focus on what changed (listed below).

## What it claims to do

Slice the model UPRIGHT (normal horizontal layers), then per point `(x, y, z)`
(z = true height, y = along the belt) derive:

    shift = y + z / tan(theta)    # belt progression
    lift  = z / sin(theta)        # gantry-rail travel for the height

and map to machine axes by `belt_axis`:

    belt_axis="z" (IR3 V2 default):  X'=x  Y'=lift   Z'=shift
    belt_axis="y" (CR-30):           X'=x  Y'=shift  Z'=lift

theta = gantry angle (45°). The IR3 V2 drives the conveyor as the (infinite) Z axis.

## Context: how this compares to the reference fork (ShidaoSlicer)

ShidaoSlicer is a hardware-validated native OrcaSlicer fork for this exact printer
family. Its real pipeline does NATIVE oblique slicing (45° cut planes) plus an
orthonormal Virtual→Firmware axis permutation, and scales layer height by sqrt(2) in
the slicer. Its documented net model→machine mapping at 45° is
`Y_machine = sqrt(2)·Z_model`, `Z_machine = Y_model + Z_model` — which this script's
`belt_axis="z"` mapping reproduces exactly. ShidaoSlicer also keeps a "legacy shear"
(`CompatibilityMode::create_legacy_shear_transform`) that is the direct analog of THIS
tool; notably it recomputes extrusion for non-orthonormal transforms
(`E_post = E_pre · length_post/length_pre`). That is the basis for the new extrusion
change below. Please sanity-check our math against that reference.

## What changed since the last review — RE-CHECK THESE

1. **Belt-axis mapping** is now `belt_axis="z"` by default (belt on machine Z), since
   the IR3 V2 is "infinite Z". Confirm the lift→Y / shift→Z routing is right, and that
   `belt_axis="y"` swaps them correctly.
2. **G92 X/Y/Z** is rewritten into machine coordinates (previously passed verbatim,
   causing firmware/model frame desync). Check the rewrite is correct, including the
   non-zero-context case, and that `G92 E0` is left alone.
3. **Modal feedrate**: F is tracked and re-emitted (scaled) on F-less moves, clamped to
   `max_velocity`. Check for over-/under-speed, and that returning to in-layer moves
   restores the original feedrate.
4. **Extrusion compensation (NEW)**: on length-changing extruding moves (vase mode /
   continuous-Z), relative-E (M83) deltas are scaled by the machine/model length ratio;
   absolute-E (M82) is left alone with a warning. **Scrutinize this:** Is scaling the
   relative-E delta by `f_scale` the correct volumetric compensation? Is leaving E
   untouched on in-layer moves (f_scale==1) right? Is the absolute-E warning-only
   stance acceptable, or is there a safe per-move absolute-E rewrite we should do?
5. **Parsing robustness**: tolerates no-space commands (`G1X10`), lowercase, leading
   `+`, and scientific notation. Check for any new mis-parse.
6. **Safety**: below-belt (model z<0) warning; arc (G2/G3) passthrough warning.

## Please also re-check the original concerns

- Math correctness of shift/lift at 45° from first principles.
- Relative mode (G91) applying the same linear transform to deltas.
- Axis re-emission: when only model-Z changes, both machine Y and Z update.
- G-code parsing edge cases and state tracking (G90/91, M82/83, G92).
- Any dangerous output that could reach hardware.

Be concrete: cite line numbers and give minimal failing examples. End with a verdict
on whether it is safe to run on an IR3 V2 (with relative E + arc-fitting off).


## SOURCE: nelox_belt.py

```python
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
```

## SOURCE: machine.py

```python
#!/usr/bin/env python3
"""
Machine configuration for Nelox Belt.

Loads an adjustable belt-printer machine definition (see machines/*.json) and
exposes the values the rest of the toolchain needs: the gantry angle and
post-processing defaults for nelox_belt.py, plus per-axis gearing so the config
is the single source of truth for the printer's mechanics.

Gear ratios follow Klipper's convention: gear_ratio = [numerator, denominator]
means the axis moves `rotation_distance` for `numerator/denominator` motor
revolutions, so the distance per MOTOR revolution is:

    effective_rotation_distance = rotation_distance / (numerator / denominator)

CLI:
    python3 machine.py machines/ideaformer_ir3v2.json            # summary
    python3 machine.py machines/ideaformer_ir3v2.json --print-cfg  # Klipper snippet
"""
from __future__ import annotations

import argparse
import json
import sys
from dataclasses import dataclass, field


@dataclass
class Axis:
    name: str
    rotation_distance: float
    gear_ratio: tuple[float, float] = (1.0, 1.0)
    microsteps: int = 16
    full_steps_per_rotation: int = 200
    geared: bool = False
    extra: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        num, den = self.gear_ratio
        if num <= 0 or den <= 0:
            raise ValueError(f"axis '{self.name}': gear_ratio parts must be > 0, got {self.gear_ratio}")
        if self.rotation_distance <= 0:
            raise ValueError(f"axis '{self.name}': rotation_distance must be > 0")
        if self.microsteps <= 0:
            raise ValueError(f"axis '{self.name}': microsteps must be > 0, got {self.microsteps}")
        if self.full_steps_per_rotation <= 0:
            raise ValueError(
                f"axis '{self.name}': full_steps_per_rotation must be > 0, got {self.full_steps_per_rotation}"
            )

    @property
    def ratio(self) -> float:
        """Reduction as a single number, e.g. [50, 17] -> 2.941."""
        num, den = self.gear_ratio
        return num / den

    @property
    def effective_rotation_distance(self) -> float:
        """Distance moved per stepper-motor revolution (gearing applied)."""
        return self.rotation_distance / self.ratio

    @property
    def steps_per_mm(self) -> float:
        motor_steps = self.full_steps_per_rotation * self.microsteps
        return motor_steps / self.effective_rotation_distance


@dataclass
class Machine:
    name: str
    gantry_angle_deg: float
    axes: dict[str, Axis]
    firmware: str = "klipper"
    kinematics: str = "corexy"
    nozzle_diameter: float = 0.4
    filament_diameter: float = 1.75
    build_volume: dict = field(default_factory=dict)
    motion: dict = field(default_factory=dict)
    postprocess: dict = field(default_factory=dict)

    def __post_init__(self) -> None:
        if not 0 < self.gantry_angle_deg < 90:
            raise ValueError(f"gantry_angle_deg must be between 0 and 90, got {self.gantry_angle_deg}")

    # --- convenience accessors for nelox_belt.py --------------------------------
    @property
    def scale_z(self) -> bool:
        return bool(self.postprocess.get("scale_z", True))

    @property
    def scale_feedrate(self) -> bool:
        return bool(self.postprocess.get("scale_feedrate", True))

    @property
    def scale_extrusion(self) -> bool:
        return bool(self.postprocess.get("scale_extrusion", True))

    @property
    def begin_marker(self):
        return self.postprocess.get("begin_marker")

    @property
    def end_marker(self):
        return self.postprocess.get("end_marker")

    @property
    def belt_axis(self) -> str:
        return self.postprocess.get("belt_axis", "z")


_RESERVED_AXIS_KEYS = {
    "rotation_distance",
    "gear_ratio",
    "microsteps",
    "full_steps_per_rotation",
    "geared",
}


def _axis_from_dict(name: str, d: dict) -> Axis:
    gr = d.get("gear_ratio", [1, 1])
    if not (isinstance(gr, (list, tuple)) and len(gr) == 2):
        raise ValueError(f"axis '{name}': gear_ratio must be a [numerator, denominator] pair")
    extra = {k: v for k, v in d.items() if k not in _RESERVED_AXIS_KEYS}
    return Axis(
        name=name,
        rotation_distance=float(d["rotation_distance"]),
        gear_ratio=(float(gr[0]), float(gr[1])),
        microsteps=int(d.get("microsteps", 16)),
        full_steps_per_rotation=int(d.get("full_steps_per_rotation", 200)),
        geared=bool(d.get("geared", False)),
        extra=extra,
    )


def load_machine(path: str) -> Machine:
    with open(path, "r", encoding="utf-8") as fh:
        data = json.load(fh)
    axes = {name: _axis_from_dict(name, d) for name, d in data.get("axes", {}).items()}
    return Machine(
        name=data.get("name", "unnamed"),
        gantry_angle_deg=float(data["gantry_angle_deg"]),
        axes=axes,
        firmware=data.get("firmware", "klipper"),
        kinematics=data.get("kinematics", "corexy"),
        nozzle_diameter=float(data.get("nozzle_diameter", 0.4)),
        filament_diameter=float(data.get("filament_diameter", 1.75)),
        build_volume=data.get("build_volume", {}),
        motion=data.get("motion", {}),
        postprocess=data.get("postprocess", {}),
    )


def to_klipper_cfg(machine: Machine) -> str:
    """Emit stepper/extruder rotation_distance + gear_ratio lines for printer.cfg.

    This keeps the JSON config as the single source of truth: edit gear ratios
    here, regenerate the Klipper snippet.
    """
    lines = [f"# Generated from '{machine.name}' by Nelox Belt machine.py", ""]
    section_for = {"x": "stepper_x", "y": "stepper_y", "z": "stepper_z", "extruder": "extruder"}
    for key, axis in machine.axes.items():
        section = section_for.get(key, f"stepper_{key}")
        lines.append(f"[{section}]")
        lines.append(f"rotation_distance: {axis.rotation_distance:g}")
        if axis.gear_ratio != (1.0, 1.0):
            num, den = axis.gear_ratio
            lines.append(f"gear_ratio: {num:g}:{den:g}")
        lines.append(f"microsteps: {axis.microsteps}")
        lines.append(f"full_steps_per_rotation: {axis.full_steps_per_rotation}")
        lines.append(f"# effective rotation_distance/motor-rev: {axis.effective_rotation_distance:.4g}"
                     f", steps/mm: {axis.steps_per_mm:.2f}")
        lines.append("")
    return "\n".join(lines)


def _summary(machine: Machine) -> str:
    out = [
        f"{machine.name}  ({machine.firmware}, {machine.kinematics})",
        f"  gantry angle      : {machine.gantry_angle_deg} deg",
        f"  nozzle / filament : {machine.nozzle_diameter} / {machine.filament_diameter} mm",
        f"  post-process      : scale_z={machine.scale_z}, scale_feedrate={machine.scale_feedrate}",
        "  axes:",
    ]
    for key, a in machine.axes.items():
        ratio = "1:1" if a.gear_ratio == (1.0, 1.0) else f"{a.gear_ratio[0]:g}:{a.gear_ratio[1]:g}"
        out.append(
            f"    {key:8s} rot_dist={a.rotation_distance:<6g} gear={ratio:<7s}"
            f" eff={a.effective_rotation_distance:<7.4g} steps/mm={a.steps_per_mm:.2f}"
            f"{'  [geared]' if a.geared else ''}"
        )
    return "\n".join(out)


def main(argv=None) -> int:
    p = argparse.ArgumentParser(prog="machine", description="Inspect a Nelox Belt machine config.")
    p.add_argument("config", help="path to a machine JSON file")
    p.add_argument("--print-cfg", action="store_true", help="emit a Klipper printer.cfg snippet")
    args = p.parse_args(argv)
    try:
        machine = load_machine(args.config)
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        print(f"machine: cannot load {args.config}: {exc}", file=sys.stderr)
        return 1
    print(to_klipper_cfg(machine) if args.print_cfg else _summary(machine))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
```

## SOURCE: machines/ideaformer_ir3v2.json

```json
{
  "name": "IdeaFormer IR3 V2",
  "firmware": "klipper",
  "kinematics": "corexy",
  "comment": "Belt/conveyor printer, 45-degree tilted gantry. Values verified against a working community printer.cfg (github.com/dborio/Ideaformer-IR3v2). Everything here is adjustable; gear_ratio is expressed as [numerator, denominator] like Klipper's gear_ratio (e.g. [50, 17]). The effective distance moved per MOTOR revolution is rotation_distance / (numerator/denominator). Caveat: confirm the y<->z motor assignment (machine Y = 45-degree gantry rail/lift, machine Z = conveyor belt feed) against your actual printer.cfg before long prints.",

  "gantry_angle_deg": 45.0,

  "build_volume": {
    "x": 250,
    "y": 354,
    "z": null,
    "infinite_axis": "z",
    "comment": "z is the belt-feed direction and is effectively infinite (we slice upright; height becomes belt length). y is the 45-degree gantry rail (lift); its 354 mm is the finite rail travel (~build height x sqrt(2)) and the stock soft limit. z is null = unbounded."
  },

  "nozzle_diameter": 0.4,
  "filament_diameter": 1.75,

  "motion": {
    "max_velocity": 400,
    "max_accel": 20000
  },

  "axes": {
    "x": {
      "rotation_distance": 40,
      "gear_ratio": [1, 1],
      "microsteps": 32,
      "full_steps_per_rotation": 200,
      "pulley_teeth": 20,
      "belt_pitch": 2.0,
      "geared": false,
      "note": "CoreXY A/B motor. 20T GT2 pulley, ungeared."
    },
    "y": {
      "rotation_distance": 40,
      "gear_ratio": [1, 1],
      "microsteps": 32,
      "full_steps_per_rotation": 200,
      "pulley_teeth": 20,
      "belt_pitch": 2.0,
      "geared": false,
      "is_belt_feed": false,
      "position_min": -5,
      "position_max": 354,
      "note": "45-degree gantry rail (lift) axis, CoreXY-driven. Ungeared 20T GT2. Finite travel ~354 mm = build height x sqrt(2). NOT the conveyor."
    },
    "z": {
      "rotation_distance": 3.7,
      "gear_ratio": [1, 1],
      "microsteps": 32,
      "full_steps_per_rotation": 200,
      "geared": true,
      "position_min": -5,
      "position_max": 99999,
      "note": "Conveyor / belt-feed axis (infinite, position_max 99999), gearbox-driven. This is the belt progression direction. The stock config folds the gearbox reduction into the effective rotation_distance of 3.7, so gear_ratio is left at 1:1. To model the gearbox explicitly, set gear_ratio to the box's reduction and set rotation_distance to the ungeared value (effective = rotation_distance / ratio must stay 3.7)."
    },
    "extruder": {
      "rotation_distance": 4.4,
      "gear_ratio": [1, 1],
      "microsteps": 32,
      "full_steps_per_rotation": 200,
      "type": "dual-gear direct drive (BMG-style)",
      "geared": true,
      "note": "Dual-gear direct extruder. The ~3:1 dual-gear reduction is already baked into the effective rotation_distance of 4.4, so gear_ratio stays 1:1. If you swap to expressing it explicitly, a common BMG ratio is [50, 17] (~2.94:1); then set rotation_distance so rotation_distance/(50/17) stays your calibrated value."
    }
  },

  "postprocess": {
    "belt_axis": "z",
    "scale_z": true,
    "scale_feedrate": true,
    "scale_extrusion": true,
    "begin_marker": "; nelox:begin",
    "end_marker": "; nelox:end",
    "comment": "Defaults for nelox_belt.py when run with --machine. belt_axis='z' because the IR3 V2 drives the conveyor as the (infinite) Z axis — see stepper_z position_max 99999 in the stock printer.cfg. scale_z=true because mainline Klipper has no belt kinematics; verify with a calibration cube before long prints."
  }
}
```
