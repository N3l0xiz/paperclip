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
VERSION = "0.1.0"

# Matches a G-code word like X12.34, Y-5, Z0.2, E1.5, F1800 (letter + signed number).
_WORD = re.compile(r"([A-Za-z])\s*(-?\d*\.?\d+)")


@dataclass
class Transform:
    """Geometric belt transform for a tilted-gantry printer."""

    angle_deg: float = 45.0
    scale_z: bool = True  # set False if the firmware compensates the gantry tilt itself

    def __post_init__(self) -> None:
        if not 0 < self.angle_deg < 90:
            raise ValueError(f"gantry angle must be between 0 and 90 degrees, got {self.angle_deg}")
        theta = math.radians(self.angle_deg)
        self._cot = 1.0 / math.tan(theta)          # y-shift per unit height
        self._inv_sin = 1.0 / math.sin(theta)      # z-scaling along the rail

    def y(self, y: float, z: float) -> float:
        return y + z * self._cot

    def z(self, z: float) -> float:
        return z * self._inv_sin if self.scale_z else z

    def point(self, x: float, y: float, z: float) -> tuple[float, float, float]:
        return x, self.y(y, z), self.z(z)


@dataclass
class State:
    """Tracks the modal G-code machine state needed to transform moves correctly."""

    # Real-frame position as the slicer intended it (pre-transform).
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    absolute_xyz: bool = True  # G90 (default) vs G91
    transforming: bool = False  # gated by the begin/end markers


@dataclass
class Stats:
    moves_transformed: int = 0
    arc_moves: int = 0
    lines: int = 0
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
    decimals: int = 4,
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

        upper = stripped.upper()

        # Track positioning mode regardless of whether we're transforming yet.
        if upper.startswith("G90"):
            state.absolute_xyz = True
            yield raw
            continue
        if upper.startswith("G91"):
            state.absolute_xyz = False
            yield raw
            continue
        if upper.startswith("G92"):
            # Position reset: keep the slicer's intent in the real frame.
            for letter, value in _WORD.findall(stripped):
                L = letter.upper()
                if L == "X":
                    state.x = float(value)
                elif L == "Y":
                    state.y = float(value)
                elif L == "Z":
                    state.z = float(value)
            yield raw
            continue

        is_linear = upper.startswith(("G0 ", "G1 ", "G0\t", "G1\t")) or upper in ("G0", "G1")
        is_arc = upper.startswith(("G2 ", "G3 ", "G2\t", "G3\t")) or upper in ("G2", "G3")

        if is_arc and state.transforming:
            # A shear turns a circle into an ellipse — it can't be re-expressed as
            # G2/G3. Pass it through and warn so the user disables arc fitting.
            stats.arc_moves += 1
            if "arc-fitting" not in " ".join(stats.warnings):
                stats.warnings.append(
                    "arc-fitting move (G2/G3) found and left untransformed — "
                    "disable 'Arc fitting' in the slicer for correct belt geometry"
                )
            yield raw
            continue

        if not (is_linear and state.transforming):
            yield raw
            continue

        # --- Transform a linear move -------------------------------------------------
        # Split off the leading command word (G0/G1) so it is not parsed as a coord.
        gword, _, rest = stripped.partition(" ")
        words = _WORD.findall(rest)
        present = {letter.upper(): value for letter, value in words}  # raw value strings
        if not any(L in present for L in ("X", "Y", "Z")):
            # Pure E/F move (e.g. retraction) — nothing geometric to do.
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

        # Feedrate scaling: machine path length / real path length for this move.
        f_scale = 1.0
        if scale_feedrate and "F" in present:
            real_d = math.dist(prev, new)
            if real_d > 1e-9:
                mp = transform.point(*prev)
                mn = transform.point(*new)
                f_scale = math.dist(mp, mn) / real_d

        # Rebuild in canonical order: command, X, Y, Z, E, extras, F.
        # Y must be emitted whenever Z is present (its value depends on z), even if
        # the original line had no Y word.
        parts = [gword]
        emit_y = ("Y" in present) or ("Z" in present)
        if state.absolute_xyz:
            if "X" in present:
                parts.append("X" + _fmt(state.x, decimals))
            if emit_y:
                parts.append("Y" + _fmt(transform.y(state.y, state.z), decimals))
            if "Z" in present:
                parts.append("Z" + _fmt(transform.z(state.z), decimals))
        else:
            if "X" in present:
                parts.append("X" + _fmt(dx, decimals))
            if emit_y:
                parts.append("Y" + _fmt(transform.y(dy, dz), decimals))  # delta transform
            if "Z" in present:
                parts.append("Z" + _fmt(transform.z(dz), decimals))

        if "E" in present:
            parts.append("E" + present["E"])  # extrusion unchanged, original text kept
        for letter, value in words:  # passthrough for any non-geometry words (e.g. S)
            if letter.upper() not in ("X", "Y", "Z", "E", "F"):
                parts.append(letter + value)
        if "F" in present:
            f_val = float(present["F"]) * f_scale
            parts.append("F" + (_fmt(f_val, 1) if f_scale != 1.0 else present["F"]))

        rebuilt = " ".join(parts)
        if comment:
            rebuilt += " " + comment
        stats.moves_transformed += 1
        yield rebuilt + "\n"

    transform_gcode.last_stats = stats  # type: ignore[attr-defined]


def _header(transform: Transform, scale_feedrate: bool) -> str:
    return (
        f"; Processed by {BRAND} v{VERSION}\n"
        f";   gantry angle = {transform.angle_deg} deg, "
        f"scale_z = {transform.scale_z}, scale_feedrate = {scale_feedrate}\n"
        f";   transform: X'=x  Y'=y+z*cot(a)  Z'=z/sin(a)\n"
    )


def main(argv=None) -> int:
    p = argparse.ArgumentParser(
        prog="nelox_belt",
        description=f"{BRAND} — belt-printer G-code post-processor (IdeaFormer IR3 V2 and similar).",
    )
    p.add_argument("input", help="input G-code file (edited in place when no -o is given)")
    p.add_argument("-o", "--output", help="write to this file instead of editing in place")
    p.add_argument("-a", "--angle", type=float, default=45.0, help="gantry angle in degrees (default: 45)")
    p.add_argument(
        "--no-scale-z",
        action="store_true",
        help="do not scale Z by 1/sin(angle) — use only if firmware compensates the tilt",
    )
    p.add_argument(
        "--no-scale-feedrate",
        action="store_true",
        help="leave F (feedrate) values untouched",
    )
    p.add_argument("--begin-marker", help="only transform after a line containing this comment")
    p.add_argument("--end-marker", help="stop transforming at a line containing this comment")
    p.add_argument("--decimals", type=int, default=4, help="coordinate precision (default: 4)")
    p.add_argument("--dry-run", action="store_true", help="analyze and report, but write nothing")
    p.add_argument("--version", action="version", version=f"{BRAND} {VERSION}")
    args = p.parse_args(argv)

    try:
        transform = Transform(angle_deg=args.angle, scale_z=not args.no_scale_z)
    except ValueError as exc:
        print(f"{BRAND}: {exc}", file=sys.stderr)
        return 2

    try:
        with open(args.input, "r", encoding="utf-8", errors="surrogateescape") as fh:
            source = fh.readlines()
    except OSError as exc:
        print(f"{BRAND}: cannot read {args.input}: {exc}", file=sys.stderr)
        return 1

    scale_feedrate = not args.no_scale_feedrate
    result = list(
        transform_gcode(
            source,
            transform,
            begin_marker=args.begin_marker,
            end_marker=args.end_marker,
            scale_feedrate=scale_feedrate,
            decimals=args.decimals,
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

    out_path = args.output or args.input
    payload = _header(transform, scale_feedrate) + "".join(result)
    try:
        with open(out_path, "w", encoding="utf-8", errors="surrogateescape") as fh:
            fh.write(payload)
    except OSError as exc:
        print(f"{BRAND}: cannot write {out_path}: {exc}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
