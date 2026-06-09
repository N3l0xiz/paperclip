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
