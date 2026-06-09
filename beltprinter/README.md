# Nelox Belt

**Belt-printer post-processor for OrcaSlicer / PrusaSlicer.**
Turns ordinary upright-sliced G-code into belt-printer G-code for tilted-gantry
conveyor printers such as the **IdeaFormer IR3 V2** (45° gantry, Klipper).

OrcaSlicer has no native belt mode and there are several open feature requests for
it ([#6885](https://github.com/SoftFever/OrcaSlicer/issues/6885),
[#11344](https://github.com/OrcaSlicer/OrcaSlicer/issues/11344),
[#2628](https://github.com/OrcaSlicer/OrcaSlicer/issues/2628)). Nelox Belt fills the
gap as a small, dependency-free post-processing script you attach to Orca's existing
post-processing hook — no fork, no rebuild.

## Why a post-processor

A belt printer's gantry is tilted by the **gantry angle** θ (45° on the IR3 V2). On
mainline Klipper there is no belt kinematics, so the firmware expects G-code already
expressed in the *sheared* machine frame. We slice the model **upright** in Orca
(normal horizontal layers) and derive two coordinates from each point `(x, y, z)`
(`z` = true height, `y` = along the belt):

```
shift = y + z / tan(θ)    # belt progression: higher layers move further along the belt
lift  = z / sin(θ)        # gantry-rail travel for the model height
```

Which **machine axis** carries each depends on the printer's convention, set by
`belt_axis`:

```
belt_axis = "z"  (IdeaFormer IR3 V2 "infinite Z", the default)
    X' = x      Y' = lift      Z' = shift
belt_axis = "y"  (CR-30 style)
    X' = x      Y' = shift     Z' = lift
```

The IR3 V2 drives the **conveyor as the Z axis** — confirmed by its marketing
("Infinite Z-axis"), by its stock Klipper config (`stepper_z` `position_max: 99999`),
and by hardware-validated belt slicers. So Nelox Belt defaults to `belt_axis="z"`;
flip to `"y"` for CR-30-family machines.

The transform is linear in `(y, z)` with no constant term, so it applies identically
to absolute coordinates (G90) and relative deltas (G91). Each horizontal layer ends
up sheared along the belt proportional to its height, with lower (earlier) layers
leading — which matches how the belt feeds finished work away.

## Quick start

1. **Set up the printer** in OrcaSlicer following
   [`profiles/ideaformer_ir3v2.md`](profiles/ideaformer_ir3v2.md) (Klipper flavor,
   250×250 bed, tall Z, **arc fitting OFF**).
2. **Attach the post-processor** under *Print Settings → Output options →
   Post-processing Scripts*:
   ```
   python3 /absolute/path/to/beltprinter/nelox_belt.py --machine /absolute/path/to/beltprinter/machines/ideaformer_ir3v2.json;
   ```
   Orca appends the sliced file path automatically. `--machine` pulls the gantry
   angle, Z/feedrate scaling and start/end markers straight from the config.
3. **Orient** the model upright with its belt-length dimension pointing **+Y**, then
   slice and export. The output is belt-ready G-code.

## Machine config (single source of truth)

All printer mechanics live in an adjustable JSON file — see
[`machines/ideaformer_ir3v2.json`](machines/ideaformer_ir3v2.json). Values are
verified against a working community Klipper config. Every field is editable, and
gear ratios are first-class:

| Axis | rotation_distance | gear_ratio | notes |
|---|---|---|---|
| X / Y (CoreXY) | 40 | 1:1 | 20T GT2, ungeared |
| Y (45° gantry rail) | 40 | 1:1 | rail/lift, finite travel ~354 mm = build height × √2 |
| Z belt feed | 3.7 | 1:1* | conveyor (infinite), gearbox-driven (reduction baked into rotation_distance) |
| Extruder | 4.4 | 1:1* | dual-gear direct drive (~3:1 baked in) |

\* The stock firmware folds the gearbox/dual-gear reduction into the effective
`rotation_distance`. To model a gear box explicitly, set `gear_ratio` to e.g.
`[50, 17]` and the ungeared `rotation_distance`; the effective value
(`rotation_distance ÷ (num/den)`) is what reaches the axis.

```bash
python3 machine.py machines/ideaformer_ir3v2.json              # human-readable summary
python3 machine.py machines/ideaformer_ir3v2.json --print-cfg  # regenerate Klipper stepper lines
```

So you edit gear ratios in one place and regenerate the `printer.cfg` stepper
sections — no drift between the slicer config and the firmware.

## Command-line usage

```bash
# Use the machine config (recommended — how Orca calls it):
python3 nelox_belt.py --machine machines/ideaformer_ir3v2.json path/to/sliced.gcode

# Or specify everything by hand:
python3 nelox_belt.py --angle 45 --belt-axis z -o out.gcode in.gcode

# Analyze only, write nothing:
python3 nelox_belt.py --dry-run in.gcode
```

| Flag | Purpose |
|---|---|
| `-m, --machine` | load a machine JSON for angle + post-process defaults (CLI flags override) |
| `-a, --angle` | gantry angle in degrees (default 45) |
| `--belt-axis` | which machine axis is the belt: `z` (IR3 V2, default) or `y` (CR-30) |
| `-o, --output` | write here instead of editing in place |
| `--no-scale-z` | skip rail (lift) scaling (only if firmware compensates the tilt — verify first) |
| `--no-scale-feedrate` | leave F values untouched |
| `--max-velocity` | clamp belt/rail feedrate to this many mm/s (default: from `--machine`) |
| `--begin-marker` / `--end-marker` | only transform between marker comments, so start/end G-code stays in machine coordinates |
| `--decimals` | coordinate precision (default 4) |
| `--dry-run` | report stats and warnings, write nothing |

See [`examples/sample_upright.gcode`](examples/sample_upright.gcode) for an input you
can run through the script, and `examples/sample_belt.gcode` for the result.

## What it handles

- G0/G1 linear moves (absolute G90 and relative G91), and tolerant parsing: no-space
  commands (`G1X10`), lowercase (`g1`), signed/`+`/scientific-notation coordinates.
- **G92 position resets** are rewritten into machine coordinates so the firmware's
  frame can't desync from the model frame (a silent belt-motion error otherwise).
- Emits the dependent machine axes even on Z-only moves (both Y and Z can depend on
  model z).
- Leaves extrusion `E` untouched (the deposited volume is unchanged) and handles the
  **modal feedrate** correctly: F is re-emitted (and scaled) on F-less moves so the
  rail/belt axis is never run at the wrong speed, clamped to `max_velocity`.
- Passes start/end G-code through untransformed via begin/end markers.
- Warns on **below-belt moves** (model z < 0) and on G2/G3 **arc moves** (a shear can't
  be re-expressed as an arc — disable arc fitting in the slicer).

## Limitations & roadmap

This is a **geometric transform**. It is exact for models that fit within the gantry
height. Known limitations:

- **No infinite-length re-ordering.** True endless printing needs the slice to be
  re-ordered into diagonal front-to-back (keel-first) columns inside the slicer engine
  — that is fundamentally a slicer-engine job, not something a post-processor on
  already-ordered horizontal-layer G-code can do. For genuinely infinite prints, a
  native belt fork (e.g. ShidaoSlicer) is the right tool; Nelox Belt targets finite,
  within-gantry-height prints on your existing single OrcaSlicer install.
- **Firmware conventions** (`belt_axis`, `scale_z`) should be confirmed with a
  calibration cube — see the profile guide.
- Arc moves are not supported (disable arc fitting).

Roadmap: (1) a configurable gantry-height check (warn/abort when a print exceeds it),
(2) an importable Orca machine profile, (3) belt-wall / first-layer compensation.

## Development

```bash
# from the repo root:
python3 beltprinter/tests/test_nelox_belt.py   # G-code transform tests
python3 beltprinter/tests/test_machine.py      # machine-config tests
```

The transform math lives in the `Transform` class, the G-code stream handling in
`transform_gcode()` (both in `nelox_belt.py`), and the machine model in `machine.py` —
all pure/iterable-based and unit-tested.

---

Built by **Nelox**.
