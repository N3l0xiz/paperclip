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
(normal horizontal layers) and then apply the transform that maps a real-world point
`(x, y, z)` — `z` = true height, `y` = along the belt — into machine coordinates:

```
X' = x
Y' = y + z / tan(θ)      # higher layers are pushed forward along the belt
Z' = z / sin(θ)          # the tilted rail travels farther than the height
```

For **θ = 45°**: `Y' = y + z`, `Z' = z·√2`.

The transform is linear in `(y, z)` with no constant term, so it applies identically
to absolute coordinates (G90) and relative deltas (G91). Each horizontal layer ends
up sheared forward along the belt proportional to its height — producing the 45°
belt geometry, with lower (earlier) layers leading and higher layers trailing, which
matches how the belt feeds finished work away.

## Quick start

1. **Set up the printer** in OrcaSlicer following
   [`profiles/ideaformer_ir3v2.md`](profiles/ideaformer_ir3v2.md) (Klipper flavor,
   250×250 bed, tall Z, **arc fitting OFF**).
2. **Attach the post-processor** under *Print Settings → Output options →
   Post-processing Scripts*:
   ```
   python3 /absolute/path/to/nelox-belt/nelox_belt.py --angle 45;
   ```
   Orca appends the sliced file path automatically.
3. **Orient** the model upright with its belt-length dimension pointing **+Y**, then
   slice and export. The output is belt-ready G-code.

## Command-line usage

```bash
# In place (how Orca calls it):
python3 nelox_belt.py path/to/sliced.gcode

# Explicit output, custom angle:
python3 nelox_belt.py --angle 45 -o out.gcode in.gcode

# Analyze only, write nothing:
python3 nelox_belt.py --dry-run in.gcode
```

| Flag | Purpose |
|---|---|
| `-a, --angle` | gantry angle in degrees (default 45) |
| `-o, --output` | write here instead of editing in place |
| `--no-scale-z` | skip Z scaling (only if firmware compensates the tilt — verify first) |
| `--no-scale-feedrate` | leave F values untouched |
| `--begin-marker` / `--end-marker` | only transform between marker comments, so start/end G-code stays in machine coordinates |
| `--decimals` | coordinate precision (default 4) |
| `--dry-run` | report stats and warnings, write nothing |

See [`examples/sample_upright.gcode`](examples/sample_upright.gcode) for an input you
can run through the script, and `examples/sample_belt.gcode` for the result.

## What it handles

- G0/G1 linear moves (absolute G90 and relative G91), with G92 position resets and
  M82/M83 / G90/G91 mode tracking.
- Emits the required Y-shift even on Z-only moves (where the original line had no Y).
- Leaves extrusion `E` untouched (the deposited volume is unchanged) and optionally
  scales feedrate `F` so real print speed is preserved.
- Passes start/end G-code through untransformed via begin/end markers.
- Detects G2/G3 **arc moves** and warns — a shear can't be re-expressed as an arc, so
  disable arc fitting in the slicer.

## Limitations & roadmap

This is a **v1 geometric transform**. It is exact for models that fit within the
gantry height. Known limitations:

- **No infinite-length re-ordering yet.** True endless printing needs the slice to be
  re-ordered into diagonal front-to-back columns (the BlackBelt approach) rather than
  full horizontal layers. That's the headline next step.
- **Firmware Z convention** (`scale_z`) should be confirmed with a calibration cube —
  see the "Verify" section in the profile guide.
- Feedrate scaling is per-move linear; arc moves are not supported (disable arc fitting).

Roadmap: (1) diagonal-column re-ordering for true infinite Z, (2) an importable Orca
machine profile, (3) belt-wall / first-layer compensation tuning.

## Development

```bash
python3 nelox-belt/tests/test_nelox_belt.py        # run the unit tests
```

The transform math lives in the `Transform` class and the G-code stream handling in
`transform_gcode()` — both are pure/iterable-based and unit-tested.

---

Built by **Nelox**.
