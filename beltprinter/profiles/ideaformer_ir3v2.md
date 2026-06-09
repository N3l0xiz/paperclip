# IdeaFormer IR3 V2 — OrcaSlicer profile guide (Nelox Belt)

The IR3 V2 is a 45° tilted-gantry conveyor belt printer running Klipper. Mainline
Klipper has no belt kinematics, so the slicer must emit G-code already expressed in
the sheared machine frame — that is exactly what `nelox_belt.py` does as a
post-processing step. This guide covers the OrcaSlicer settings to pair with it.

> OrcaSlicer has **no native belt mode**, so we configure it as a custom Klipper
> printer with a tall build volume and attach the Nelox Belt post-processor.

## 1. Create the printer (Printer Settings)

| Setting | Value | Why |
|---|---|---|
| G-code flavor | **Klipper** (or Marlin if you prefer) | IR3 V2 firmware |
| Bed shape | **250 × 250 mm** | belt width × usable depth per "slice window" |
| Max print height (Z) | **300+ mm** (as tall as your model needs) | we slice upright; height becomes belt length |
| Nozzle diameter | 0.4 mm (your nozzle) | — |
| Use relative E distances | optional | the post-processor handles G90/G91 + M82/M83 |
| **Arc fitting** | **OFF** (disable!) | a shear turns arcs into ellipses — G2/G3 can't be transformed |

Arc fitting **must** be off. If left on, the post-processor passes G2/G3 through
untransformed and prints a warning; geometry will be wrong.

## 2. Orientation & slicing workflow

1. Place the model **upright** on the virtual bed, oriented so the dimension you
   want to run *along the belt* points in **+Y** (away from you).
2. Slice normally. Orca produces ordinary flat-bed, horizontal-layer G-code.
3. The Nelox Belt post-processor shears it into belt geometry on export.

This v1 transform is geometrically exact for any model that fits within the gantry
height. True infinite-length printing (re-ordering into diagonal columns) is on the
roadmap — see the main README.

## 3. Attach the post-processor (Print Settings → Output options)

Set **Post-processing Scripts** to (one line, adjust the absolute path):

```
python3 /absolute/path/to/beltprinter/nelox_belt.py --machine /absolute/path/to/beltprinter/machines/ideaformer_ir3v2.json;
```

OrcaSlicer appends the G-code file path as the final argument automatically. Using
`--machine` pulls the 45° gantry angle, scaling and markers from the config, so
there's a single place to adjust the printer's parameters.

On Windows, point at your interpreter explicitly, e.g.:

```
"C:\Python311\python.exe" "C:\tools\beltprinter\nelox_belt.py" --machine "C:\tools\beltprinter\machines\ideaformer_ir3v2.json";
```

### Optional flags

- `--no-scale-z` — only if your IR3 V2 firmware compensates the 45° tilt itself
  (stock mainline Klipper does **not**; leave scaling **on** unless a test proves
  otherwise — see "Verify" below).
- `--no-scale-feedrate` — leave F values untouched.
- `--begin-marker "; nelox:begin"` / `--end-marker "; nelox:end"` — only transform
  between markers, so custom start/end G-code (homing, belt priming) stays in raw
  machine coordinates. Put the markers in your Machine start/end G-code.

## 4. Start / End G-code

Keep homing and priming **outside** the transformed region. A minimal pattern:

**Machine start G-code (end it with the begin marker):**
```
G28
G90
M82
; ... your purge line / bed prep in machine coordinates ...
; nelox:begin
```

**Machine end G-code (start it with the end marker):**
```
; nelox:end
G91
G1 E-3 F1800
G90
M104 S0
M84
; (belt models usually keep running the belt to eject — add your eject macro)
```

Then run the post-processor with `--begin-marker "; nelox:begin" --end-marker "; nelox:end"`.

## 5. Verify before a long print (important)

The one firmware-dependent unknown is the **Z-scaling convention**. Confirm it
cheaply before committing to a long print:

1. Slice a small calibration cube (e.g. 20 mm) upright and export through Nelox Belt.
2. Open the output and check the **first-layer** move: at real z = 0.2 mm with a 45°
   gantry you should see `Z0.2828` (= 0.2 × √2) and the layer shifted by `Y+0.2`.
3. Print it. If the cube's height is off by a factor of √2 (≈1.41), your firmware is
   already compensating the tilt — re-run with `--no-scale-z`.
4. If the 45° lean angle looks wrong, double-check the model orientation (length
   along +Y) and that arc fitting is disabled.

If your IR3 V2 was supplied with a known-good belt G-code from the stock slicer,
diff a simple shape against the Nelox Belt output to confirm the convention.
