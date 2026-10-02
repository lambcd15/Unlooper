# Unlooper

This code is for unlooping gcode and rendering an image

## Table of Contents

* [General Info](#general-info)
* [Functions](#function)
* [Lag compensation, fibre diameter and tubes](#lag-compensation-fibre-diameter-and-tubes)
* [Setup](#setup)

## General Info

This software is designed to unloop M98 gcode into linear code that can be read by any gcode system. This system is primarily optimised for 2D x and y co-ordinates with empahsis on G1, G2 and G3 commands using the G90 and G91 co-ordinate systems. This software can be used either by command line or by running in python editor

## Function

Python editor:
To run using a python editor you can edit either of these variables
```
# Commands you can edit
# Setting to 0 will unloop and generate an image
# Setting to 1 will unloop only and not generate an image
variables["unloop_only"] = 1

# Everything must contain forward slashes only
file_name = "TXT Files/DO_4 x10.txt"
```

Change the filename to the path from the current folder to the gcode file to be read preferably the file to be read and the unlooper are in the same folder or the gcode file is placed inside a sub folder with the unlooper being in the main folder. An output folder will be created that will contain a folder with the same name as the gcode file that was unlooped. This folder will contain the unlooped code and image generated (if requested)


```
Random Folder
  ├── unlooper.py
  ├── TXT_FILES
  │   ├── filename.gcode
  ├── Output
  │   ├── filename
  |   │   ├── filename_unlooped.txt
```

Command line
This program can also be run from the command line (or through the GUI, `python Unlooper_gui.py`) using
```
python Unlooper.py "filename" <unloop_only 0|1> [feedrate mm/min] [density] [fibre diameter um]
                   [render precise|preview|both|none] [acceleration mm/s2] [junction deviation mm]
                   [skip pixel coords 1|0] [jerk mm/s] [lag prediction 1|0] [CTS mm/min]
                   [write lag-format files 1|0] [lag compensation none|overshoot|pointwise|slowdown|iterative|hybrid|hybrid_constant]
                   [rapid mm/min] [overshoot scale] [slow-down ratio of CTS] [iterations]
                   [pointwise point spacing um, 0 = adaptive] [hybrid corner tolerance um]
                   [hybrid: fibre diameter limit %, 0 = none] [mandrel diameter mm, 0 = flat]
                   [fibre diameter tolerance % for the diameter view]
                   [overshoot / pointwise: time-preserving feeds 1|0]
                   [overshoot / pointwise: swing blend radius um, -1 = automatic, 0 = off]
```
Only the first two arguments are required; 0 for an override means "use the file's own value".

## Code layout

`Unlooper.py` holds the settings, the command line and the order the stages run in. The
stages are in `unlooper_core/`:

| Module | What it does |
| --- | --- |
| `gcode_reader.py` | read the file, strip comments, pull out the print parameters, unloop O / M98 / M99 sub-programs |
| `toolpath.py` | turn each command into tool movement: position tracking, G1 lines, G2 / G3 arcs, distance, time, material |
| `motion_planner.py` | acceleration-limited moves with junction deviation and / or classic jerk corner speeds |
| `pixel_coords.py` | nozzle positions every 1 ms along the planned motion; speed / acceleration PNGs |
| `corner_path.py` | the same with corners rounded as the machine runs them (constant-velocity mode) |
| `lag_model.py` | jet lag prediction (Ievgenii's python_lag model, compiled with numba) along the corner path |
| `lag_compensation.py` | writes `_Lag_compensated.txt` (ISBF overshoot arcs, point-by-point ISBF, corner slow-down, iterative model-driven correction or hybrid), runs it and draws the before / after image |
| `hybrid.py` | the hybrid lag compensation: the nozzle leads the jet along the path by the lag (the lag model run backwards), sheds lag before sharp corners, fitted to the planner and the simulated jet; or (`hybrid_constant`) keeps the jet speed, lag and fibre diameter constant and smooths the path to the tightest turn the jet can make; G1 moves only |
| `mandrel.py` | tubular printing: A (degrees) read as the distance round the mandrel, flat code wrapped onto a mandrel (`python -m unlooper_core.mandrel wrap <in> <out> <diameter mm>`), paths drawn on the tube |
| `isbf.py` | the ISBF lag compensation (Gcode_processing.py `vector_angle`), ported so it writes the same G-code |
| `path_reference.py` | the programmed path as a polyline; how far the jet lands from it |
| `scaffold_outputs.py` | runs the stages above and totals the results |
| `rendering.py` | move-type PNG and vector (SVG) preview |
| `palette.py`, `common.py` | shared colours and helpers |

Outputs in `Output/<name>/` besides the unlooped code and images: `_pixel_cords.csv`,
`_corner_pixel_cords.csv` (same layout, for the lag model) and their `_motion.csv` companions,
`_lag.csv` (jet contact point X, Y and lag in mm), `_lag.png` and the `_legend.json` files.
The lag calibration data is `lag_data/Lag_1.2b_fM.csv`. A lag-compensated run's outputs go
to `Output/<name>_Lag_compensated/`, and `Output/<name>/<name>_lag_compensation.png` shows the
programmed path in black, the jet before compensation in grey and after it in green (the ISBF
code's colours). The GUI's jet lag view can show the nozzle and jet paths of both runs as
separate layers, coloured by lag or ("Colour: lag compensation") in those same colours.

## Lag compensation, fibre diameter and tubes

* **Hybrid lag compensation** (`hybrid`): the nozzle leads the jet by exactly its lag along the
  path, turning sharp corners with the arc of Lamb et al. (2026). G1 moves only.
* **Fibre diameter**: estimated from the nozzle's speed over the collector, d / d0 = sqrt(v0 / v):
  the fibre only thickens where the machine runs slower than the programmed feed (braking into a
  sharp kink), which is how the ISBF code prints at a constant diameter in Lamb et al. (2026).
  Reported for every run ("Fibre diameter ... within +5% for ...% of the print"); the GUI's
  "Colour: fibre diameter" view colours the jet by it, banded around the tolerance.
  (`Diameter_basis = "jet"` in `Unlooper.py` estimates it from the jet's contact speed instead.)
* **Point-by-point, adaptive** (`pointwise`, point spacing 0): ISBF's overshoot and swing at every
  sharp corner, each overshoot solved on the lag model with the machine's own motion, then tuned
  by the correction passes. On top of that:
  * *smooth swings* - the kink where an overshoot line meets its swing arc, and the arc the next
    line, is replaced by one small fillet arc the machine can take at the feed (radius
    v² / (0.85 × acceleration), about 0.12 mm at 612 mm/min and 1000 mm/s²; argument 25), so the
    nozzle doesn't brake and the fibre keeps its diameter;
  * *steered curves* - turns under 30° (a curve written as short lines, e.g. a sinusoid) are not
    swung round: the nozzle carries on at the feed and moves across onto each new line as fast
    as the machine allows (faster than the feed, never slower). How far ahead it looks is found
    on the file. The points are thinned to within 3 µm before writing.
* **ISBF corrections** (`Lag_comp_isbf_fixes`, on): every swing onto an arc at the rapid feed (two
  of the four cases had no feed), no divide by zero at a zero-length line after an arc, and every
  shortened arc written as a true arc (start and end at the same radius, which GRBL-type
  controllers check).
* **Holding the diameter**: give the hybrid a fibre diameter limit (argument 21, or tick "Hybrid
  holds the fibre within it" in the GUI). It tries the paper's corner arc, rounding the path,
  constant speed and the program unchanged, simulates each, and writes the most accurate one
  whose fibre stays within the limit.
* **Export for printing**: the GUI's "Export compensated G-code…" saves
  `<name>_Lag_compensated.txt` wherever you choose, with `%` or `;` comments or none
  (`lag_compensation.export_gcode`).
* **Tubes**: set the mandrel diameter (argument 22). A (degrees) is read as the distance round
  the tube and a rotation on its own takes F in degrees/min; the compensated file is written with
  A. `<name>_mandrel.png` draws the path on the mandrel in 3D. For a shaped mandrel (the nozzle
  following the surface in Z, e.g. an ellipse end with G18 arcs) the radius at each point is the
  mandrel's radius plus Z, so the diameter set is the one where the program starts (Z = 0).
  Flat code can be wrapped onto a tube with `python -m unlooper_core.mandrel wrap <in> <out> <D>`.

## Setup

Download the file and place in the chosen folder
Make sure to download and run the requriements.txt file (navigate to the folder first in cmd prompt)
```
pip install -r requirements.txt
```

## Example image from output

![Custom_complex_scaffold_SR1 0_Raw_image_output](https://github.com/user-attachments/assets/f256c392-e3ee-4ad2-af75-4a53d4f6fc08)

