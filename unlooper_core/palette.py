"""Colours shared by Unlooper.py's images and the GUI, and the indexed-PNG writer.

The speed / acceleration / jet-lag PNGs are saved as palette (indexed) PNGs where the pixel
value is the colour band, so the GUI can highlight a single band just by changing the
palette. Index 0 is the background, 1..N the bands, NOZZLE_INDEX the nozzle path drawn
under the jet path, GRID_INDEX the 1 mm grid.
"""
import numpy as np

# PrusaSlicer's 11-step legend (dark blue = slowest ... dark red = fastest)
PRUSA_COLOURS = [
    (11, 44, 122), (19, 89, 133), (28, 136, 145), (4, 214, 15), (170, 242, 0), (252, 249, 3),
    (245, 206, 10), (227, 136, 32), (209, 104, 48), (194, 82, 60), (148, 38, 22),
]
SPEED_BANDS = 32
# Acceleration view: decelerating / constant speed / accelerating
ACCEL_COLOURS = [(19, 89, 133), (150, 150, 150), (209, 104, 48)]
BACKGROUND = (255, 255, 255)
GRID_COLOUR = (220, 220, 220)
NOZZLE_COLOUR = (200, 200, 200)
GRID_INDEX = 254
NOZZLE_INDEX = 253


def band_colours(count=SPEED_BANDS):
    # PrusaSlicer's colours stretched to `count` bands (linear in RGB between its steps)
    anchors = np.asarray(PRUSA_COLOURS, dtype=np.float64)
    positions = np.linspace(0.0, len(anchors) - 1, count)
    low = np.floor(positions).astype(int)
    high = np.minimum(low + 1, len(anchors) - 1)
    frac = (positions - low)[:, None]
    colours = anchors[low] * (1 - frac) + anchors[high] * frac
    return [tuple(int(round(c)) for c in rgb) for rgb in colours]


SPEED_COLOURS = band_colours(SPEED_BANDS)


def band_of(values, v_min, v_max, bands=SPEED_BANDS):
    # 0-based band of each value on an even split of [v_min, v_max]
    width = max(v_max - v_min, 1e-9) / bands
    return np.clip(((np.asarray(values) - v_min) / width).astype(np.int64), 0, bands - 1)


def band_edges(v_min, v_max, bands=SPEED_BANDS):
    return np.linspace(v_min, v_max, bands + 1).tolist()


def save_indexed_png(path, index_image, colours, background=BACKGROUND):
    # colours[i] (RGB) is used for index i + 1
    from PIL import Image
    palette = np.zeros((256, 3), dtype=np.uint8)
    palette[:] = background
    palette[GRID_INDEX] = GRID_COLOUR
    palette[NOZZLE_INDEX] = NOZZLE_COLOUR
    for i, rgb in enumerate(colours):
        palette[i + 1] = rgb
    image = Image.fromarray(index_image, mode="P")
    image.putpalette(palette.ravel().tolist())
    image.save(path, compress_level=1)


def ramp(low_rgb, high_rgb, count=SPEED_BANDS):
    # `count` colours from low_rgb to high_rgb
    low, high = np.asarray(low_rgb, dtype=np.float64), np.asarray(high_rgb, dtype=np.float64)
    return [tuple(int(round(c)) for c in low + (high - low) * t) for t in np.linspace(0.0, 1.0, count)]


# Two-hue lag comparison: original jet in blues, compensated jet in reds (light = small lag)
BLUE_RAMP = ramp((158, 202, 225), (8, 48, 107))
RED_RAMP = ramp((252, 174, 145), (103, 0, 13))
