"""Stage 6 - images of the toolpath: the full-resolution move-type PNG (Plot_code) and
the fast vector SVG preview."""
import math

import cv2
import numpy as np

from tqdm import tqdm

from .toolpath import line_reader

def draw_grid(params, variables, grid_spacing=1000, color=(220, 220, 220), thickness=1):
    # This function can be used to draw light grey grid lines on the images to show the size of the parts
    # This will automatically use a grid spacing of 1mm unless otherwise specified
    # https://stackoverflow.com/questions/44816682/drawing-grid-lines-across-the-image-using-opencv-python

    grid_spacing_pixels = grid_spacing / variables["scale"] * 100
    h, w, _ = params["Image"].shape

    cols = int(round(w / grid_spacing_pixels))
    rows = int(round(h / grid_spacing_pixels))
    dy = dx = int(round(grid_spacing_pixels))

    # draw vertical lines
    for x in np.linspace(start=dx, stop=w-dx, num=cols-1):
        x = int(round(x))
        cv2.line(params["Image"], (x, 0), (x, h), color=color, thickness=thickness)

    # draw horizontal lines
    for y in np.linspace(start=dy, stop=h-dy, num=rows-1):
        y = int(round(y))
        cv2.line(params["Image"], (0, y), (w, y), color=color, thickness=thickness)

    return params


def Plot_code(params, variables):

    # Build plate size:
    buildplate = [variables["Y_build"] * 100,variables["X_build"] * 100,3,]  # 10um, 10um, RGB i.e. 5000 x 5000 is 50mm x 50mm
    # Plot the external of the build plate
    # Create the image that will show the gcode, the size of the image will be the bed size of the printer at 1um
    if len(params["Image"]) == 0:
        params["Image"] = np.zeros(buildplate, dtype="uint8")  # Maximum of 150000, 150000, 3
        params["Image"][:] = variables["Background_colour"]  # Make the image have a white background
        # Draw in the background grid
        params = draw_grid(params, variables)
    # Set the filename for the image output
    params["Image_name"] = "Output/" + params["Filename_only"] + "/" + params["Filename_only"] + "_cv2_Image_output.png"
    # Match units
    variables["Current_X"] = variables["Current_X"] * variables["scale"] # to return to µm
    variables["Current_Y"] = variables["Current_Y"] * variables["scale"]

    # Make variable for error checking
    variables["segment"]  = 0

    variables["calc_only"] = 0
    # Stop showing errors whilst plotting
    variables["Display_radius_error"] = False
    variables["radius_error_output"] = False
    variables["edited_flag"] = False
    variables["radius_error_fix"] = False
    
    for line in tqdm(params["Unlooped_contents"], ncols = 100):
        # Loop through all the lines in the edited contents array
        params["Line"] = line
        params, variables = line_reader(params, variables)
        # if show_image == 2:
        #     # Visualization of the print posistion
        #     draw_circle(img, (current_x, current_y), int(0.1 * 100), G4_colour)
        #     # Define 4K image size square due to square buildplate
        #     dim = (resolution_x, resolution_y)
        #     # Re-size the image
        #     resized = cv2.resize(img, dim, interpolation=cv2.INTER_AREA)
        #     # Add the frame to the output buffer
        #     out.write(resized)
        #     cv2.imshow("", resized)
        #     # Wait to ensure that the frame has been written
        #     cv2.waitKey(1)
        #     # time.sleep(0.1)
    # if show_image == 2:
    #     # Release video output
    #     out.release()

    if variables["Display_image"] == True:
        r = 500.0 / params["Image"].shape[1]
        dim = (500, int(params["Image"].shape[0] * r))
        resized = cv2.resize(params["Image"], dim, interpolation=cv2.INTER_AREA)
        cv2.imshow("", resized)
        cv2.waitKey()
        cv2.imwrite(params["Image_name"], params["Image"])
    elif variables["Generate_output_image"] == True:
        cv2.imwrite(params["Image_name"], params["Image"])


def render_preview_svg(params, variables):
    # NCViewer-style toolpath preview, as a vector image. Unlike the raster output it has
    # no resolution ceiling: arcs are exact SVG arc commands and strokes use
    # vector-effect="non-scaling-stroke" so they stay hairline-thin at any zoom.
    # Two files are written:
    #   _preview.svg          - for opening in a browser / other tools
    #   _preview_segments.npz - the raw segment table the GUI draws from directly
    #                           (much faster than having Qt parse a large SVG)
    segs = params["Preview_segments"]
    if not segs:
        print("No toolpath segments recorded; skipping preview output.")
        return

    out_base = "Output/" + params["Filename_only"] + "/" + params["Filename_only"]

    # cv2 colours are BGR - convert so the preview matches the PNG output
    def to_rgb(colour):
        return (int(colour[2]), int(colour[1]), int(colour[0]))

    colours = {
        1: to_rgb(variables["G1_colour"]),
        2: to_rgb(variables["G2_colour"]),
        3: to_rgb(variables["G3_colour"]),
    }
    background = to_rgb(variables["Background_colour"])

    seg_array = np.asarray(segs, dtype=np.float64)
    np.savez(out_base + "_preview_segments.npz",
             segments=seg_array.astype(np.float32),
             colours=np.array([colours[1], colours[2], colours[3]], dtype=np.uint8),
             background=np.array(background, dtype=np.uint8),
             # Same grid as draw_grid() puts on the PNG: light grey lines every 1 mm (1000 µm)
             grid_colour=np.array(to_rgb((220, 220, 220)), dtype=np.uint8),
             grid_spacing=np.float64(1000.0),
             # Reduced pixel coords from generate_pixel_coords(): rows of
             # (x µm, y µm, speed mm/s, acceleration mm/s^2, line), plus the critical
             # translation speed (mm/s, 0 = unset) and the acceleration setting
             samples=params.get("Motion_samples", np.zeros((0, 5), dtype=np.float32)),
             cts=np.float64(variables.get("global_return_CTS", 0) / 60.0),
             acceleration=np.float64(variables["Acceleration_mm_s2"]),
             # Speed range the overlay colour bands were cut from (mm/s)
             speed_range=np.asarray(params.get("Motion_speed_range", (0.0, 0.0)), dtype=np.float64),
             # Jet contact points from the lag model: rows of (x µm, y µm, lag mm, line),
             # and the lag range its colour bands are cut from (mm)
             lag_samples=params.get("Lag_samples", np.zeros((0, 5), dtype=np.float32)),
             lag_range=np.asarray(params.get("Lag_range", (0.0, 0.0)), dtype=np.float64))

    min_x = float(min(seg_array[:, 1].min(), seg_array[:, 3].min()))
    max_x = float(max(seg_array[:, 1].max(), seg_array[:, 3].max()))
    min_y = float(min(seg_array[:, 2].min(), seg_array[:, 4].min()))
    max_y = float(max(seg_array[:, 2].max(), seg_array[:, 4].max()))
    margin = max(max_x - min_x, max_y - min_y, 1.0) * 0.02
    vb_x, vb_y = min_x - margin, min_y - margin
    vb_w, vb_h = max_x - min_x + 2 * margin, max_y - min_y + 2 * margin

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="{vb_x:.2f} {vb_y:.2f} {vb_w:.2f} {vb_h:.2f}">',
        f'<rect x="{vb_x:.2f}" y="{vb_y:.2f}" width="{vb_w:.2f}" height="{vb_h:.2f}" fill="rgb{background}"/>',
    ]

    # Faint background grid, starting at 1 mm and doubling until there are <= 40 lines
    spacing, max_lines = 1000.0, 40
    while vb_w / spacing > max_lines or vb_h / spacing > max_lines:
        spacing *= 2
    grid = []
    gx = math.floor(vb_x / spacing) * spacing
    while gx <= vb_x + vb_w:
        grid.append(f"M {gx:.2f} {vb_y:.2f} V {vb_y + vb_h:.2f}")
        gx += spacing
    gy = math.floor(vb_y / spacing) * spacing
    while gy <= vb_y + vb_h:
        grid.append(f"M {vb_x:.2f} {gy:.2f} H {vb_x + vb_w:.2f}")
        gy += spacing
    parts.append(f'<path d="{" ".join(grid)}" fill="none" stroke="rgba(0,0,0,0.08)" vector-effect="non-scaling-stroke"/>')

    # Segments are written in chunks of consecutive moves, one <path> per move type per
    # chunk, with continuous moves joined into a single sub-path. This keeps the element
    # count small (a <path> per line made large files unusable) while each <g> still
    # records which G-code lines it covers.
    chunk_size = 2000
    for start in range(0, len(segs), chunk_size):
        chunk = segs[start:start + chunk_size]
        paths = {1: [], 2: [], 3: []}
        last_end = {}
        for kind, x1, y1, x2, y2, cx, cy, sweep, _line, *_ in chunk:
            d = paths[kind]
            if last_end.get(kind) != (x1, y1):
                d.append(f"M{x1:.2f} {y1:.2f}")
            if kind == 1:
                d.append(f"L{x2:.2f} {y2:.2f}")
            else:
                radius = math.hypot(x1 - cx, y1 - cy)
                sweep_flag = 1 if sweep < 0 else 0  # SVG sweep 1 = clockwise on screen
                if abs(sweep) >= 360.0:
                    # SVG can't draw a full circle as one arc (same start and end) - use two halves
                    ox, oy = 2 * cx - x1, 2 * cy - y1
                    d.append(f"A{radius:.2f} {radius:.2f} 0 0 {sweep_flag} {ox:.2f} {oy:.2f}")
                    d.append(f"A{radius:.2f} {radius:.2f} 0 0 {sweep_flag} {x1:.2f} {y1:.2f}")
                else:
                    large_arc = 1 if abs(sweep) > 180.0 else 0
                    d.append(f"A{radius:.2f} {radius:.2f} 0 {large_arc} {sweep_flag} {x2:.2f} {y2:.2f}")
            last_end[kind] = (x2, y2)
        parts.append(f'<g id="chunk_{start // chunk_size}" data-first-line="{chunk[0][8]}" data-last-line="{chunk[-1][8]}">')
        for kind in (1, 2, 3):
            if paths[kind]:
                parts.append(f'<path d="{"".join(paths[kind])}" fill="none" stroke="rgb{colours[kind]}" vector-effect="non-scaling-stroke"/>')
        parts.append("</g>")

    parts.append("</svg>")

    out_path = out_base + "_preview.svg"
    with open(out_path, "w") as f:
        f.write("\n".join(parts))
    print("Preview vector saved:", out_path)
