"""Stage 4 - pixel coords: sample the planned motion every scatter_resolution seconds
(the lag model's input), colour them by speed / acceleration and draw them as PNGs.

Pixel coords are the positions of the nozzle at fixed time steps (1 ms by default) along
the planned (acceleration / corner-speed limited) motion. Like the original doline() /
docircle() pixel coords they sit on one continuous time grid, so the remainder of one
command carries into the next, but the spacing follows the real speed instead of the
programmed feed and G4 dwells appear as stationary points.

The building blocks here (build_timeline, sample_timeline, LagFormatWriter) are shared with
corner_path.py, which samples the corner-rounded path the same way.
"""
import json
import math
from types import SimpleNamespace

import cv2
import numpy as np

from .palette import (SPEED_BANDS, SPEED_COLOURS, ACCEL_COLOURS, GRID_INDEX, NOZZLE_INDEX,
                      band_of, band_edges, save_indexed_png)

POINTS_PER_BLOCK = 1_000_000  # pixel coords are generated in blocks to keep memory flat
PNG_SHIFT = 4  # sub-pixel precision for cv2 drawing (1/16 pixel)


def output_base(params):
    return "Output/" + params["Filename_only"] + "/" + params["Filename_only"]


# --- timeline -----------------------------------------------------------------------------

def build_timeline(segments, planned, dwells, accel):
    # One entry per planned move (and per G4 dwell, as a stationary entry) with its
    # geometry, speed profile, start / end time and the G-code command that owns it.
    plan = np.asarray(planned, dtype=np.float64)
    seg_index = plan[:, 0].astype(np.int64)
    length, v0, peak, v1, d_acc, d_dec = (plan[:, k] for k in range(1, 7))
    t_acc = (peak - v0) / accel
    t_cruise = np.where(peak > 0, np.maximum(length - d_acc - d_dec, 0.0) / np.maximum(peak, 1e-12), 0.0)
    t_dec = (peak - v1) / accel
    geometry = np.asarray([segments[i][:9] for i in seg_index], dtype=np.float64)
    owner = np.asarray([segments[i][10] for i in seg_index], dtype=np.int64)  # G-code command of each move

    if len(dwells):
        dwell = np.asarray(dwells, dtype=np.float64)  # (segments before, seconds, commands before, line)
        where = np.searchsorted(seg_index, dwell[:, 0], side="left")
        # A dwell sits at the start of the next move, or the end of the last one
        nxt = np.minimum(where, len(seg_index) - 1)
        at_end = where >= len(seg_index)
        px = np.where(at_end, geometry[nxt, 3], geometry[nxt, 1])
        py = np.where(at_end, geometry[nxt, 4], geometry[nxt, 2])
        dwell_geometry = np.column_stack((np.ones(len(dwell)), px, py, px, py, px, py, np.zeros(len(dwell)), dwell[:, 3]))
        zeros = np.zeros(len(dwell))
        length, v0, peak, v1, d_acc, d_dec, t_acc, t_dec = (np.insert(a, where, zeros) for a in (length, v0, peak, v1, d_acc, d_dec, t_acc, t_dec))
        t_cruise = np.insert(t_cruise, where, dwell[:, 1])
        geometry = np.insert(geometry, where, dwell_geometry, axis=0)
        # The stationary points belong to the command before the dwell
        owner = np.insert(owner, where, np.maximum(dwell[:, 2].astype(np.int64) - 1, 0))

    duration = t_acc + t_cruise + t_dec
    t_end = np.cumsum(duration)
    kind, x1, y1, x2, y2, cx, cy, sweep, line = (geometry[:, k] for k in range(9))
    return SimpleNamespace(length=length, v0=v0, peak=peak, v1=v1, d_acc=d_acc, d_dec=d_dec,
                           t_acc=t_acc, t_cruise=t_cruise, t_dec=t_dec, duration=duration,
                           t_start=t_end - duration, t_end=t_end, owner=owner,
                           kind=kind, x1=x1, y1=y1, x2=x2, y2=y2, cx=cx, cy=cy, sweep=sweep, line=line)


def sample_timeline(tl, accel, dt, scale, block=POINTS_PER_BLOCK):
    # Generate the pixel coords block by block. Points per timeline entry are every multiple
    # of dt inside [t_start, t_end), plus the very end point. Membership is decided by these
    # counts (not by comparing floats) so the per-command counts in the lag file always
    # match the rows written.
    cum_counts = np.cumsum(point_counts(tl, dt))
    total_points = int(cum_counts[-1])

    is_arc = tl.kind != 1
    seg_len_um = np.maximum(tl.length * scale, 1e-12)
    ux = np.where(tl.length > 0, (tl.x2 - tl.x1) / seg_len_um, 0.0)
    uy = np.where(tl.length > 0, (tl.y2 - tl.y1) / seg_len_um, 0.0)
    radius = np.hypot(tl.x1 - tl.cx, tl.y1 - tl.cy)
    a_start = np.arctan2(tl.cy - tl.y1, tl.x1 - tl.cx)
    direction = np.where(tl.sweep > 0, 1.0, -1.0)
    v0, peak, length, d_acc, d_dec = tl.v0, tl.peak, tl.length, tl.d_acc, tl.d_dec
    t_acc, t_cruise, t_dec, t_start, duration = tl.t_acc, tl.t_cruise, tl.t_dec, tl.t_start, tl.duration

    for first in range(0, total_points, block):
        # One extra point past the block so the last point's spacing (speed) is known
        count = min(first + block, total_points) - first
        k = np.arange(first, min(first + count + 1, total_points), dtype=np.int64)
        m = np.searchsorted(cum_counts, k, side="right")
        tau = np.clip(k * dt - t_start[m], 0.0, duration[m])
        in_acc = tau < t_acc[m]
        in_cruise = ~in_acc & (tau < t_acc[m] + t_cruise[m])
        in_dec = ~in_acc & ~in_cruise
        tau_dec = np.maximum(tau - t_acc[m] - t_cruise[m], 0.0)
        dist = np.where(in_acc, v0[m] * tau + 0.5 * accel * tau ** 2,
               np.where(in_cruise, d_acc[m] + peak[m] * (tau - t_acc[m]),
                        length[m] - d_dec[m] + peak[m] * tau_dec - 0.5 * accel * tau_dec ** 2))
        dist = np.clip(dist, 0.0, length[m])
        speed = np.where(in_acc, v0[m] + accel * tau, np.where(in_cruise, peak[m], np.maximum(peak[m] - accel * tau_dec, 0.0)))
        acc = np.where(in_acc, accel, np.where(in_cruise, 0.0, -accel))
        acc = np.where((in_acc & (t_acc[m] <= 0)) | (in_dec & (t_dec[m] <= 0)), 0.0, acc)

        d_um = dist * scale
        angle = a_start[m] + direction[m] * d_um / np.maximum(radius[m], 1e-12)
        arc = is_arc[m]
        x = np.where(arc, tl.cx[m] + radius[m] * np.cos(angle), tl.x1[m] + ux[m] * d_um)
        y = np.where(arc, tl.cy[m] - radius[m] * np.sin(angle), tl.y1[m] + uy[m] * d_um)

        # Speed of each point = distance to the next point over the time step, i.e. the
        # speed of that piece of the path given the pixel coords spacing. The very last
        # point of the print has no next point and keeps the planned speed (0 at the end).
        spacing_speed = np.hypot(np.diff(x), np.diff(y)) / scale / dt
        if len(k) > count:
            speed = spacing_speed
        else:
            speed = np.append(spacing_speed, speed[-1])
        yield SimpleNamespace(
            first=first, count=count, total=total_points, last=first + count == total_points,
            full_x=x, full_y=y,  # includes the extra point (for drawing the piece to it)
            x=x[:count], y=y[:count], speed=speed, acc=acc[:count], m=m[:count], tau=tau[:count],
            d_um=d_um[:count], arc=arc[:count], radius=radius, t=(t_start[m] + tau)[:count])


# --- lag-format files --------------------------------------------------------------------

class LagFormatWriter:
    # Writes pixel coords in Gcode_processing.py's layout for the lag model:
    #   <name>.csv        "x, y, 'G-code', count" on the first point of each command,
    #                     "x, y" on the rest (µm, y flipped)
    #   <motion>.csv      the same rows in the same order with each point's time, speed,
    #                     acceleration and G-code line
    # A command with no points of its own (feed-only line, move shorter than one time step)
    # gets one repeated point so every command appears in order.

    def __init__(self, xy_file, motion_path, commands, owner, counts):
        self.xy_file = xy_file
        self.commands = commands
        self.rows_per_command = np.bincount(owner, weights=counts, minlength=len(commands)).astype(np.int64)
        self.rows_per_command[self.rows_per_command == 0] = 1
        self.motion_file = open(motion_path, "w")
        self.motion_file.write("time_s,speed_mm_s,accel_mm_s2,line\n")
        self.last_command = -1
        self.last_row = None

    def _blank_commands(self, up_to, xy_row, motion_row, xy_out, motion_out):
        # Commands between the last one written and up_to with no points of their own
        for c in range(self.last_command + 1, up_to):
            xy_out.append(f"{xy_row[0]}, {xy_row[1]}, {self.commands[c]!r}, 1")
            motion_out.append(motion_row)
        self.last_command = max(self.last_command, up_to - 1)

    def write(self, b, owner, line):
        xy_out, motion_out = [], []
        xs, ys = np.round(b.x, 3).tolist(), np.round(b.y, 3).tolist()
        ts, vs, acs, ls = b.t.tolist(), b.speed[:b.count].tolist(), b.acc.tolist(), line[b.m].astype(np.int64).tolist()
        cmd = owner[b.m].tolist()
        rows_per_command, commands = self.rows_per_command, self.commands
        for i in range(len(xs)):
            motion_row = f"{ts[i]:.4f},{vs[i]:.5f},{acs[i]:.1f},{ls[i]}"
            c = cmd[i]
            if c != self.last_command:
                self._blank_commands(c, self.last_row[0] if self.last_row else (xs[i], ys[i]),
                                     self.last_row[1] if self.last_row else motion_row, xy_out, motion_out)
                xy_out.append(f"{xs[i]}, {ys[i]}, {commands[c]!r}, {rows_per_command[c]}")
                self.last_command = c
            else:
                xy_out.append(f"{xs[i]}, {ys[i]}")
            motion_out.append(motion_row)
            self.last_row = ((xs[i], ys[i]), motion_row)
        if b.last:
            self._blank_commands(len(commands), self.last_row[0], self.last_row[1], xy_out, motion_out)
        self.xy_file.write("\n".join(xy_out) + "\n")
        self.motion_file.write("\n".join(motion_out) + "\n")

    def close(self):
        self.motion_file.close()


# --- PNG drawing --------------------------------------------------------------------------

def blank_index_image(variables):
    # Plate-sized colour-index image (10 µm per pixel) with the 1 mm grid, as Plot_code() draws
    height, width = int(variables["Y_build"] * 100), int(variables["X_build"] * 100)
    image = np.zeros((height, width), dtype=np.uint8)
    for gx in range(100, width, 100):
        image[:, gx] = GRID_INDEX
    for gy in range(100, height, 100):
        image[gy, :] = GRID_INDEX
    return image


def to_png_pixels(variables, x, y):
    # Calc-pass µm -> plot pixels (with PNG_SHIFT sub-pixel bits), matching Plot_code()'s
    # origin 1 mm in from the lowest x / y
    scale = variables["scale"]
    off_x = (abs(variables["min_x"]) + 1) * scale
    off_y = (abs(variables["min_y"]) + 1) * scale
    return (x + off_x) / 10.0 * (1 << PNG_SHIFT), (y + off_y) / 10.0 * (1 << PNG_SHIFT)


def draw_runs(index_image, values, px, py, line_width):
    # One polyline per run of equal colour; piece i (point i -> i + 1) has colour values[i]
    if len(values) == 0:
        return
    breaks = np.flatnonzero(values[1:] != values[:-1]) + 1
    starts = np.concatenate(([0], breaks))
    ends = np.concatenate((breaks, [len(values)]))
    pts = np.column_stack((px, py)).astype(np.int32)
    for a_, b_ in zip(starts.tolist(), ends.tolist()):
        cv2.polylines(index_image, [pts[a_:b_ + 1]], False, int(values[a_]), line_width, cv2.LINE_8, PNG_SHIFT)


# --- the sharp-corner pixel coords ---------------------------------------------------------

def generate_pixel_coords(params, variables):
    # Pixel coords along the planned motion of the G-code as written (sharp corners).
    # At 1 ms this is millions of points (13 M for a 3.7 h print), so everything is done
    # with numpy in blocks rather than as Python lists:
    #   - high_speed False: every point is written out for the lag model
    #       _pixel_cords.csv / _pixel_coords_motion.csv (see LagFormatWriter)
    #   - _pixel_coords_speed.png / _pixel_coords_accel.png: a line from each point to the
    #     next, coloured by that piece's speed (32 bands) or acceleration
    #   - params["Motion_samples"]: the points needed to draw the speed / acceleration
    #     overlay in the GUI - any point where the speed band or the accel/cruise/decel phase
    #     changes, the first and last point of every command, and enough points on arcs to
    #     keep them round. Nothing between those carries extra information for the overlay.
    #   - params["Lag_index_image"]: the plate with the nozzle path drawn in, for the jet-lag
    #     PNG (lag_model.py draws the jet over it)
    planned = params.get("Planned_moves", [])
    dwells = params.get("Dwells", [])
    if not planned:
        params["Motion_samples"] = np.zeros((0, 5), dtype=np.float32)
        return
    accel = float(variables["Acceleration_mm_s2"])
    dt = float(variables["scatter_resolution"])
    scale = variables["scale"]
    tl = build_timeline(params["Preview_segments"], planned, dwells, accel)

    # Speed bands (the GUI uses the same 32) so a band change is always kept
    v_min = float(min(tl.v0.min(), tl.v1.min(), tl.peak.min()))
    v_max = float(tl.peak.max())
    params["Motion_speed_range"] = (v_min, v_max)
    arc_step = math.radians(5.0)  # keep a point every 5 degrees on arcs
    out_base = output_base(params)
    with open(out_base + "_pixel_coords_legend.json", "w") as f:
        json.dump({"speed_min_mm_s": v_min, "speed_max_mm_s": v_max, "speed_bands": SPEED_BANDS,
                   "speed_band_edges_mm_s": band_edges(v_min, v_max),
                   "cts_mm_s": variables.get("global_return_CTS", 0) / 60.0,
                   "acceleration_mm_s2": accel, "junction_deviation_mm": variables["Junction_deviation_mm"],
                   "jerk_mm_s": variables.get("Jerk_mm_s", 0.0)}, f)

    # PNG images drawn straight from the pixel coords, on the same plate, scale (10 µm per
    # pixel) and grid as the move-type PNG from Plot_code(). Drawn as colour indices so
    # several full-size images fit in memory, and saved as palette PNGs.
    draw_png = variables["Generate_output_image"]
    draw_nozzle = variables.get("Lag_prediction", False) and draw_png
    line_width = int(variables["Line_width"])
    if draw_png:
        speed_index = blank_index_image(variables)
        accel_index = speed_index.copy()
        lag_index = speed_index.copy() if draw_nozzle else None

    write_all = variables["high_speed"] == False
    if write_all:
        writer = LagFormatWriter(params["Pixel_File"], out_base + "_pixel_coords_motion.csv",
                                 params["One_coordinate_system"], tl.owner, point_counts(tl, dt))

    kept = []
    previous = None  # last point of the previous block: (entry, band, phase, arc bucket)
    for b in sample_timeline(tl, accel, dt, scale):
        m = b.m
        if draw_png:
            px, py = to_png_pixels(variables, b.full_x, b.full_y)
            draw_runs(speed_index, band_of(b.speed, v_min, v_max) + 1, px, py, line_width)
            draw_runs(accel_index, np.sign(b.acc).astype(np.int64) + 2, px, py, line_width)
            if draw_nozzle:
                cv2.polylines(lag_index, [np.column_stack((px, py)).astype(np.int32)], False, NOZZLE_INDEX, line_width, cv2.LINE_8, PNG_SHIFT)

        if write_all:
            writer.write(b, tl.owner, tl.line)

        # Keep only the points where something visible changes
        speed = b.speed[:b.count]
        band = band_of(speed, v_min, v_max)
        phase = np.sign(b.acc).astype(np.int64)
        bucket = np.where(b.arc, np.floor(b.d_um / np.maximum(b.radius[m], 1e-12) / arc_step), -1).astype(np.int64)
        keys = (m, band, phase, bucket)
        change = np.zeros(b.count, dtype=bool)
        for key in keys:
            change[1:] |= key[1:] != key[:-1]
        change[0] = previous is None or any(int(key[0]) != p for key, p in zip(keys, previous))
        move_change = np.zeros(b.count, dtype=bool)
        move_change[1:] = m[1:] != m[:-1]
        keep = change.copy()
        keep[:-1] |= move_change[1:]  # also the last point of each command, to hold the corner
        keep[-1] = keep[-1] or b.last
        kept.append(np.column_stack((b.x, b.y, speed, b.acc, tl.line[m]))[keep].astype(np.float32))
        previous = tuple(int(key[-1]) for key in keys)
        print("Pixel coords progress:", str(int(100 * (b.first + b.count) / b.total)) + "%")
    total_points = b.total

    if draw_png:
        for index_image, colours, suffix in ((speed_index, SPEED_COLOURS, "_pixel_coords_speed.png"),
                                             (accel_index, ACCEL_COLOURS, "_pixel_coords_accel.png")):
            save_indexed_png(out_base + suffix, index_image, colours, variables["Background_colour"])
            print("Pixel coords image saved:", out_base + suffix)
        del speed_index, accel_index
        params["Lag_index_image"] = lag_index

    if write_all:
        writer.close()
        variables["Motion_pixel_coords_written"] = True
        print("Pixel coords saved (lag format):", total_points, "points,", out_base + "_pixel_cords.csv")
        print("Pixel coords motion data saved:", out_base + "_pixel_coords_motion.csv")
    params["Motion_samples"] = np.concatenate(kept) if kept else np.zeros((0, 5), dtype=np.float32)
    print("Pixel coords (accel/junction):", total_points, "points every", round(dt * 1000, 3), "ms,", len(params["Motion_samples"]), "kept for the speed overlay")


def point_counts(tl, dt):
    # Points per timeline entry, as sample_timeline() places them
    cum_counts = np.ceil(tl.t_end / dt - 1e-9).astype(np.int64)
    cum_counts[-1] += 1
    return np.diff(np.concatenate(([0], cum_counts)))
