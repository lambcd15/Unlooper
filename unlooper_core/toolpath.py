"""Stage 2 - interpreting each unlooped command as tool movement: track the position
(G90 / G91 / G92), work out G1 lines and G2 / G3 arcs (centre, sweep, radius
checks and fixes), their distance, time and material, and record them as numeric
toolpath segments (params['Preview_segments']) for the planner, pixel coords and preview."""
import math
import re

import cv2
import numpy as np

from .common import error_message
from .gcode_reader import is_macro_line

def draw_ellipse(params, variables):
    # uses the shift to accurately get sub-pixel resolution for arc
    # taken from https://stackoverflow.com/a/44892317/5087436
    # The units from center and axes are in µm
    # Convert to tens of µm before plotting
    lineType=cv2.LINE_AA
    shift=10
    center = (int(round(((params["Centre_1"][0] / variables["scale"]) * 100) * 2**shift)),int(round(((params["Centre_1"][1] / variables["scale"]) * 100) * 2**shift)),)
    axes = (int(round(((params["Axes"][0] / variables["scale"]) * 100) * 2**shift)),int(round(((params["Axes"][1] / variables["scale"]) * 100) * 2**shift)),)

    return cv2.ellipse(params["Image"],center,axes,params["Pt1_angle"],params["Pt2_angle"],params["Diff"],params["Plotting_colour"],variables["Line_width"],lineType,shift,)


def draw_circle(params, variables):
    lineType=cv2.LINE_AA
    shift=10
    # uses the shift to accurately get sub-pixel resolution for arc
    # taken from https://stackoverflow.com/a/44892317/5087436
    center = (int(round(((params["Centre_1"][0] / variables["scale"]) * 100) * 2**shift)),int(round(((params["Centre_1"][1] / variables["scale"]) * 100) * 2**shift)),)
    radius = int(round(((params["Radius"] / variables["scale"]) * 100 )* 2**shift))
    return cv2.circle(params["Image"], center, radius, params["Plotting_colour"], variables["Line_width"], lineType, shift)


def draw_line(params, variables):
    lineType=cv2.LINE_AA
    shift=10
    # System must provide the units in 10's of um
    # uses the shift to accurately get sub-pixel resolution for arc
    # taken from https://stackoverflow.com/a/44892317/5087436
    center1 = (int(round(((params["Centre_1"][0] / variables["scale"]) * 100) * 2**shift)),int(round(((params["Centre_1"][1] / variables["scale"]) * 100)* 2**shift)),)
    center2 = (int(round(((params["Centre_2"][0] / variables["scale"]) * 100) * 2**shift)),int(round(((params["Centre_2"][1] / variables["scale"]) * 100) * 2**shift)),)
    return cv2.line(params["Image"], center1, center2, params["Plotting_colour"],  variables["Line_width"], lineType, shift)


def setdirection(x1, x3, y1, y3):
    dy = y3 - y1
    if dy < 0:
        yo = -1
    else:
        yo = 1
    dy = abs(dy)
    dx = x3 - x1
    if dx < 0:
        xo = -1
    else:
        xo = 1
    dx = abs(dx)
    fxy = dx - dy
    return fxy, xo, yo, dx, dy


def getdir(f, a, b, d):
    binrep = 0
    xo = yo = 0
    if d == 1:
        binrep = binrep + 8
    if f == 1:
        binrep = binrep + 4
    if a == 1:
        binrep = binrep + 2
    if b == 1:
        binrep = binrep + 1

    if binrep == 0:
        yo = -1
    if binrep == 1:
        xo = -1
    if binrep == 2:
        xo = 1
    if binrep == 3:
        yo = 1
    if binrep == 4:
        xo = 1
    if binrep == 5:
        yo = -1
    if binrep == 6:
        yo = 1
    if binrep == 7:
        xo = -1
    if binrep == 8:
        xo = -1
    if binrep == 9:
        yo = 1
    if binrep == 10:
        yo = -1
    if binrep == 11:
        xo = 1
    if binrep == 12:
        yo = 1
    if binrep == 13:
        xo = 1
    if binrep == 14:
        xo = -1
    if binrep == 15:
        yo = -1

    return xo, yo


def doline(params, variables):
    # Values are coming in are in 100 of nm to ensure accuracy with the simulation
    # Units inbound are floats
    # Change the units to 10's of µm
    # Input is in µm which is then converted to mm then to 10's of µm

    # 2025 skip if the line does not contain XYZIJ (ported from Gcode_processing.py) - a
    # command with no movement still gets one pixel coords entry so the per-command
    # bookkeeping in line_reader lines up
    if math.isnan(params["X_increase"]) and math.isnan(params["Y_increase"]) and math.isnan(params["Z_increase"]):
        params["Pixel_coords_um"].append([variables["Current_X"],variables["Current_Y"],"",0]) # Append a blank line to the pixel coords array
        return params, variables

    # Determine the angle of the line
    params["Angle"] = math.atan2(params["Y2"] - variables["Current_Y"], params["X2"] - variables["Current_X"])
    # Extract the values ***********************************************
    # previous_values stores the last commands distance left and feedrate
    # Next determine the distance into the current command from the segment_length and the distance_from_previous
    # First determine the segment length
    segment_length = (round(variables["scatter_resolution"] * params["Feed_rate"], 10) * 1000)  # To get segment length (speed mm/s * time s) tehn convert to microns
    # Then determine the distance from the start of command
    start_distance = 0
    if params["Feed_rate_previous"] != 0:
        start_distance = segment_length - (params["Distance_from_previous"] * (params["Feed_rate"] / params["Feed_rate_previous"]))
    # Now determine the length or distance of the current command
    # params["Distance"] = math.sqrt((params["X3"] - params["X1"]) ** 2 + (params["Y3"] - params["Y1"]) ** 2)  # micron
    # Now check to see if the start_distance is greater than the current command length, if it is skip the command
    if start_distance >= params["Distance"]:
        # The length is greater than the segment length as such skip the segment with the new distance left
        params["Distance_from_previous"] = start_distance - params["Distance"]
        segments = 0
    else:
        segments = math.trunc((params["Distance"] - start_distance) / segment_length)
        params["Distance_from_previous"] = params["Distance"] - start_distance - segments * segment_length
    # print("previous ",distance_from_previous, "start_distance ", start_distance, "distance ", distance, "distance left ",distance_left,"segments ",segments, "segment_length ",segment_length)
    # Loop through the rest of the distance
    # To allow the gcode command to be added to pixel cords
    first_go = False
    # 2/10/2023 trying to increase speed using map and high speed
    if variables["high_speed"] == False and not variables["Motion_pixel_coords"]:
        for i in range(0, segments + 1):  # Plus one to compensate for start point
            params["X2"] = variables["Current_X"] + (start_distance + segment_length * i) * math.cos(params["Angle"])
            params["Y2"] = variables["Current_Y"] + (start_distance + segment_length * i) * math.sin(params["Angle"])
            params["X4"] = round((params["X2"]) / variables["scale"] * 100)
            params["Y4"] = round((params["Y2"]) / variables["scale"] * 100)
            # print(params["X2"],params["Y2"])
            if variables["calc_only"] == 0:
                params["Plotting_colour"] = variables["G1_colour"]
                params["Radius"] = variables["scatter_size"]
                draw_circle(params, variables)
            else:
                params["Pixel_coords"].append([params["X4"], params["Y4"]])
                if first_go == False:
                    # This is to allow for gcodes to be added to the pixel cords file for the lag vector calculation and then the length of each command alogn the list
                    params["Pixel_coords_um"].append([params["X2"],params["Y2"],params["Line"],0])
                    first_go = True
                else:
                    params["Pixel_coords_um"].append([params["X2"],params["Y2"]])
    return params, variables


def docircle(params, variables, flag=0):
    # Calcualte the length of the arc
    if flag == 1:
        # This is for the case where the arc is a full circle and the diff is 360 degrees
        params["Diff"] = 360.0
    # Calcualte the length of the arc
    params["Distance"] = (math.pi * params["Radius"] * 2) * (params["Diff"] / 360.0)
    # Determine the number of segments
    segment_length = (round(variables["scatter_resolution"] * params["Feed_rate"], 10) * 1000)  # To get segment length (speed mm/s * time s)
    # Then determine the distance from the start of command
    start_distance = 0
    if params["Feed_rate_previous"] != 0:
        start_distance = segment_length - (params["Distance_from_previous"] * (params["Feed_rate"] / params["Feed_rate_previous"]))
    # Now check to see if the start_distance is greater than the current command length, if it is skip the command
    if start_distance >= params["Distance"]:
        # The length is greater than the segment length as such skip the segment with the new distance left
        params["Distance_from_previous"] = start_distance - params["Distance"]
        segments = 0
    else:
        segments = math.trunc((params["Distance"] - start_distance) / segment_length)
        params["Distance_from_previous"] = params["Distance"] - start_distance - segments * segment_length
    # Determine the new start angle, then calculate the next points
    # Calculate the first point given the distance
    # print(start_distance, params["Distance"], params["Diff"],params["Radius"])
    # print(params["Line"])
    theta_0 = start_distance / params["Distance"] * params["Diff"]
    theta = ((start_distance + segment_length) / params["Distance"] * params["Diff"]) - theta_0
    # print("previous ",distance_from_previous, "start_distance ", start_distance, "distance ", distance, "distance left ",distance_left,"segments ",segments, "segment_length ",segment_length, "start angle ",theta_0 )
    # To allow the gcode command to be added to pixel cords
    first_go = False
    # 2/10/2023 trying to increase speed using map and high speed
    if variables["high_speed"] == False and not variables["Motion_pixel_coords"]:
        for i in range(0, segments + 1):
            # Calculate the new co-ordinates then plot the line
            if params["dir"] == 3:
                # The negative sign is added to ensure that the direction is correct for the counter_clockwise move
                params["X2"] = (params["X"] + (variables["Current_X"] - params["X"]) * math.cos(math.radians(theta * i * -1 - theta_0)) - (variables["Current_Y"] - params["Y"]) * math.sin(math.radians(theta * i * -1 - theta_0)))
                params["Y2"] = (params["Y"] + (variables["Current_X"] - params["X"]) * math.sin(math.radians(theta * i * -1 - theta_0)) + (variables["Current_Y"] - params["Y"]) * math.cos(math.radians(theta * i * -1 - theta_0)))
            else: #params["dir"] == 2:
                params["X2"] = (params["X"] + (variables["Current_X"] - params["X"]) * math.cos(math.radians(theta * i + theta_0)) - (variables["Current_Y"] - params["Y"]) * math.sin(math.radians(theta * i + theta_0)))
                params["Y2"] = (params["Y"] + (variables["Current_X"] - params["X"]) * math.sin(math.radians(theta * i + theta_0)) + (variables["Current_Y"] - params["Y"]) * math.cos(math.radians(theta * i + theta_0)))
                # https://math.stackexchange.com/questions/2688062/calculating-the-coordinates-of-end-terminal-point-of-an-arc-from-known-r-arc-in
            params["X4"] = round((params["X2"] / variables["scale"]) * 100) # NaN caused by direction of I or J command
            params["Y4"] = round((params["Y2"] /  variables["scale"]) * 100)
            if variables["calc_only"] == 0:
                params["Plotting_colour"] = variables["G2_colour"]
                params["Radius"] = variables["scatter_size"]
                draw_circle(params, variables)
            else:
                params["Pixel_coords"].append([params["X4"], params["Y4"]])
                if first_go == False:
                    params["Pixel_coords_um"].append([params["X2"],params["Y2"],params["Line"],0])
                    first_go = True
                else:
                    params["Pixel_coords_um"].append([params["X2"],params["Y2"]])
    return params, variables


def check_command(params, variables):
    # This function checks with the global command array and determines is the command has been used before (1) or not (0)
    # Once the code has been completed the image can be then be used to iterate accross to form the array
    # This also returns the color increase every time a command is called so that the color changes the more times the print head passses over the same location
    color = (5, 0, 0)
    key = (variables["Current_X"], variables["Current_Y"], params["Line"], params["Positioning"])
    if key in params["commands_used"]:
        params["commands_used"][key] += 1
        count = params["commands_used"][key]
        color = (5 * count, 0, 0)
        if 5 * count > 255:
            color = (255, (5 * count) - 255, 0)
        # print(color)
        if variables["all_black"] == 0:
            params["command_check"] = True
            params["Plotting_colour"] = color
            return params, variables
        else:
            params["command_check"] = True
            params["Plotting_colour"] = (0, 0, 0)
            return params, variables
    else:
        if variables["all_black"] == 0:
            params["command_check"] = False
            params["Plotting_colour"] = color
            return params, variables
        else:
            params["command_check"] = False
            params["Plotting_colour"] = (0, 0, 0)
            return params, variables


def radius_check(x1, y1, x2, y2, center):
    # This is not required but is good practice for the start and end of the curve to have the same
    # radius from the center point
    # Inputs are in µm
    distance_1 = math.sqrt(((x1 - center[0]) ** 2) + ((y1 - center[1]) ** 2))
    distance_2 = math.sqrt(((x2 - center[0]) ** 2) + ((y2 - center[1]) ** 2))
    difference = abs(distance_1 - distance_2)
    if difference > 0.002:
        return -1
    else:
        return 1


def record_preview_segment(params, variables, kind, x1, y1, x2, y2, cx=0.0, cy=0.0, full_circle=False):
    # Store one move for the vector preview (µm, y already flipped to screen/y-down).
    # Only recorded in the motion-calculation pass - Plot_code re-reads the same lines in a
    # shifted frame, so recording there too would double every segment in "both" mode.
    if variables["calc_only"] != 1:
        return
    if not variables["Generate_preview_image"] and variables["Acceleration_mm_s2"] <= 0:
        return
    sweep = 0.0
    if kind in (2, 3):
        # Sweep in degrees using Qt's arcTo convention: +ve = anticlockwise on screen.
        # G2 is clockwise on screen, G3 anticlockwise.
        a1 = math.degrees(math.atan2(cy - y1, x1 - cx))
        a2 = math.degrees(math.atan2(cy - y2, x2 - cx))
        if full_circle:
            sweep = 360.0
        elif kind == 2:
            sweep = (a1 - a2) % 360.0
        else:
            sweep = (a2 - a1) % 360.0
        if kind == 2:
            sweep = -sweep
    # Last field is the programmed feed (mm/s) for the acceleration planner
    # ... plus the index of this command in One_coordinate_system (appended right after this)
    params["Preview_segments"].append((kind, x1, y1, x2, y2, cx, cy, sweep, params.get("Preview_line_number", 0), params["Feed_rate"], len(params["One_coordinate_system"])))


def Plotting_G1_2D(params, variables):
    # This function plots G1 commands as well as calculating the distance reuquired for each command
    # New idea 3/03/2023 keep all units at floats in µm then at the last second before plotting convert but keep in DRO as correct units (pixel_cords)
    # Extract parameters from array
    # Function to check commands that have already been plotted
    if variables["calc_only"] == 0 and variables["Disable_colour_update"] == False:  # Ensure the function is in plotting mode
        params, variables = check_command(params, variables)
        if params["command_check"] == False:
            params["commands_used"][(variables["Current_X"], variables["Current_Y"], params["Line"], params["Positioning"])] = 1
    # Segment the line into seperate cells
    params, variables = segment_line(params, variables)
    if params["Positioning"] == "G90" or params["Positioning"] == "G90 ":
        if math.isnan(params["X_increase"]):
            params["X2"] = variables["Current_X"]
        else:
            params["X2"] = variables["Origin_X"] + params["X_increase"]
        if math.isnan(params["Y_increase"]):
            params["Y2"] = variables["Current_Y"]
        else:
            params["Y2"] = variables["Origin_Y"]  - params["Y_increase"]
    elif params["Positioning"] == "G91" or params["Positioning"] == "G91 ":
        if math.isnan(params["X_increase"]):
            params["X2"] = variables["Current_X"]
        else:
            params["X2"] = variables["Current_X"] + params["X_increase"]
        if math.isnan(params["Y_increase"]):
            params["Y2"] = variables["Current_Y"]
        else:
            params["Y2"] = variables["Current_Y"] - params["Y_increase"]
    # Create a temporary holder to ensure that the next command starts at the right point
    params["X2"] = round(params["X2"],2)
    params["Y2"] = round(params["Y2"],2)
    temp1_x = params["X2"]
    temp1_y = params["Y2"]
    params["X1"] = variables["Current_X"]
    params["Y1"] = variables["Current_Y"]
    # Create the start and end posistions for plotting
    params["Centre_1"] = []
    params["Centre_2"] = []

    params["Centre_2"].append(params["X2"])
    params["Centre_2"].append(params["Y2"])

    params["Centre_1"].append(variables["Current_X"])
    params["Centre_1"].append(variables["Current_Y"])
    # If the line only contains Feed rate command (required for Marlin return)
    if "F" in params["Command_array"] and len(params["Command_array"]) == 4:
        # As in Gcode_processing.py: doline() records the blank pixel coords entry
        if variables["calc_only"] == 1:
            params, variables = doline(params, variables)
        return params, variables
    if variables["calc_only"] == 1:
        # Only calculate the distance if the system is asking for it
        # Keep as high precision units
        params["Distance"] = math.sqrt((params["X2"] - variables["Current_X"]) ** 2 + (params["Y2"] - variables["Current_Y"]) ** 2)
        params, variables = doline(params, variables)
    else:
        # if check == 0:
        # Only plot the line if the program has not plotted that command from that posistion before
        if variables["scatter_path"] == 0:
            draw_line(params, variables)
        else:  # Scatter command
            params, variables = doline(params, variables)
    record_preview_segment(params, variables, 1, variables["Current_X"], variables["Current_Y"], temp1_x, temp1_y)
    variables["Current_X"] = round(temp1_x,2)
    variables["Current_Y"] = round(temp1_y,2)
    return params, variables


def Plotting_G2_2D(params, variables):
    # This function will plot the G2 command using the current line on the plot
    # All parameters are in µm given by scale
    # Check if the command has been used before plotting it, this is used to speed up processing
    if variables["calc_only"] == 0 and variables["Disable_colour_update"] == False:  # Ensure the function is in plotting mode
        params, variables = check_command(params, variables)
        if params["command_check"] == False:
            params["commands_used"][(variables["Current_X"], variables["Current_Y"], params["Line"], params["Positioning"])] = 1
    # Segment the line into seperate cells
    params, variables = segment_line(params, variables)
    # Update the end co-ordinates for the end of the curve ensure that the correct co-ordinate system is used
    if params["Positioning"] == "G90" or params["Positioning"] == "G90 ":
        if math.isnan(params["X_increase"]):
            params["X2"] = variables["Current_X"]
        else:
            params["X2"] = variables["Origin_X"] + params["X_increase"]
        if math.isnan(params["Y_increase"]):
            params["Y2"] = variables["Current_Y"]
        else:
            params["Y2"] = variables["Origin_Y"]  - params["Y_increase"]
    elif params["Positioning"] == "G91" or params["Positioning"] == "G91 ":
        if math.isnan(params["X_increase"]):
            params["X2"] = variables["Current_X"]
        else:
            params["X2"] = variables["Current_X"] + params["X_increase"]
        if math.isnan(params["Y_increase"]):
            params["Y2"] = variables["Current_Y"]
        else:
            params["Y2"] = variables["Current_Y"] - params["Y_increase"]
    # Create a temporary holder to ensure that the next command starts at the right point
    # print(params["X2"])
    params["X2"] = round(params["X2"],2)
    params["Y2"] = round(params["Y2"],2)
    temp2_x = params["X2"]
    temp2_y = params["Y2"]
    params["X1"] = variables["Current_X"]
    params["Y1"] = variables["Current_Y"]
    # Determine the center of the arc given the the start and end pos
    q = math.sqrt((params["X2"] - variables["Current_X"]) ** 2 + (params["Y2"] - variables["Current_Y"]) ** 2)
    params["Y3"] = (variables["Current_Y"] + params["Y2"]) / 2
    params["X3"] = (variables["Current_X"] + params["X2"]) / 2

    if q == 0 and params["I_increase"] == 0 and params["J_increase"] == 0:
        return params, variables
    
    if (params["Line"].find("J", 0, len(params["Line"])) != -1 or params["Line"].find("I", 0, len(params["Line"])) != -1):
        # Determine the radius and center using I and J and then check the radius against both pos to check for failure
        params["Radius"] = math.sqrt(params["I_increase"]**2 + params["J_increase"]**2)
        # x and y are the centre of the circle
        params["X"] = variables["Current_X"] + params["I_increase"]
        params["Y"] = variables["Current_Y"] - params["J_increase"]
        params["Centre_1"] = ((params["X"]), (params["Y"]))
        # print(x1, y1, x2, y2, center)
        # Error check the radius to ensure that it is physcially possible
        if radius_check(variables["Current_X"], variables["Current_Y"], params["X2"], params["Y2"],  params["Centre_1"]) == -1:
            if variables["radius_error_output"]:
                if variables["radius_error_fix"]:
                    # Fix the radius error by correcting the centre using the distance between the arcs
                    params["Center_1"] = ((params["X3"]), (params["Y3"]))
                    variables["edited_flag"] = True
                    params["Edited_line"] = "G2 X" + str(params["X_increase"] / variables["scale"]) + " Y" + str(params["Y_increase"] / variables["scale"]) + " I" + str(round((params["X2"] - variables["Current_X"])/ 2) / variables["scale"]) + " J" + str(round((variables["Current_Y"] - params["Y2"]) / 2) / variables["scale"])
                    # print("editied",edited_line)
            if variables["Display_radius_error"]:
                print(params["Line"])
                error_message(-5)
    else:
    # elif (params["Line"].find("J", 0, len(params["Line"])) == -1 or params["Line"].find("I", 0, len(params["Line"])) == -1):
        params["X"] = params["X3"] + math.sqrt((params["Radius"]**2) - ((q / 2) ** 2)) * (variables["Current_Y"] - params["Y2"]) / q
        params["Y"] = params["Y3"] + math.sqrt((params["Radius"]**2) - ((q / 2) ** 2)) * (params["X2"] - variables["Current_X"]) / q
        # Determine the center of the arc
        params["Centre_1"] = ((params["X"]), (params["Y"]))
    # print(params["Centre_1"])
    # if variables["calc_only"] == 0:
    #     temp = params["Radius"]
    #     params["Radius"] = 1
    #     temp_colour = params["Plotting_colour"]
    #     params["Plotting_colour"] = [255,50,120]
    #     draw_circle(params, variables)
    #     params["Radius"] = temp
    #     params["Plotting_colour"] = temp_colour
    if q == 0:
        # This is a full circle so the start and end pos are the same
        if variables["calc_only"] == 0:
            if variables["scatter_path"] == 0:
                draw_circle(params,variables)
            else:
                params["dir"] = 2
                params,variables = docircle(params,variables, flag=1)
        else:
            # Calculate the distance of the circle and then call the docircle function to get the pixel coordinates
            params["Distance"] = math.pi * params["Radius"] * 2
            params["dir"] = 2
            params, variables = docircle(params, variables, flag=1)
        record_preview_segment(params, variables, 2, variables["Current_X"], variables["Current_Y"], temp2_x, temp2_y, params["X"], params["Y"], full_circle=True)
    else:
        # Determine the start and end angle of the arc
        if params["Line"].find("J", 0, len(params["Line"])) != -1 or params["Line"].find("I", 0, len(params["Line"])) != -1:
            params["Pt1_angle"] = (180 * np.arctan2(variables["Current_Y"] - (variables["Current_Y"] - params["J_increase"]), variables["Current_X"] - (variables["Current_X"]  + params["I_increase"])) / np.pi)
            params["Pt2_angle"] = (180 * np.arctan2(params["Y2"] - (variables["Current_Y"] - params["J_increase"]), params["X2"] - (variables["Current_X"]  + params["I_increase"])) / np.pi)
        else:
            params["Pt1_angle"] = 180 * np.arctan2(variables["Current_Y"] - params["Y"], variables["Current_X"]  - params["X"]) / np.pi
            params["Pt2_angle"] = 180 * np.arctan2(params["Y2"] - params["Y"], params["X2"] - params["X"]) / np.pi
            # https://stackoverflow.com/questions/36211171/finding-center-of-a-circle-given-two-points-and-radius
        # As we are plotting using an ellipse we want to set both the major and minor axis the same
        params["Axes"] = ((params["Radius"]), (params["Radius"]))
        # Adjust the start and end angle based on the sign of the pt1 and pt2 angles to account for params["Diff"]erent orientations of the arc
        if params["Pt1_angle"] < 0 and params["Pt2_angle"] > 0:
            if params["Pt1_angle"] > params["Pt2_angle"]:
                params["Diff"] = abs(params["Pt1_angle"] - params["Pt2_angle"])
            else:
                params["Diff"] = abs(params["Pt2_angle"] - params["Pt1_angle"])
        elif params["Pt1_angle"] > 0 and params["Pt2_angle"] > 0:
            if params["Pt1_angle"] > params["Pt2_angle"]:
                params["Diff"] = 360 - abs(params["Pt1_angle"] - params["Pt2_angle"])
            else:
                params["Diff"] = abs(params["Pt2_angle"] - params["Pt1_angle"])
        elif params["Pt1_angle"] < 0 and params["Pt2_angle"] < 0:
            if params["Pt1_angle"] > params["Pt2_angle"]:
                params["Diff"] = 360 - abs(params["Pt1_angle"] - params["Pt2_angle"])
            else:
                params["Diff"] = abs(params["Pt2_angle"] - params["Pt1_angle"])
        else:
            if params["Pt1_angle"] > params["Pt2_angle"]:
                params["Diff"] = 360 - abs(params["Pt1_angle"] - params["Pt2_angle"])
            else:
                params["Diff"] = abs(params["Pt2_angle"] - params["Pt1_angle"])
        # Draw the arc
        if variables["calc_only"] == 0:
            # if check == 0:
            
            if variables["scatter_path"] == 0:
                params["Pt2_angle"] = 0
                # print(params["Diff"])
                draw_ellipse(params, variables)
            else:
                # Produce a scatter path
                params["dir"] = 2
                params, variables = docircle(params, variables)
        else:
            # Calculate the difference between the angles and then using the radius get the length
            params["Distance"] = (math.pi * params["Radius"] * 2) * (params["Diff"] / 360.0)
            params["dir"] = 2
            params, variables = docircle(params, variables)
        record_preview_segment(params, variables, 2, variables["Current_X"], variables["Current_Y"], temp2_x, temp2_y, params["X"], params["Y"])
    variables["Current_X"] = temp2_x
    variables["Current_Y"] = temp2_y
    # Append the centre of the circle to the command as a comment
    # params["Line"] = params["Line"] + " ; " + str(params["Centre_1"])
    return params, variables


def Plotting_G3_2D(params, variables):
    # This function will plot the G3 command using the current line on the plot
    # Check if the command has been used before plotting it, this is used to speed up processing
    if variables["calc_only"] == 0 and variables["Disable_colour_update"] == False:  # Ensure the function is in plotting mode
        params, variables = check_command(params, variables)
        if params["command_check"] == False:
            params["commands_used"][(variables["Current_X"], variables["Current_Y"], params["Line"], params["Positioning"])] = 1
    # Segment the line into seperate cells
    params, variables = segment_line(params, variables)
    # Update the end co-ordinates for the end of the curve ensure that the correct co-ordinate system is used
    if params["Positioning"] == "G90" or params["Positioning"] == "G90 ":
        if math.isnan(params["X_increase"]):
            params["X2"] = variables["Current_X"]
        else:
            params["X2"] = variables["Origin_X"] + params["X_increase"]
        if math.isnan(params["Y_increase"]):
            params["Y2"] = variables["Current_Y"]
        else:
            params["Y2"] = variables["Origin_Y"]  - params["Y_increase"]
    elif params["Positioning"] == "G91" or params["Positioning"] == "G91 ":
        if math.isnan(params["X_increase"]):
            params["X2"] = variables["Current_X"]
        else:
            params["X2"] = variables["Current_X"] + params["X_increase"]
        if math.isnan(params["Y_increase"]):
            params["Y2"] = variables["Current_Y"]
        else:
            params["Y2"] = variables["Current_Y"] - params["Y_increase"]
    # Create a temporary holder to ensure that the next command starts at the right point
    temp3_x = params["X2"]
    temp3_y = params["Y2"]
    params["X1"] = variables["Current_X"]
    params["Y1"] = variables["Current_Y"]
    # Determine the center of the arc given the radius and the start and end points
    q = math.sqrt((params["X2"] - variables["Current_X"]) ** 2 + (params["Y2"] - variables["Current_Y"]) ** 2)
    params["Y3"] = (variables["Current_Y"] + params["Y2"]) / 2
    params["X3"] = (variables["Current_X"] + params["X2"]) / 2

    if q == 0 and params["I_increase"] == 0 and params["J_increase"] == 0:
        return params, variables
    if (params["Line"].find("J", 0, len(params["Line"])) != -1 or params["Line"].find("I", 0, len(params["Line"])) != -1):
        # Determine the radius and center using I and J and then check the radius against both points to check for failure
        params["Radius"] = math.sqrt(params["I_increase"]**2 + params["J_increase"]**2)
        # x and y are the centre of the circle
        params["X"] = variables["Current_X"] + params["I_increase"]
        params["Y"] = variables["Current_Y"] - params["J_increase"]
        params["Centre_1"] = ((params["X"]), (params["Y"]))
        # Error check the radius to ensure that it is physcially possible
        if radius_check(variables["Current_X"], variables["Current_Y"], params["X2"], params["Y2"],  params["Centre_1"]) == -1:
            if variables["radius_error_output"]:
                if variables["radius_error_fix"]:
                    # Fix the radius error by correcting the centre using the distance between the arcs
                    params["Center_1"] = ((params["X3"]), (params["Y3"]))
                    variables["edited_flag"] = True
                    params["Edited_line"] = "G2 X" + str(params["X_increase"] / variables["scale"]) + " Y" + str(params["Y_increase"] / variables["scale"]) + " I" + str(round((params["X2"] - variables["Current_X"])/ 2) / variables["scale"]) + " J" + str(round((variables["Current_Y"] - params["Y2"]) / 2) / variables["scale"])
                    # print("editied",edited_line)
            if variables["Display_radius_error"]:
                print(params["Line"])
                error_message(-5)
    else:
        # https://lydxlx1.github.io/blog/2020/05/16/circle-passing-2-pts-with-fixed-r/
        params["X"] = params["X3"] - math.sqrt((params["Radius"]**2) - ((q / 2) ** 2)) * (variables["Current_Y"] - params["Y2"]) / q
        params["Y"] = params["Y3"] - math.sqrt((params["Radius"]**2) - ((q / 2) ** 2)) * (params["X2"] - variables["Current_X"]) / q
        # Determine the center of the arc
        params["Centre_1"] = ((params["X"]), (params["Y"]))
    # if variables["calc_only"] == 0:
    #     temp = params["Radius"]
    #     params["Radius"] = 1
    #     temp_colour = params["Plotting_colour"]
    #     params["Plotting_colour"] = [255,50,120]
    #     draw_circle(params, variables)
    #     params["Radius"] = temp
    #     params["Plotting_colour"] = temp_colour
    if q == 0:
        # This is a full circle so the start and end pos are the same
        if variables["calc_only"] == 0:
            if variables["scatter_path"] == 0:
                draw_circle(params,variables)
            else:
                params["dir"] = 3
                params,variables = docircle(params,variables, flag=1)       
        else:
            # Calculate the distance of the circle and then call the docircle function to get the pixel coordinates
            params["Distance"] = math.pi * params["Radius"] * 2
            params["dir"] = 3
            params, variables = docircle(params, variables, flag=1)
        record_preview_segment(params, variables, 3, variables["Current_X"], variables["Current_Y"], temp3_x, temp3_y, params["X"], params["Y"], full_circle=True)
    else:
        # Determine the start and end angle of the arc
        if params["Line"].find("J", 0, len(params["Line"])) != -1 or params["Line"].find("I", 0, len(params["Line"])) != -1:
            params["Pt1_angle"] = (180 * np.arctan2(variables["Current_Y"] - (variables["Current_Y"] - params["J_increase"]), variables["Current_X"] - (variables["Current_X"]  + params["I_increase"])) / np.pi)
            params["Pt2_angle"] = (180 * np.arctan2(params["Y2"] - (variables["Current_Y"] - params["J_increase"]), params["X2"] - (variables["Current_X"]  + params["I_increase"])) / np.pi)
        else:
            params["Pt1_angle"] = 180 * np.arctan2(variables["Current_Y"] - params["Y"], variables["Current_X"]  - params["X"]) / np.pi
            params["Pt2_angle"] = 180 * np.arctan2(params["Y2"] - params["Y"], params["X2"] - params["X"]) / np.pi
            # https://stackoverflow.com/questions/36211171/finding-center-of-a-circle-given-two-points-and-radius
        # As we are plotting using an ellipse we want to set both the major and minor axis the samediff
        params["Axes"] = ((params["Radius"]), (params["Radius"]))
        # Adjust the start and end angle based on the sign of the pt1 and pt2 angles to account for different orientations of the arc
        if params["Pt1_angle"] < 0 and params["Pt2_angle"] > 0:
            if params["Pt1_angle"] > params["Pt2_angle"]:
                params["Diff"] = 360 - abs(params["Pt1_angle"] - params["Pt2_angle"])
            else:
                params["Diff"] = 360 - abs(params["Pt2_angle"] - params["Pt1_angle"])
        elif params["Pt1_angle"] > 0 and params["Pt2_angle"] > 0:
            if params["Pt1_angle"] > params["Pt2_angle"]:
                params["Diff"] = abs(params["Pt1_angle"] - params["Pt2_angle"])
            else:
                params["Diff"] = 360 - abs(params["Pt2_angle"] - params["Pt1_angle"])
        elif params["Pt1_angle"] < 0 and params["Pt2_angle"] < 0:
            if params["Pt1_angle"] > params["Pt2_angle"]:
                params["Diff"] = abs(params["Pt1_angle"] - params["Pt2_angle"])
            else:
                params["Diff"] = 360 - abs(params["Pt2_angle"] - params["Pt1_angle"])
        else:
            if params["Pt1_angle"] > params["Pt2_angle"]:
                params["Diff"] = abs(params["Pt1_angle"] - params["Pt2_angle"])
            else:
                params["Diff"] = abs(params["Pt2_angle"] - params["Pt1_angle"])
        # Draw the arc
        if variables["calc_only"] == 0:
            # if check == 0:
            # draw_circle(image, center, int(1 * 5), plotting_colour)
            if variables["scatter_path"] == 0:
                params["Pt1_angle"] = params["Pt2_angle"]
                params["Pt2_angle"] = 0
                draw_ellipse(params, variables)
            else:
                params["dir"] = 3
                params, variables = docircle(params, variables)
        else:
            # Calculate the difference between the angles and then using the radius get the length
            params["Distance"] = (math.pi * params["Radius"] * 2) * (params["Diff"] / 360.0)
            params["dir"] = 3
            params, variables = docircle(params, variables)
        record_preview_segment(params, variables, 3, variables["Current_X"], variables["Current_Y"], temp3_x, temp3_y, params["X"], params["Y"])
    variables["Current_X"] = temp3_x
    variables["Current_Y"] = temp3_y
    return params, variables

# Functions for outputting img and scaffold outputs


def segment_line(params, variables):
    # Strip any square bracket macros before tokenising so their bracketed text is
    # never mistaken for motion parameters such as X/Y/I/J/F values.
    if is_macro_line(params["Line"]):
        params["Command_array"] = []
        params["Command_flag"] = ""
        params["Command_number"] = 0
        return params, variables
    params["Line"] = re.sub(r"\[[^\]]*\]", "", params["Line"])
    params["Command_array"] = (re.findall(r"[^\W\d_]+|[-+]?(?:\d*\.*\d+)", params["Line"]))
    # Check the array length and make sure that it is even otherwise throw an error
    if (len(params["Command_array"]) % 2 == 1):
        # Length is off throw an error and pass it the line
        print(params["Command_array"])
        error_message(-11)
    for i in range(0,len(params["Command_array"]),2):
        var = params["Command_array"][i]
        match var:
            # Converts everything to µm
            case "X":
                params["X_increase"] = round(float(params["Command_array"][i+1]) * variables["scale"],2)
            case "Y":
                params["Y_increase"] = round(float(params["Command_array"][i+1]) * variables["scale"],2)
            case "R":
                params["Radius"] = round(float(params["Command_array"][i+1]) * variables["scale"],2)
            case "I":
                params["I_increase"] = round(float(params["Command_array"][i+1]) * variables["scale"],2)
            case "J":
                params["J_increase"] = round(float(params["Command_array"][i+1]) * variables["scale"],2)
            case "Z":
                params["Z_increase"] = round(float(params["Command_array"][i+1]) * variables["scale"],2)
            case "A":
                # Mandrel rotation (degrees): with a mandrel diameter set it is the distance
                # round the tube's surface, unrolled flat as Y (mandrel.py)
                if variables.get("Mandrel_diameter_mm", 0) > 0:
                    params["Y_increase"] = round(float(params["Command_array"][i+1]) * math.pi * variables["Mandrel_diameter_mm"] / 360.0 * variables["scale"], 2)
            case "E":
                params["E_increase"] = round(float(params["Command_array"][i+1]) * variables["scale"],2)
            case "P":
                params["P_value"] = float(params["Command_array"][i+1])
            case "S":
                params["S_value"] = round(float(params["Command_array"][i+1]) * variables["scale"],2)
            case "F":
                # Feed rate will now only be found attached to another command, G1, G2, G3
                params["Feed_rate"] = round(float(params["Command_array"][i+1]),2)
                params["Feed_rate"] = round((params["Feed_rate"] / 60) * variables["Feed_rate_to_mach_3_conversion_factor"], 10)
            case "G":
                params["Command_number"] = float(params["Command_array"][i+1])
                params["Command_flag"] = "G"
            case "M":
                params["Command_number"] = float(params["Command_array"][i+1])
                params["Command_flag"] = "M"
    letters = params["Command_array"][0::2]
    if (variables.get("Mandrel_diameter_mm", 0) > 0 and "F" in letters and "A" in letters
            and not any(k in letters for k in ("X", "Y", "Z"))):
        # A rotation on its own takes F in degrees / min (Mach3): as surface speed
        params["Feed_rate"] = params["Feed_rate"] * math.pi * variables["Mandrel_diameter_mm"] / 360.0
    return params, variables


def line_reader(params, variables):
    # This function is designed to be called by any path planning or motion calculation function to standardise the output
    
    # Add the other parameters for the functions and pass through these depending on the input function
    # By knowing the current feedrate and being given the acceleration the program should be able to determine the total time and path length
    # Change this to use the style for params["Line"] reading from the G1 function to streamline the process and allow for more commands in the future
    
    if is_macro_line(params["Line"]):
        return params, variables
    
    params["X_increase"] = float('NaN')
    params["Y_increase"] = float('NaN')
    params["Z_increase"] = float('NaN')
    params["E_increase"] = float('NaN')
    params["S_value"] = float('NaN')
    params["P_value"] = float('NaN')
    params["I_increase"] = 0
    params["J_increase"] = 0
    params["Radius"] = 0
    params["Command_flag"] = ""
    params["Command_number"] = 0
    params["Edited_line"] = ""
    # Reset so a command that doesn't move (a bare "G3", "G1 F320", a zero-length arc)
    # adds 0 to the distance/time totals instead of re-adding the previous move's length
    params["Distance"] = 0
    # Segment the params["Line"] into seperate cells
    params, variables = segment_line(params, variables)

    # One_coordinate_system command format
    # G1 X# Y# F# ; prev_x prev_y command_num
    # G2 X# Y# R# F# ; prev_x prev_y cent_x cent_y command_num
    decimal_place = 5
    # print(command_Array)
    # Sort the commands by G and M, this will then remove issues due to Micheals lack of F commands not being on a params["Line"] with G first
    # Temporary store length of pixel cords
    temp_length_pixel_coords = len(params["Pixel_coords_um"]) 
    if params["Command_flag"] == "G":
        if params["Command_number"] == 92:
        # Altered to allow individual G92 commands to be used for each axis, this will allow for the use of G92.1 to reset the origin to the original machine coordinates
            if params["Line"].find("X", 0, len(params["Line"])) != -1:
                variables["Origin_X_G92"] =  variables["Origin_X"]
                variables["Origin_X"] = variables["Current_X"] - params["X_increase"]
            if params["Line"].find("Y", 0, len(params["Line"])) != -1:
                variables["Origin_Y_G92"] =  variables["Origin_Y"]
                variables["Origin_Y"] = variables["Current_Y"] + params["Y_increase"]
                        
        if params["Command_number"] == 92.1:
        # Reset the G92 command to original machine global coordinates
            variables["Origin_X"] = variables["Origin_X_G92"]
            variables["Origin_Y"] = variables["Origin_Y_G92"]
            
        if params["Command_number"] == 4 and variables["calc_only"] == 1:
            # Dwell - the machine stops here for this long. Recorded as
            # (segments so far, seconds, commands so far, line) for plan_motion() / generate_pixel_coords()
            if not math.isnan(params["P_value"]):
                dwell_seconds = params["P_value"] / 1000.0 if variables["Dwell_P_is_ms"] else params["P_value"]
            elif not math.isnan(params["S_value"]):
                dwell_seconds = params["S_value"] / variables["scale"]
            else:
                dwell_seconds = 0.0
            params["Dwells"].append((len(params["Preview_segments"]), dwell_seconds, len(params["One_coordinate_system"]), params.get("Preview_line_number", 0)))
        # Store the current params["Positioning"] system absolute or incremental to be passed to the plotting fucntions
        if params["Command_number"] == 90:
            params["Positioning"] = "G" + str(int(params["Command_number"]))
        if params["Command_number"] == 91:
            params["Positioning"] = "G" + str(int(params["Command_number"]))
        # print(params["Positioning"])
        # print(params["Line"])
        if params["Command_number"] == 1 or params["Command_number"] == 0:
            # Found a G0 or G1 command
            # New command structure, just pass it the command array instead
            # If line only contains a feedrate command then just update the feedrate and return
            # if "F" in params["Command_array"] and len(params["Command_array"]) == 4:
            #     return params, variables
            params, variables = Plotting_G1_2D(params, variables)  # Has to be a plus 3 to compensate for the index being at the start of the "G1 "
            if variables["calc_only"] == 1:
                # Update the overall array's
                params["Current_X_array"].append(variables["Current_X"] / variables["scale"])
                params["Current_Y_array"].append(variables["Current_Y"] / variables["scale"])
                # New array that will transpose everything to G90 
                temp_One_coordinate_system = ("G1 X" + str(round(variables["Current_X"] / variables["scale"],decimal_place)) + " Y" + str(round(variables["Current_Y"] * -1 / variables["scale"],5)) + " F" + str(round(params["Feed_rate"] * 60,decimal_place)) 
                + " ; " + str(round(params["Centre_1"][0] / variables["scale"],decimal_place)) + " " + str(round(params["Centre_1"][1] * -1 / variables["scale"],decimal_place)) + " " + str(int(params["Command_number"])))
                params["One_coordinate_system"].append(temp_One_coordinate_system)
                if variables["high_speed"] == False:
                    # A command that produced no pixel coords (e.g. a G2/G3 that doesn't move)
                    # gets a blank entry rather than indexing past the end of the list
                    if len(params["Pixel_coords_um"]) == temp_length_pixel_coords:
                        params["Pixel_coords_um"].append([variables["Current_X"], variables["Current_Y"], "", 0])
                    params["Pixel_coords_um"][temp_length_pixel_coords][2] = temp_One_coordinate_system
                    # Used in lag vector to set the length of the computation without having to calculate it
                    params["Pixel_coords_um"][temp_length_pixel_coords][3] = (len(params["Pixel_coords_um"]) - temp_length_pixel_coords)
                params["Distance_array"].append((params["Distance"] / variables["scale"]))  # mm / scale
                params["Filament_array"].append(round((params["E_increase"] / variables["scale"]),5))  # mm / scale
                # Compute the current time to complete the command using the feedrate
                if params["Feed_rate"] > 0:
                    params["Time_array"].append((params["Distance"] / variables["scale"]) / params["Feed_rate"])  # mm / (mm/s)
        if params["Command_number"] == 2:
            # Found a G2 command
            # Section the params["Line"] to remove the G2 command before sending it through to the plotting function
            params, variables = Plotting_G2_2D(params, variables)
            if variables["calc_only"] == 1:
                # Update the overall array's
                params["Current_X_array"].append(variables["Current_X"] / variables["scale"])
                params["Current_Y_array"].append(variables["Current_Y"] / variables["scale"])
                # New array that will transpose everything to G90 
                temp_One_coordinate_system = ("G2 X" + str(round(variables["Current_X"]  / variables["scale"],decimal_place)) + " Y" + str(round(variables["Current_Y"] * -1 / variables["scale"],decimal_place)) + " I" + str(round(params["I_increase"] / variables["scale"],decimal_place)) + " J" + str(round(params["J_increase"] / variables["scale"],decimal_place)) + " F" + str(round(params["Feed_rate"] * 60, decimal_place)) 
                + " ; " + str(round(params["X1"] / variables["scale"],decimal_place)) + " " + str(round(params["Y1"] * -1 / variables["scale"],decimal_place)) + " " + str(round(params["Centre_1"][0] / variables["scale"],decimal_place)) + " " + str(round(params["Centre_1"][1] * -1 / variables["scale"],decimal_place)) + " " + str(round(params["Diff"],decimal_place)) + " " + str(int(params["Command_number"])))
                params["One_coordinate_system"].append(temp_One_coordinate_system)
                if variables["high_speed"] == False:
                    # A command that produced no pixel coords (e.g. a G2/G3 that doesn't move)
                    # gets a blank entry rather than indexing past the end of the list
                    if len(params["Pixel_coords_um"]) == temp_length_pixel_coords:
                        params["Pixel_coords_um"].append([variables["Current_X"], variables["Current_Y"], "", 0])
                    params["Pixel_coords_um"][temp_length_pixel_coords][2] = temp_One_coordinate_system
                    # Used in lag vector to set the length of the computation without having to calculate it
                    params["Pixel_coords_um"][temp_length_pixel_coords][3] = (len(params["Pixel_coords_um"]) - temp_length_pixel_coords)
                params["Distance_array"].append((params["Distance"] / variables["scale"]))  # mm / scale
                params["Filament_array"].append((params["E_increase"] / variables["scale"]))  # mm / scale
                # Compute the current time to complete the command using the feedrate
                if params["Feed_rate"] > 0:
                    params["Time_array"].append((params["Distance"] / variables["scale"]) / params["Feed_rate"])  # mm / (mm/s)
        if params["Command_number"] == 3:
            # Found a G3 command
            # Section the params["Line"] to remove the G3 command before sending it through to the plotting function
            params, variables = Plotting_G3_2D(params, variables)
            if variables["calc_only"] == 1:
                # Update the overall array's
                params["Current_X_array"].append(variables["Current_X"] / variables["scale"])
                params["Current_Y_array"].append(variables["Current_Y"] / variables["scale"])
                # New array that will transpose everything to G90 
                temp_One_coordinate_system = ("G3 X" + str(round(variables["Current_X"]  / variables["scale"],decimal_place)) + " Y" + str(round(variables["Current_Y"] * -1 / variables["scale"],decimal_place)) + " I" + str(round(params["I_increase"] / variables["scale"],decimal_place)) + " J" + str(round(params["J_increase"] / variables["scale"],decimal_place)) + " F" + str(round(params["Feed_rate"] * 60, decimal_place)) 
                + " ; " + str(round(params["X1"] / variables["scale"],decimal_place)) + " " + str(round(params["Y1"] * -1 / variables["scale"],decimal_place)) + " " + str(round(params["Centre_1"][0] / variables["scale"],decimal_place)) + " " + str(round(params["Centre_1"][1] * -1 / variables["scale"],decimal_place)) + " " + str(round(params["Diff"],decimal_place)) + " " + str(int(params["Command_number"]))) #params["Command_number"]
                params["One_coordinate_system"].append(temp_One_coordinate_system)
                if variables["high_speed"] == False:
                    # A command that produced no pixel coords (e.g. a G2/G3 that doesn't move)
                    # gets a blank entry rather than indexing past the end of the list
                    if len(params["Pixel_coords_um"]) == temp_length_pixel_coords:
                        params["Pixel_coords_um"].append([variables["Current_X"], variables["Current_Y"], "", 0])
                    params["Pixel_coords_um"][temp_length_pixel_coords][2] = temp_One_coordinate_system
                    # Used in lag vector to set the length of the computation without having to calculate it
                    params["Pixel_coords_um"][temp_length_pixel_coords][3] = (len(params["Pixel_coords_um"]) - temp_length_pixel_coords)
                params["Distance_array"].append((params["Distance"] / variables["scale"]))  # mm * scale
                params["Filament_array"].append((params["E_increase"] / variables["scale"]))  # mm * scale
                # Compute the current time to complete the command using the feedrate
                if params["Feed_rate"] > 0:
                    params["Time_array"].append((params["Distance"] / variables["scale"]) / params["Feed_rate"])  # mm / (mm/s)
    # Update the previous feedrate to be used in the next command
    params["Feed_rate_previous"] = params["Feed_rate"]
    return params, variables
