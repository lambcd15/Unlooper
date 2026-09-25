"""The 'Scaffold outputs' pass that ties the stages together: run every command through
the toolpath, total the distance / time / material, size the build plate, run the planner,
the pixel coords, the corner-rounded path and the lag model, and write the results back."""
import math

import numpy as np

from .common import format_duration
from .corner_path import generate_corner_path
from .gcode_reader import parameters_extraction
from .motion_planner import plan_motion
from .pixel_coords import generate_pixel_coords
from .toolpath import line_reader


def motion_calculations(params, variables):
    # Variables for the new start location
    variables["Current_X"] = 0
    variables["Current_Y"] = 0
    variables["Origin_X"] = 0
    variables["Origin_Y"] = 0
    # Make an array for all the variables["Current_X"] and variables["Current_Y"] coordinates
    params["Current_X_array"] = []
    params["Current_Y_array"] = []
    params["Distance_array"] = []
    params["Filament_array"] = []
    params["Time_array"] = []
    # Feed rate will be used to calculate the total time of the code
    params["Feed_rate"] = 1
    # Which positioning system is being used?
    params["Positioning"] = []        
    # Determine the parameters used within the parameter table
    params, variables = parameters_extraction(params, variables)
    if variables.get("CTS_override_mm_min", 0) > 0:
        # User-supplied critical translation speed overrides the file's
        variables["global_return_CTS"] = float(variables["CTS_override_mm_min"])
    params["G2_G3_Edited_output"] = []
    # line_num = 0
    variables["calc_only"] = 1
    total_lines = max(len(params["Unlooped_contents"]), 1)
    last_percent = -1
    for preview_line_number, line in enumerate(params["Unlooped_contents"]):
        # Progress for the GUI (and console), once per whole percent
        percent = int(100 * (preview_line_number + 1) / total_lines)
        if percent != last_percent:
            print("Scaffold outputs progress:", str(percent) + "%")
            last_percent = percent
        # Loop through all the lines in the edited contents array
        params["Line"] = line
        params["Preview_line_number"] = preview_line_number
        params, variables = line_reader(params, variables)
        if params["Edited_line"] != "":
            params["G2_G3_Edited_output"].append(params["Edited_line"])
        else:
            params["G2_G3_Edited_output"].append(line)
        
    # New feature for adapting any bad G2 and G3 commands is to alter the lines and then save the files
    if variables["edited_flag"]:
        # If the system has edited a line - save all the contents to a new output file
        for line in params["G2_G3_Edited_output"]:
            params["Edit_Output"].write("%s\n" % str(line))
        # f.write(f"{pixel_cords[i]}\n")
        params["Edit_Output"].close()
    # Determine the direction of the print and then the size to ensure that it remains in frame
    
    min_x = min(params["Current_X_array"])
    max_x = max(params["Current_X_array"])
    variables["min_x"] = min_x
    variables["max_x"] = max_x
    min_y = min(params["Current_Y_array"])
    max_y = max(params["Current_Y_array"])
    variables["min_y"] = min_y
    variables["max_y"] = max_y
    # Can calculate the size of the scaffold and determine which corner I am calculating from
    # Need to determine which orientation the scaffold is from zero value and correct the current x and y

    variables["Current_X"] = abs(min_x) + 1
    variables["Current_Y"] = abs(min_y) + 1
    
    variables["Origin_X"] = variables["Current_X"] * variables["scale"]
    variables["Origin_Y"] = variables["Current_Y"] * variables["scale"]
    # This will only track the last G92 command not the first which is used for later plotting
    # Addition of new variable for original origin (the first one)
    if variables["First_run"] == False:
        variables["First_Origin_X"] = variables["Current_X"] * variables["scale"]
        variables["First_Origin_Y"] = variables["Current_Y"] * variables["scale"]
        variables["First_run"] = True
    # Entire Print calculations
    # Determine the linear distance travelled and use it to compute the approximate time
    total_distance_mm = sum(params["Distance_array"])
    # print(params["Filament_array"])
    total_filament_used_mm = np.nansum(params["Filament_array"])
    print("Distance travelled:", round(total_distance_mm / 1000, 3), "m")
    if variables["Feedrate_override_mm_min"] > 0:
        print("Filament used:", round(total_filament_used_mm, 3), "mm")
    # Time array is in seconds
    if variables["Feedrate_override_mm_min"] > 0:
        # User-supplied feedrate overrides whatever feed rates are written in the file
        total_seconds = total_distance_mm / variables["Feedrate_override_mm_min"] * 60
    else:
        total_seconds = sum(params["Time_array"])
    seconds = round(total_seconds)
    day = seconds // (24 * 3600)
    seconds = seconds % (24 * 3600)
    hour = seconds // 3600
    seconds %= 3600
    minutes = seconds // 60
    seconds %= 60
    # Display the time to console
    print("Total Time:", day, "day", hour, "hr", minutes, "min", seconds, "s")
    variables["Estimated_Time_accel"] = ""
    if variables["Acceleration_mm_s2"] > 0:
        print("Planning motion (acceleration", variables["Acceleration_mm_s2"], "mm/s^2, junction deviation", variables["Junction_deviation_mm"], "mm, jerk", variables.get("Jerk_mm_s", 0), "mm/s)...")
        accel_seconds = plan_motion(params, variables)
        variables["Estimated_Time_accel"] = format_duration(accel_seconds)
        print("Total Time (accel/junction):", variables["Estimated_Time_accel"])
        if variables["Dwell_time_s"] > 0:
            print("Dwell time (G4, included above):", round(variables["Dwell_time_s"], 3), "s")
        if accel_seconds > 0:
            print("Average speed (accel/junction):", round(total_distance_mm / accel_seconds * 60, 2), "mm/min")
        if variables["global_return_CTS"] > 0:
            below = variables["Distance_below_CTS_mm"]
            print("Path below CTS:", round(below / 1000, 3), "m", "(" + str(round(100 * below / max(total_distance_mm, 1e-9), 2)) + "%)")
    # Correction factor for the material due to differences in weight of the fibre and the actual volume of material used in the print. 
    correction_factor = 1.0 #? 1.25
    # Volume in cm^3
    if variables["Fibre_Diameter_override"] != 0 and variables["Material_Density_override"] != 0:
        # User-supplied fibre diameter and material density overrides whatever values are written in the file
        volume = (((variables["Fibre_Diameter_override"] * 0.001 / 2) ** 2 * math.pi) * total_distance_mm) * 0.001 #mm3 the 0.001 is to convert to cm3
        print("Volume Used: ",round(volume, 4),"ml",)
        material_mass = volume * variables["Material_Density_override"]  # cm3 * g/cm3 to get grams
        variables["Material_Used"] = round(material_mass * correction_factor, 5) * 1000 #
    elif variables["Fibre_Diameter"] != 0 and variables["Material_Density"] != 0:
        volume = (((variables["Fibre_Diameter"] * 0.001 / 2) ** 2 * math.pi) * total_distance_mm) * 0.001 #mm3 the 0.001 is to convert to cm3
        print("Volume Used: ",round(volume, 4),"ml",)
        material_mass = volume * variables["Material_Density"]  # cm3 * g/cm3 to get grams
        variables["Material_Used"] = round(material_mass * correction_factor, 5) * 1000 #
    else:
        variables["Material_Used"] = 0
    if variables["Material_Used"] > 0:
        print("Material Used: ",round(round(variables["Material_Used"], 4), 10),"mg",)
    # Get the out;uts ready to save to the ext file
    variables["Estimated_Time"] = (str(day)+ " day "+ str(hour)+ " hr "+ str(minutes)+ " min "+ str(seconds)+ " s")

    variables["X_build"] = round((abs(variables["min_x"]) + abs(variables["max_x"])) + 2)  # mm
    if variables["min_y"] < 0 and variables["max_y"] < 0:
        # This is to solve issues with having a build plate that is longer than the print due to Two negative values
        variables["Y_build"] = math.ceil(abs(variables["min_y"])) + 2  # mm
    else:
        variables["Y_build"] = math.ceil(abs(variables["min_y"]) + abs(variables["max_y"])) + 2  # mm
    print("Total size used x:", (variables["X_build"] - 2), "y:", (variables["Y_build"] - 2))
    # Pixel coords (needs the plate size above for the PNGs)
    if variables["Acceleration_mm_s2"] > 0:
        if variables["Motion_pixel_coords"]:
            generate_pixel_coords(params, variables)
        elif not variables["Generate_pixel_coords"]:
            print("Pixel coords skipped")
        # Corner-rounded path: written with the lag-format files, and what the lag model follows
        write_corner = variables["Generate_pixel_coords"] and variables["high_speed"] == False
        if write_corner or variables.get("Lag_prediction", False):
            consumers = []
            if variables.get("Lag_prediction", False):
                from .lag_model import JetLagModel
                consumers.append(JetLagModel(params, variables))
            generate_corner_path(params, variables, consumers)
            for consumer in consumers:
                consumer.finish()
    elif variables.get("Lag_prediction", False):
        print("Lag prediction skipped - it needs Acceleration > 0 (it follows the planned motion)")
    # Save the pixel coordinates to a file
    if variables["high_speed"] == False and not variables["Motion_pixel_coords_written"]:
        for i in range(len(params["Pixel_coords_um"])):
            params["Pixel_File"].write("%s\n" % str(params["Pixel_coords_um"][i])[1:-1])
            # f.write(f"{pixel_cords[i]}\n")
        # params["Pixel_File"].close()

    return params, variables


def save_outputs(params,variables):
    # This function is designed to append the print time and filament usage to the file for easy viewing before printing
    # To do this the parameter_line needs to be known in the true file posistion, then the file needs to be open and these added or changed

    # re-open unlooped code text file
    # Insert at the top of the file the time and material used
    with open(("Output/" + params["Filename_only"] + "/" + params["Filename_only"] + "_Unlooped_Code.txt"), 'r+') as f:
        contents = f.read()
        f.seek(0, 0)
        # Add the total time and material used
        f.write("; " + "Estimated_Time: " + str(variables["Estimated_Time"]) + "\n")
        f.write("; " + "Total size used x: " + str(variables["X_build"] - 2) + " mm "+ "y: " + str(variables["Y_build"] - 2) + " mm "+ "\n")
        if variables.get("Estimated_Time_accel"):
            f.write("; " + "Estimated_Time_accel: " + variables["Estimated_Time_accel"] + " (a=" + str(variables["Acceleration_mm_s2"]) + " mm/s^2, junction deviation=" + str(variables["Junction_deviation_mm"]) + " mm, jerk=" + str(variables.get("Jerk_mm_s", 0)) + " mm/s)" + "\n")
        f.write("; " + "Material_Used: " + str(variables["Material_Used"]) + " mg" + "\n" + contents)
    f.close()

    variable_names = ["Estimated_Time", "Material_Used"]
    # Do the same for the input file - always as the first lines of the file, in a fixed
    # order. Any %@ output lines from a previous run are removed first (wherever they
    # ended up) so they are replaced rather than duplicated.
    with open(params["Filename"], mode="r+") as f:
        contents = f.readlines()

    contents = [line for line in contents
                if not (line.lstrip().startswith("%@") and any(name in line for name in variable_names))]
    header = [
        "%@ " + "Estimated_Time: " + str(variables["Estimated_Time"]) + "\n",
        "%@ " + "Material_Used: " + str(variables["Material_Used"]) + " mg" + "\n",
    ]
    contents = header + contents

    with open(params["Filename"], "w") as f:
        contents = "".join(contents)
        f.write(contents)

# Functions for unlooping code
