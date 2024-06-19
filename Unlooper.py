import cv2
import psutil
import os
import sys
from msvcrt import getch

# https://stackoverflow.com/questions/42660670/collapse-all-methods-in-visual-studio-code
# crtl + k, crtl + 1
# Git commands:
# git pull
# git commmit -a -m "Message"
# git push

# Commands for running in command prompt
# python Gcode_processing.py "filename"

# *******************************************************************************************************************************************
# Multi-processing function calls
# Structure for point in cartesian plane.
if __name__ == "__main__":
    # *******************************************************************************************************************************************
    # Library
    # import cv2
    import numpy as np
    from matplotlib import pyplot as plt
    import math
    # import datetime
    # import scipy.spatial as spatial
    # from scipy.optimize import curve_fit
    # from ipywidgets import interact, interactive, fixed, interact_manual
    # import ipywidgets as widgets
    # import os
    # from subpixel_edges import edgepixel, subpixel_edges
    # import argparse #For sending commands via serial
    import time  # For determination of current time and for speed of program
    import copy
    # import PIL
    from sympy import false  # For displaying the gcode on an image
    import re
    from matplotlib import cm
    # from collections import OrderedDict
    # import time #For determination of current time and for speed of program
    import multiprocessing as mp  # For multi-core processing
    from multiprocessing import Pool
    import concurrent.futures
    # import numpy as np
    from concurrent.futures import as_completed
    from sys import getsizeof
    # import glob
    from moviepy.editor import VideoFileClip, concatenate_videoclips
    from natsort import natsorted
    # from PIL import Image, ImageOps
    from tqdm import tqdm
    import mmap
    import subprocess
    import pandas as pd
    from Ievgenii.python_lag import *

    # ************************************   Variables    ******************************************
    variables = {
        "Start_time": time.time(),
        "Previous_time": time.time(),
        "Current_X": 0, #Starting co-ordinates
        "Current_Y": 0,
        "Origin_X": 0,
        "Origin_Y": 0,
        "Origin_X_G92": 0, # These are to allow the restoration of global co-ordinates using G92
        "Origin_Y_G92": 0,
        # To remove error due to multiple G92 commands being used
        "First_Origin_X": 0,
        "First_Origin_Y": 0,
        # First run boolean to stop errors
        "First_run": False,
        "X_build": 100, # Build plate / image size
        "Y_build": 100,
        "resolution_x": 1080, # Output video size
        "resolution_y": 1080,
        "Line_width": 2, # Line width
        # Colours
        "Background_colour": (255, 255, 255), # White
        "G1_colour": (255, 0, 0),
        "G2_colour": (0, 0, 255),
        "G3_colour": (0, 255, 0),
        "G4_colour": (120, 0, 120),
        "Disable_colour_update": False,
        "all_black": 1, # This variable will set the outputs to disable colour making it all_black

        "scale": 1000, # scale for calculations to put units in the requried format
        "scatter_resolution": 0.001,  # in (s)
        # scatter_path is to show the path as a series of points for videos and direction plotting
        # If zero show the path as opencv lines if 1 show as scatter points
        "scatter_path": 0,  # Do I want everything to be made from multiple G1 commands, required for everything is G1
        "scatter_size": 1,  # Size of the G1 line
        # Define the resolution higher is more coarse
        
        # to increase speed and reduce issues high_speed will not output pixel_coords
        "high_speed": True,
        # To permit different codes to be used currently beyond the scale set unloop_only to 1 and use ncviewer.com to view the unlooped files
        # This does not perform time and distance calculations
        "unloop_only": 0,
        # Generate image output
        "Generate_output_image": True,
        # Display image to screen
        "Display_image": False,
        # Errors
        # Ignore the radius math error for the G2 and G3 commands, if True this will break point the code
        "Display_radius_error": False,
        # Do not output the errors to console, if true this will only log the errors to console
        "radius_error_output": True,
        # Global editing flag, this is if the code has had to make changes to the file to correct gcode this flag will be updated
        "edited_flag": False,
        "radius_error_fix": True,

        "global_return_feedrate": 0,
        "global_return_CTS": 0,
        # This is the array that is searched for parameter extraction
        "Variable_names": ["SyringeTemperature","NeedleTemperature","BuildPlateTemperature","AppliedVoltage","AppliedPressure","FibreDiameter","MaterialDensity","CriticalTranslationSpeed","Speed_Ratio"],

        "segment": 0,
        "calc_only": 1, # If this flag is true (1) the system will not plot anything
        "Feed_rate_to_mach_3_conversion_factor": 1, #1.12

        "Total_Distance": 0.0, # This is to store the total distance travelled
        "Estimated_Time": 0.0,
        "Material_Used": 0.0, # grams

        # Lag compensation
        "compensation_image": [],
        "compensation_complete": False # This will change after the lag has been compensated
    }
    
    # ************************************ User Variables ******************************************
    # Commands you can edit
    # Setting to 0 will unloop and generate an image
    # Setting to 1 will unloop only and not generate an image
    variables["unloop_only"] = 1
    # gcode filename enter in your own filename below
    lag_length = 0
    if len(sys.argv) < 2:
        # Everything must contain forward slashes only
        file_name = "TXT Files/DO_4 x10.txt"
    else:
        # collector speed [mm/min]
        file_name = sys.argv[1]

    # ************************************ Functions ******************************************
    # Functions for reading in gcode:
    # Extract the filename from the input, permit this to work by backwards scanning to allow for different names
    # Remove the file extension
    split_path = os.path.splitext(file_name)[0]

    file_name_only = os.path.basename(split_path)  # This works for any path variation
    # Do not touch
    params = {
        "Filename": file_name,
        "Filename_only": file_name_only,
        "File_length": "",
        "Text_File": "",
        "Pixel_File": "",
        "Edit_Output": "",
        "Image_name": "",
        "Image": [],
        # Array's
        "File_contents": [],
        "File_contents_edited": [], # This can be updated with the latest functions edit
        "G2_G3_Edited_output": [], # This is to house the edited G2 or G3 contents
        "Parameters": [], # This is used to house the parameters array for all the parameters found from the text file
        "Parameters_line_array": [], # Stores the raw reads from the text file that contain parameters
        "M98_Array": [],
        "M99_Array": [],
        "O_Array": [],
        "Unlooped_contents": [],
        "Pixel_coords": [],
        "Pixel_coords_um": [],
        "One_coordinate_system": [], # This is to contain all commands transposed to G90 (absolute)
        # The following commands are for reduction in drawing time
        "commands_used": [],
        "command_check": False, # This is true if the command has been used before
        "commands_used_counter": [], # This array stores all the commands that have been plotted already
        "Current_X_array": [], # This array is a record of all the x commands 
        "Current_Y_array": [],
        "Distance_array": [], # array for all the distances that each command is
        "Distance": 0.0, # This is the previous feedrate from the last command
        "Distance_from_previous": 0.0, # This is the distance after the previous command
        "Time_array": [], # for cupouting the time taken for each move
        # Per line commands, these should be zeroed after each line has been processed
        "Positioning": [],
        "Line": [],
        "Edited_line": "", # This is used when the G2 or G3 commands do not line up correctly and have been edited
        "X_increase": float('NaN'),
        "Y_increase": float('NaN'),
        "Z_increase": float('NaN'),
        "Radius": float('NaN'),
        "I_increase": float('NaN'),
        "J_increase": float('NaN'),
        "Feed_rate": 0.0,
        "Feed_rate_previous": 0.0, # This allows for pixel_coords to space out the points when changing speed between commands
        "Command_number": float('NaN'),
        "Command_flag": "",
        "Command_array": [],
        "Pt1_angle": 0,
        "Pt2_angle": 0,
        "Diff": 0.0,
        "Angle": 0.0,
        "Centre": [],
        "Centre_1": [],
        "Centre_2": [],
        "Axes": [],
        "dir": 0,
        "Plotting_colour": [],
        "X": 0.0,
        "Y": 0.0,
        "X1": 0.0,
        "X2": 0.0,
        "X3": 0.0,
        "Y1": 0.0,
        "Y2": 0.0,
        "Y3": 0.0,
        "X4": 0.0,
        "Y4": 0.0
        
    }

    def scale_resolution(variables):
        # This function shows the user the current scale and resolution assuming the base units are mm
        scale = variables["scale"]
        scatter_resolution = variables["scatter_resolution"]
        if scale <= 1000:
            # Current multiplication is in micron
            print("unit scale is ",1000 / scale," um"," and delta time is ",scatter_resolution," s",)
        else:
            # Current multiplication is in nano-meters
            print("unit scale is ",scale," nm"," and delta time is ",scatter_resolution," s",)

    def error_message(status):
        # This is to display the error messages that are produced by the functions with different error codes
        match status:
            case -1:
                print("ERROR: Configuration of M98 command not supported")
            case -2:
                print("ERROR: No sub-program call and loop number found on M98 command")
            case -3:
                print("ERROR: Incorrect number of sub-programs to M99 returns")
            case -4:
                print("ERROR: Infinite loop")
            case -5:
                print("G2 I or J command error radius not equal to both points")
            case -6:
                print("G3 I or J command error radius not equal to both points")
            case -7:
                print("Error")
            case -8:
                print("Error: G1 command contains no co-ordinates")
            case -9:
                print("Error: Subprogram call M98 contains a non valid program number or loop number or is missing a number")
            case -10:
                print("Error: Subprogram call o contains a non valid number or no number")
            case -11:
                print("Line contains incorrect syntax for example axis variable with no value (Y only instead of Y0)")

        sys.exit("Error: Program exit, refer to message above")

    def read_in_file(params):
        # This will read in the file
        # First the file is opened for reading 'r' from the filename defined above
        # Check that the file exists first
        if not os.path.isfile(params["Filename"]):
            # File does not exist
            sys.exit("Error: File or path does not exist")
        file = open(params["Filename"], "r")
        # The file is read in line by line into the file contents
        with open(params["Filename"]) as file:
            # Reads in each line as a string and appends to the array
            params["File_contents"] = file.readlines()
        # file_length will always be one short due to the index of list being zero
        params["File_length"] = len(params["File_contents"])
        file.close()
        return params

    def does_line_contain_P_l(line):
        # This function is called as part of the unlooping for sub-programs
        # Function to return the number of times a line contains P or l
        index = line.find("P")
        check = 0
        if index != -1:
            check += 1
            # Also run check to make sure that character before is a blank space
            if line[index - 1 : index] != " ":
                return 0
        index = 0
        index = line.find("L")
        if index != -1:
            check += 1
            # Also run check to make sure that character before is blank space
            if line[index - 1 : index] != " ":
                return 0
        return check

    def remove_comments(params):
        # Before removing comments allow settings to be taken from them if they have the correct Syntax
        # Custom comment after the % to show that these are what is bring found
        # This will also remove anyline that does contain a letter or number
        line_number = 0
        # Loop through the code and extract each line
        for line in params["File_contents"]:
            if line.find("%") != -1 or line.find(";") != -1:
                if line.find("%") != -1:
                    # Remove '%' from anywhere within the code
                    newline = line.split("%", 1)
                    # Check the second half of the line for the special symbol used to denote parameters
                    # Save these parameters into a serpate array
                    if line.find("@") != -1:
                        parameter_line = line.split("@", -1)  # Split the line after @
                        parameter = parameter_line[1].rstrip("\n")  # Strip the newline command
                        params["Parameters_line_array"].append(parameter)  # Append to parameter array
                        # params["Parameters_line_array"].append(line_number)
                if line.find(";") != -1:
                    # Remove ';' from anywhere within the code
                    newline = line.split(";", 1)
                if newline[0] != "":
                    # If line is blank remove it
                    params["File_contents_edited"].append(newline[0])
            elif line.find("#") != -1:
                # If line contains # skip it
                pass
            else:
                newline = line
                if newline.isalnum() == "True":
                    params["File_contents_edited"].append(newline)
                elif newline != "":
                    # If line is not blank add it
                    params["File_contents_edited"].append(newline)
            line_number = line_number + 1
        # print(return_contents)
        return params

    def remove_newline(params):
        # Remove newline removes any line that contains only a newline character
        temp = np.copy(params["File_contents_edited"])
        params["File_contents_edited"] = []
        for line in temp:
            # Remove '\n' from end of each line
            newline = line.replace("\n", "")
            if newline != "":
                params["File_contents_edited"].append(newline)
        return params

    def remove_tabs(params):
        # This will remove the tabs found in lines
        temp = copy.deepcopy(params["File_contents_edited"])
        params["File_contents_edited"] = []
        for line in temp:
            # Remove '\n' from end of each line
            newline = line.replace("\t", "")
            if newline != "":
                params["File_contents_edited"].append(newline)
        return params

    def all_uppercase(params):
        # Make everything upper case to avoid issues later on and standardise the output of the code
        temp = copy.deepcopy(params["File_contents_edited"])
        params["File_contents_edited"] = []
        for line in temp:
            params["File_contents_edited"].append(line.upper())
        return params

    def parameters_extraction(params, variables):
        # This will take in a parameter array and loop through extracting the parameters and ordering them then returning the array
        params["Parameters"] = [0] * len(variables["Variable_names"])
        # Loop through the lines within contents and match each variable with the correct name listed above
        for line in params["Parameters_line_array"]:
            # First section the line at the ':'
            newline = line.split(":", -1)
            # print(line)
            # Remove all spaces within the newline[0]
            variable_temp = newline[0].replace(" ", "")
            # Check to see if newline is within the array of variable_names
            if variable_temp in variables["Variable_names"]:
                # Found the name now take the value and place it inside the correct cell within the return_array
                index = variables["Variable_names"].index(variable_temp)
                # Remove any part of the string that is not a number or decimal point
                params["Parameters"][index] = float(re.sub("[^\d\.]", "", newline[1]))

                # Assign the read in variables to their correct parameters 
                match variable_temp:
                    # Converts everything to µm
                    case "SyringeTemperature":
                        variables["Syringe_Temperature"] = float(re.sub("[^\d\.]", "", newline[1]))
                    case "NeedleTemperature":
                        variables["Needle_Temperature"] = float(re.sub("[^\d\.]", "", newline[1]))
                    case "BuildPlateTemperature":
                        variables["Build_Plate_Temperature"] = float(re.sub("[^\d\.]", "", newline[1]))
                    case "AppliedVoltage":
                        variables["Applied_Voltage"] = float(re.sub("[^\d\.]", "", newline[1]))
                    case "AppliedPressure":
                        variables["Applied_Pressure"] = float(re.sub("[^\d\.]", "", newline[1]))
                    case "FibreDiameter":
                        variables["Fibre_Diameter"] = float(re.sub("[^\d\.]", "", newline[1]))
                    case "MaterialDensity":
                        variables["Material_Density"] = float(re.sub("[^\d\.]", "", newline[1]))
                    case "CriticalTranslationSpeed":
                        variables["global_return_CTS"] = float(re.sub("[^\d\.]", "", newline[1]))
                    case "Speed_Ratio":
                        variables["Speed_Ratio"] = float(re.sub("[^\d\.]", "", newline[1]))
        return params, variables

    def check_outputs(params):
        # This function is designed to check that all output folders have been created.
        # Currently 10/03/2023 - Ouput\txt_file_name\Gcode_processing_output
        # Creates a text file or erases the current text file so that it does not append
        # This file exports the unlooped code for each run of the program for safe keeping
        if os.path.exists(os.path.join(os.getcwd(), "Output")):
            print("'Output' folder found")
        else:
            # Create the required folder
            print("Creating " + "'Output'" + " folder")
            os.mkdir(os.path.join(os.getcwd(), "Output"))
        # Next create the output folder for the specified txt file
        if os.path.exists(os.path.join(os.getcwd(), ("Output/" + params["Filename_only"]))):
            print("'" + params["Filename_only"] + "' folder found")
        else:
            # Create the required folder
            print("Creating '" + params["Filename_only"] + "' folder")
            os.mkdir(os.path.join(os.getcwd(), ("Output/" + params["Filename_only"])))
        # Create the unlooped code text file
        params["Text_File"] = open(("Output/" + params["Filename_only"] + "/" + params["Filename_only"] + "_Unlooped_Code.txt"), "w+")
        # Create the pixel coordinates file
        params["Pixel_File"] = open("Output/" + params["Filename_only"] + "/" + params["Filename_only"] + "_pixel_cords.csv", "w+")
        # Create the edited output file
        params["Edit_Output"] = open("Output/" + params["Filename_only"] + "/" + params["Filename_only"] + "_editied_output.txt", "w+")
        # Console log file
        params["Console Log"] = "Output/" + params["Filename_only"] + "/" + params["Filename_only"] + "_console_log.txt"
        # sys.stdout = open(params["Console Log"], 'w')

        # Create the output folder for all the images when running new make video
        if os.path.exists(os.path.join(os.getcwd(), ("Output/" + params["Filename_only"] + "/Images"))):
            print("'Images' folder found")
        else:
            # Create the required folder
            print("Creating 'Images' folder found")
            os.mkdir(os.path.join(os.getcwd(), ("Output/" + params["Filename_only"] + "/Images")))
        return params

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
            f.write("; " + "Material_Used: " + str(variables["Material_Used"]) + " mg" + "\n" + contents)
        f.close()

        variable_names = ["Estimated_Time", "Material_Used"]
        # Do the same for the input file
        with open(params["Filename"], mode="r+") as f:
            contents = f.readlines()
        
        for i in range(len(variable_names)):
            line_number = 0
            found  = False
            for line in contents:
                if variable_names[i] in line:
                    # Edit the current line 
                    found  = True
                    break
                line_number = line_number + 1
            if found  == True:
                match i:
                    case 0:
                        contents[line_number] = ("%@ " + "Estimated_Time: " + str(variables["Estimated_Time"]) + "\n")
                    case 1:
                        contents[line_number] = ("%@ " + "Material_Used: " + str(variables["Material_Used"]) + " mg" + "\n")
            elif found == False:
                match i:
                    case 0:
                        contents.insert(1, "%@ " + "Estimated_Time: " + str(variables["Estimated_Time"]) + "\n")
                    case 1:
                        contents.insert(1, "%@ " + "Material_Used: " + str(variables["Material_Used"]) + " mg" + "\n")
            
        
        with open(params["Filename"], "w") as f:
            contents = "".join(contents)
            f.write(contents)

    # Functions for unlooping code
    def unloop_lines(params):
        # This function goes through the gcode and finds the line numbers of all the sub-call functions
        temp = copy.deepcopy(params["File_contents_edited"])
        params["File_contents_edited"] = []
        for line in temp:
            # if line.find("G") != -1 and line.find("F") != -1:
            #     # If line contains G and F
            #     newline = line.split("F", 1)
            #     return_contents.append("F" + newline[1])
            #     return_contents.append(newline[0])
            # elif line.find("M") != -1 and line.find("F") != -1:
            #     # If line contains M and F
            #     newline = line.split("F", 1)
            #     return_contents.append("F" + newline[1])
            #     return_contents.append(newline[0])
            # For multiple G commands on the same line scan through the line and then break at each of the G commands
            # Whilst doing this I will also remove any +signs after the G commands
            if line.find("G") != -1:
                # If line contains a G command Split the line at every command
                newline = line.split("G", -1)
                # See if there are multiple splits
                if len(newline) > 2:
                    for i in range(1, len(newline)):
                        params["File_contents_edited"].append("G" + newline[i])
                else:
                    # If no doubles found append the line to the file contents
                    newline = line
                    if newline != "":
                        # If line is blank remove it
                        params["File_contents_edited"].append(newline)
            else:
                newline = line
                if newline != "":
                    # If line is blank remove it
                    params["File_contents_edited"].append(newline)
        return params

    def scan_for_subprogram(params):
        # zero index and line count
        count = 0
        index = 0
        array_num = 0
        # ensure that the arrays are accessible globally
        # This function will scan through the file for all the M98, 99 and o commands
        for line in params["File_contents_edited"]:
            # determine the index for M98 if line does not contain M98 find returns -1
            index = line.find("M98", 0, 3)
            if index != -1:
                # make mini array to contain contents to be added to global array
                append_contents = []
                append_contents.append(count)  # - 1)
                # Check the rest of the line for the P and l commands
                check = does_line_contain_P_l(line)
                if check == 1:
                    # This configuration for M98 is not supported
                    error_message(-1)
                elif check == 2:
                    # Line contains both P and L meaning that it contains a sub-program number as well as the number of loops
                    # Check that the program number is a number
                    program_number = line[line.find("P") + 1 : line.find("L")]
                    isInt = True
                    try:
                        # converting to integer
                        int(program_number)
                    except ValueError:
                        isInt = False
                    if isInt:
                        # Program is valid and append to array
                        append_contents.append(int(float(program_number)))
                    else:
                        # Error the program number is not a valid number
                        error_message(-9)
                    # Test the loop number to check if it is a number
                    loop_number = line[line.find("L") + 1 : len(line)]
                    isInt = True
                    try:
                        # converting to integer
                        int(loop_number)
                    except ValueError:
                        isInt = False
                    if isInt:
                        # Program is valid and append to array
                        append_contents.append(int(float(loop_number)))
                    else:
                        # Error the program number is not a valid number
                        error_message(-9)
                    append_contents.append(0)
                else:
                    error_message(-2)
                # Append to the M98 array the line number, function call number and the number of loops
                params["M98_Array"].append(append_contents)
            # determine the index for o if line does not contain o find returns -1
            index = line.find("O", 0, 1)
            if index != -1:
                # make mini array to contain contents to be added to global array
                append_contents = []
                append_contents.append(count)  # - 1)
                sub_routine_number = line[line.find("O") + 1 : len(line)]
                # sub_routine_number = int(float(line[line.find("O")+1:len(line)]))
                isInt = True
                try:
                    # converting to integer
                    int(sub_routine_number)
                except ValueError:
                    isInt = False
                if isInt:
                    # Program is valid and append to array
                    append_contents.append(int(float(sub_routine_number)))
                else:
                    # Error the program number is not a valid number
                    error_message(-10)
                array_num = int(float(sub_routine_number))
                # Third cell in array holds where we came from
                append_contents.append(0)
                # Append line number and function number to array
                params["O_Array"].append(append_contents)
            # determine the index for M99 if line does not contain M99 find returns -1
            index = line.find("M99", 0, 3)
            if index != -1:
                # Append the line number to the M99 array and the corresponding subprogram
                params["M99_Array"].append([count, array_num])
            # Increment the line number counter
            count += 1
        # Error check to be performed to make sure that all the functions have matching names and M99 commands
        # Check if M99 and o subprograms have the same length as each subprogram needs a m99 command at the end
        if len(params["M99_Array"]) != len(params["O_Array"]):
            print("M99_array_count: ",len(params["M99_Array"]),"O_array_count: ",len(params["O_Array"]))
            print("M99_array [line num, sub program linked num]: ",params["M99_Array"],"O_array [line num, sub program number, number of loops (to be used)]",params["O_Array"])
            error_message(-3)
        # Commented as of 21/11/2021 as this may lead to a user not using sub-programs but being flagged as an error
        # count = 0
        # for x in range(len(o_array)):
        #     # Check each function with the function call to confirm
        #     for i in range(len(M98_array)):
        #         if M98_array[i-1][1] == o_array[x-1][1]:
        #             count += 1
        # if count < len(o_array):
        #     # This is checking if there are not enough call commands for the number of subprograms listed
        #     # Doing this allows for multiple different calls to the same sub-program
        #     return -2
        # Check the subprograms to make sure that no infinite loops occur
        for x in range(len(params["M98_Array"])):
            for i in range(len(params["O_Array"])):
                if (params["M98_Array"][x - 1][0] > params["O_Array"][i - 1][0] and params["M98_Array"][x - 1][0] < params["M99_Array"][i - 1][0]):
                    # Testing to see if there is a M98 command within a subprogram, if there is does the M98 command address the current subprogram
                    if params["M98_Array"][x - 1][1] == params["O_Array"][i - 1][1]:
                        error_message(-4)
        return params

    def line_by_line(params):
        # This function will unloop the code and make it into a program that can be read by any gcode reader
        M98_variable = copy.deepcopy(params["M98_Array"])
        test = copy.deepcopy(params["M98_Array"])
        # This array can be changed as we go through the program to account for nested loops
        pointer = 0
        # What is the last g or m command
        last_command = ["", ""]
        # Doing this as we are going to be pointing all across the file
        current_line = []
        current_line = params["File_contents_edited"][pointer]
        M98_first = [i[0] for i in M98_variable]
        O_second = [i[1] for i in params["O_Array"]]
        while True:
            # If current line does contain M2 stop
            if current_line.find("M2") != -1:
                # End of program
                break

            if pointer == (len(params["File_contents_edited"])):
                break

            # for i in range(len(M98_variable)):  # As the range is from 1:x\
            
            if pointer in M98_first:
                # Found a loop shift the pointer to the corresponding o command
                i =  M98_first.index(pointer)
                if M98_variable[i][2] > 0:
                    if M98_variable[i][1] in O_second:
                        m =  O_second.index(M98_variable[i][1])
                        if (M98_variable[i][1] == params["O_Array"][m][1]):  # Scans through the command array and finds the corresponding command to the requested loop
                            params["O_Array"][m][2] = pointer  # Which command called the loop
                            M98_variable[i][3] = pointer  # Which command called the loop to make a match
                            pointer = params["O_Array"][m][0]  # + 1#Set the pointer to the array index of the loop + 1 as I want to skip the loop command 'o'
                            # subprogram = params["O_Array"][m-1][1]
                            # break

            # for i in range(len(M98_variable)):  # As the range is from 1:x
            #     if pointer == M98_variable[i][0]:
            #         # Found a loop shift the pointer to the corresponding o command
            #         for m in range(len(params["O_Array"])):  # As the range is from 1:x
            #             if (M98_variable[i][1] == params["O_Array"][m][1]):  # Scans through the command array and finds the corresponding command to the requested loop
            #                 params["O_Array"][m][2] = pointer  # Which command called the loop
            #                 M98_variable[i][3] = pointer  # Which command called the loop to make a match
            #                 pointer = params["O_Array"][m][0]  # + 1#Set the pointer to the array index of the loop + 1 as I want to skip the loop command 'o'
            #                 # subprogram = params["O_Array"][m-1][1]
            #                 break

            if current_line.find("M99") != -1:
                # Need to go to start of loop again and decrement params["M98_Array"]_variable
                # Only need to find the closest o command above it
                for n in range(len(params["M99_Array"])):
                    # DETERMINE which M99 we have reached
                    if params["M99_Array"][n][0] == pointer - 1:  # This is correct
                        # can determine the current function
                        for m in range(len(params["O_Array"])):
                            if params["M99_Array"][n][1] == params["O_Array"][m][1]:
                                for i in range(len(M98_variable)):
                                    # Linked line number and subprogram to be able to return
                                    if params["O_Array"][m][2] == M98_variable[i][3]:
                                        # Determined which array we are refferring too in terms of the loops
                                        M98_variable[i][2] = (M98_variable[i][2] - 1)  # Decrement the number of loops remaining
                                        pointer = params["O_Array"][m][0]  # + 1 #Set the pointer to the array index of the loop + 1 as I want to skip the loop command 'o'
                                        if (int(M98_variable[i][2]) <= 0):  # If the number of loops remaining is zero or less then the loop has been completed
                                            pointer = params["O_Array"][m][2]
                                            params["O_Array"][m][2] = 0
                                            M98_variable[i][3] = 0
                                            # print("check")
                                            temp_2 = test[i][2]
                                            M98_variable[i][2] = temp_2
                                            break
                                        break
            # pointer += 1
            current_line = params["File_contents_edited"][pointer]
            # # Testing
            # # print(params["M98_Array"]_variable)
            # # print( params["O_Array"])

            # Break out the following into another function
            # Add the ability to ignore blank lines like the one at the end of the code
            # This goes through the code and appends it to the unlooped file and ignores the loop commands
            if (current_line.find("M99", 0, 3) == -1 and current_line.find("M98", 0, 3) == -1 and current_line.find("O", 0, 1) == -1):
                # If current command is not one of the listed commands i.e. does not contain M or G or D
                if (current_line.find("G", 0, 1) != -1 or current_line.find("M", 0, 1) != -1 or current_line.find("D", 0, 1) != -1 or current_line.find("F", 0, 1) != -1 or current_line.find("O", 0, 1) != -1):
                    # Add new line to output file or terminal
                    params["Unlooped_contents"].append(current_line)
                    # print(current_line)
                    params["Text_File"].write(current_line + "\n")
                    # Split the line to get the command
                    last_command = current_line.split(" ", 1)  # The command id will appear in last_command[0]
                else:
                    # Do not add the line if it does not contain numbers or letter i.e. line only contains spaces
                    result = current_line.isspace()
                    if (result == 0):
                        if current_line.find(" ", 0, 1) != -1:
                            # There is a space before the co-ordinates so add the command
                            # print(last_command[0] + current_line)
                            params["Text_File"].write(last_command[0] + current_line + "\n")
                            params["Unlooped_contents"].append(last_command[0] + current_line)
                        else:
                            # As line does not contain space at the start add a space after the command
                            # print(last_command[0] + " " + current_line)
                            params["Text_File"].write(last_command[0] + " " + current_line + "\n")
                            params["Unlooped_contents"].append(last_command[0] + " " + current_line)
            pointer += 1
        return params

    # Functions for plotting gcode:
    # https://stackoverflow.com/questions/48145096/draw-an-arc-by-using-end-points-and-bulge-distance-in-opencv-or-pil
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
        # Determine the angle of the line
        params["Angle"] = math.atan2(params["Y2"] - variables["Current_Y"], params["X2"] - variables["Current_X"])
        # print(x1,x3,y1,y3)
        # Extract the values ***********************************************
        # previous_values stores the last commands distance left and feedrate
        # Next determine the distance into the current command from the segment_length and the distance_from_previous
        # First determine the segment length
        segment_length = (round(variables["scatter_resolution"] * params["Feed_rate"], 10) * 1000)  # To get segment length (speed mm/s * time s) tehn convert to microns
        # print(segment_length, " µm") # µm
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
        if variables["high_speed"] == False:
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

    def docircle(params, variables):
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
        if variables["high_speed"] == False:
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
        if [variables["Current_X"], variables["Current_Y"], params["Line"], params["Positioning"]] in params["commands_used"]:
            index = params["commands_used"].index([variables["Current_X"], variables["Current_Y"], params["Line"], params["Positioning"]])
            params["commands_used_counter"][index] += 1
            color = (5 * params["commands_used_counter"][index], 0, 0)
            if 5 * params["commands_used_counter"][index] > 255:
                color = (255, (5 * params["commands_used_counter"][index]) - 255, 0)
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

    def Plotting_G1_2D(params, variables):
        # This function plots G1 commands as well as calculating the distance reuquired for each command
        # New idea 3/03/2023 keep all units at floats in µm then at the last second before plotting convert but keep in DRO as correct units (pixel_cords)
        # Extract parameters from array
        # Function to check commands that have already been plotted
        if variables["calc_only"] == 0 and variables["Disable_colour_update"] == False:  # Ensure the function is in plotting mode
            params, variables = check_command(params, variables)
            if params["command_check"] == False:
                params["commands_used"].append([variables["Current_X"], variables["Current_Y"], params["Line"], params["Positioning"]])
                params["commands_used_counter"].append(1)
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
        # print(params["Centre_1"],params["Centre_2"])
        # If the line only contains Feed rate command (required for Marlin return)
        if "F" in params["Command_array"] and len(params["Command_array"]) == 4:
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
                params["commands_used"].append([variables["Current_X"], variables["Current_Y"], params["Line"], params["Positioning"]])
                params["commands_used_counter"].append(1)
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
            if variables["calc_only"] == 0:
                if variables["scatter_path"] == 0:
                    draw_circle(params,variables)
                else:
                    params["dir"] = 2
                    params,variables = docircle(params,variables)
            else:
                params["Distance"] = math.pi * params["Radius"] * 2
                params["dir"] = 2
                params, variables = docircle(params, variables)
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
                params["commands_used"].append([variables["Current_X"], variables["Current_Y"], params["Line"], params["Positioning"]])
                params["commands_used_counter"].append(1)
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
            if variables["calc_only"] == 0:
                if variables["scatter_path"] == 0:
                    draw_circle(params,variables)
                else:
                    params["dir"] = 3
                    params,variables = docircle(params,variables)       
            else:
                params["Distance"] = math.pi * params["Radius"] * 2
                params["dir"] = 3
                params, variables = docircle(params, variables)
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
        variables["Current_X"] = temp3_x
        variables["Current_Y"] = temp3_y
        return params, variables

    # Functions for outputting img and scaffold outputs
    def draw_grid(params, grid_spacing=1000, color=(220, 220, 220), thickness=1):
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

    def segment_line(params, variables):
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

        # print(params["X_increase"],params["Y_increase"],params["I_increase"],params["J_increase"])
        return params, variables

    def line_reader(params, variables):
        # This function is designed to be called by any path planning or motion calculation function to standardise the output
        
        # Add the other parameters for the functions and pass through these depending on the input function
        # By knowing the current feedrate and being given the acceleration the program should be able to determine the total time and path length
        # Change this to use the style for params["Line"] reading from the G1 function to streamline the process and allow for more commands in the future
        
        params["X_increase"] = float('NaN')
        params["Y_increase"] = float('NaN')
        params["Z_increase"] = float('NaN')
        params["I_increase"] = 0
        params["J_increase"] = 0
        params["Radius"] = 0
        params["Command_flag"] = ""
        params["Command_number"] = 0
        params["Edited_line"] = ""
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
            # This command will change the values for the origin_x and origin_y
                variables["Origin_X_G92"] =  variables["Origin_X"]
                variables["Origin_Y_G92"] =  variables["Origin_Y"]
                # This will only track the last G92 command not the first which is used for later plotting
                # Addition of new variable for original origin (the first one)
                variables["Origin_X"] = variables["Current_X"] - params["X_increase"]
                variables["Origin_Y"] = variables["Current_Y"] + params["Y_increase"]
                
            if params["Command_number"] == 92.1:
            # Reset the G92 command to original machine global coordinates
                variables["Origin_X"] = variables["Origin_X_G92"]
                variables["Origin_Y"] = variables["Origin_Y_G92"]
                
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
                        params["Pixel_coords_um"][temp_length_pixel_coords][2] = temp_One_coordinate_system
                        # Used in lag vector to set the length of the computation without having to calculate it
                        params["Pixel_coords_um"][temp_length_pixel_coords][3] = (len(params["Pixel_coords_um"]) - temp_length_pixel_coords)
                    params["Distance_array"].append((params["Distance"] / variables["scale"]))  # mm * scale
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
                        params["Pixel_coords_um"][temp_length_pixel_coords][2] = temp_One_coordinate_system
                        # Used in lag vector to set the length of the computation without having to calculate it
                        params["Pixel_coords_um"][temp_length_pixel_coords][3] = (len(params["Pixel_coords_um"]) - temp_length_pixel_coords)
                    params["Distance_array"].append((params["Distance"] / variables["scale"]))  # mm * scale
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
                        params["Pixel_coords_um"][temp_length_pixel_coords][2] = temp_One_coordinate_system
                        # Used in lag vector to set the length of the computation without having to calculate it
                        params["Pixel_coords_um"][temp_length_pixel_coords][3] = (len(params["Pixel_coords_um"]) - temp_length_pixel_coords)
                    params["Distance_array"].append((params["Distance"] / variables["scale"]))  # mm * scale
                    # Compute the current time to complete the command using the feedrate
                    if params["Feed_rate"] > 0:
                        params["Time_array"].append((params["Distance"] / variables["scale"]) / params["Feed_rate"])  # mm / (mm/s)
            # print(params["Line"],params["X1"],params["Y1"],params["X2"],params["Y2"],params["Diff"])
        params["Feed_rate_previous"] = params["Feed_rate"]
        return params, variables

    def progressBar(iterable, prefix="", suffix="", decimals=1, length=100, fill="█", printEnd="\r"):
        """
        Call in a loop to create terminal progress bar
        @params:
            iterable    - Required  : iterable object (Iterable)
            prefix      - Optional  : prefix string (Str)
            suffix      - Optional  : suffix string (Str)
            decimals    - Optional  : positive number of decimals in percent complete (Int)
            length      - Optional  : character length of bar (Int)
            fill        - Optional  : bar fill character (Str)
            printEnd    - Optional  : end character (e.g. "\r", "\r\n") (Str)
        """
        total = len(iterable)

        # Progress Bar Printing Function
        def printProgressBar(iteration):
            percent = ("{0:." + str(decimals) + "f}").format(100 * (iteration / float(total)))
            filledLength = int(length * iteration // total)
            bar = fill * filledLength + "-" * (length - filledLength)
            print(f"\r{prefix} |{bar}| {percent}% {suffix}", end=printEnd)

        # Initial Call
        printProgressBar(0)
        # Update Progress Bar
        for i, item in enumerate(iterable):
            yield item
            printProgressBar(i + 1)
        # Print New Line on Complete
        print()

    def Plot_code(params, variables):

        # Build plate size:
        buildplate = [variables["Y_build"] * 100,variables["X_build"] * 100,3,]  # 10um, 10um, RGB i.e. 5000 x 5000 is 50mm x 50mm
        # Plot the external of the build plate
        # Create the image that will show the gcode, the size of the image will be the bed size of the printer at 1um
        if len(params["Image"]) == 0:
            params["Image"] = np.zeros(buildplate, dtype="uint8")  # Maximum of 150000, 150000, 3
            params["Image"][:] = variables["Background_colour"]  # Make the image have a white background
            # Draw in the background grid
            params = draw_grid(params)
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
        params["Time_array"] = []
        # Feed rate will be used to calculate the total time of the code
        params["Feed_rate"] = 1
        # Which positioning system is being used?
        params["Positioning"] = []
        # Build plate size:
        # buildplate = [100 * 100,100 * 100,3,]  # 10um, 10um, RGB i.e. 5000 x 5000 is 50mm x 50mm
        # Create the image that will show the gcode, the size of the image will be the bed size of the printer at 1um
        # img = np.zeros(buildplate, dtype="uint8")  # Maximum of 150000, 150000, 3
        # img[:] = variables["Background_colour"]  # Make the image have a black background
        # Determine the parameters used within the parameter table
        params, variables = parameters_extraction(params, variables)
        params["G2_G3_Edited_output"] = []
        # line_num = 0
        variables["calc_only"] = 1
        for line in params["Unlooped_contents"]:
            # Loop through all the lines in the edited contents array
            params["Line"] = line
            # print(line)
            # print(params["Unlooped_contents"][0])
            params, variables = line_reader(params, variables)
            # params["Unlooped_contents"][line_num] = params["Line"]
            if params["Edited_line"] != "":
                params["G2_G3_Edited_output"].append(params["Edited_line"])
            else:
                params["G2_G3_Edited_output"].append(line)
            # line_num = line_num + 1
            
        # New feature for adapting any bad G2 and G3 commands is to alter the lines and then save the files
        if variables["edited_flag"]:
            # If the system has edited a line - save all the contents to a new output file
            for line in params["G2_G3_Edited_output"]:
                params["Edit_Output"].write("%s\n" % str(line))
            # f.write(f"{pixel_cords[i]}\n")
            params["Edit_Output"].close()
        # Determine the direction of the print and then the size to ensure that it remains in frame
        
        # Changed to pixel_cords as of 24/10/2022
        # if variables["high_speed"] == False : 
        #     x_pixel_cords = [item[0] for item in params["Pixel_coords"]]  # Pixel_cords save the reolution for images i.e. at 10um
        #     y_pixel_cords = [item[1] for item in params["Pixel_coords"]]
        #     min_x = min(x_pixel_cords) / 100
        #     max_x = max(x_pixel_cords) / 100

        #     min_y = min(y_pixel_cords) / 100
        #     max_y = max(y_pixel_cords) / 100
        # else:
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
        # print("Correct: ",variables["Origin_X"],variables["Origin_Y"])
        # Entire Print calculations
        # Determine the linear distance travelled and use it to compute the approximate time
        print("Distance travelled:", round(sum(params["Distance_array"]) / 1000, 3), "m")
        # Time array is in seconds
        seconds = round(sum(params["Time_array"]))
        day = seconds // (24 * 3600)
        seconds = seconds % (24 * 3600)
        hour = seconds // 3600
        seconds %= 3600
        minutes = seconds // 60
        seconds %= 60
        # Display the time to console
        print("Total Time:", day, "day", hour, "hr", minutes, "min", seconds, "s")

        # Volume in cm^3
        volume = (((params["Parameters"][5] / 2) ** 2 * math.pi) * sum(params["Distance_array"])) * 0.001
        material_mass = volume * params["Parameters"][6]
        correction_factor = 1.25
        variables["Material_Used"] = material_mass * correction_factor
        print("Material Used: ",round(round(variables["Material_Used"], 4) * 1000, 10),"mg",)
        # global global_return_CTS
        # global_return_CTS = params["Parameters"][7]
        # Get the out;uts ready to save to the ext file
        variables["Estimated_Time"] = (str(day)+ " day "+ str(hour)+ " hr "+ str(minutes)+ " min "+ str(seconds)+ " s")
        # materialout = (str(round(round((material_mass * correction_factor), 4) * 1000, 1)) + " mg")

        variables["X_build"] = round((abs(min_x) + abs(max_x)) + 2)  # mm
        if min_y < 0 and max_y < 0:
            # This is to solve issues with having a build plate that is longer than the print due to Two negative values
            variables["Y_build"] = math.ceil(abs(min_y)) + 2  # mm
        else:
            variables["Y_build"] = math.ceil(abs(min_y) + abs(max_y)) + 2  # mm
        print("Total size used x:", (variables["X_build"] - 2), "y:", (variables["Y_build"] - 2))
        # Save the pixel coordinates to a file
        if variables["high_speed"] == False : 
            for i in range(len(params["Pixel_coords_um"])):
                params["Pixel_File"].write("%s\n" % str(params["Pixel_coords_um"][i])[1:-1])
                # f.write(f"{pixel_cords[i]}\n")
            # params["Pixel_File"].close()

        return params, variables

    # Functions for editing gcode and sending gcode:
    # *******************************************************************************************************************************************
    # Main code:
    print("************** Setup **************")
    scale_resolution(variables)
    # Read in the text file
    params = read_in_file       (params)
    # Remove non-required elements from the file and make uppercase
    params = remove_comments    (params)
    params = remove_newline     (params)
    params = remove_tabs        (params)
    params = all_uppercase      (params)
    # Unloop multiple commands per line such as G1 X20 F200
    params = unloop_lines       (params)
    # Insert blank to start of contents to permit correct reading of the first line after comments have been removed
    params["File_contents_edited"].insert(len(params["File_contents_edited"]), "")
    # Determine where the sub-programs are and error out if conditions are not met
    params = scan_for_subprogram(params)
    # Print the time taken to this point
    print("It took", round(time.time() - variables["Previous_time"], 2), "seconds to scan for subprogram.")
    previous = time.time()
    # Check that all output files are in place
    params = check_outputs      (params)
    # Unloop the code to allow for it to be read line by line and for the unlooped code to be saved to a txt file
    params = line_by_line       (params)
    file_contents = []
    # Close the unlooped text file as it is no longer required
    params["Text_File"].close()
    # Any changes made to unlooped_code after this point will not be written to file
    if variables["unloop_only"] == 0:
        # Print the time taken to this point
        print("It took",round(time.time() - variables["Previous_time"], 2),"seconds to un-loop the code.",)
        previous = time.time()
        print("************** Scaffold outputs **************")
        # To determine time, print view start location and amount of polymer used enable function below

        params, variables = motion_calculations(params, variables)

        # To allow for analysis over the structure make every command into a G1 of a ceratin resolution and then save that file.
        # This can also be adapted to compute the digital model of a scaffold be looping along these points and changing them when another fibre is encountered
        print("It took",round(time.time() - previous, 2),"seconds to extract scaffold outputs.",)
        print("************** Plotting **************")
        # Plot everything as G1
        # inital_parameters.append(file_name_only)
        # # Everything_is_G1(3, inital_parameters)

        # # Initalise the plot
        # # Second variable make equal to 1 if you want to display image as well as save it
        # # Second variable make equal to 2 if you want to save animation as well as save the image
        # # Second variable make equal to 3 if you want to save image only
        
        if variables["Generate_output_image"] == True:
            Plot_code(params, variables)
        save_outputs(params,variables)
        
        # Print the time taken to this point
        previous = time.time()
        # Print the time taken to complete the entire code
        print("It took",round(time.time() - variables["Start_time"], 5),"seconds to complete the program.",)
        # print(commands_used_counter)
        # print(commands_used)
 
        # Close the console log file
        sys.stdout.close()