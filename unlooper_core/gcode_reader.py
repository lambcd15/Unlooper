"""Stage 1 - reading the G-code: load the file, strip comments / whitespace, pull the
print parameters out of the comments, split multi-command lines, find the
sub-programs (O / M98 / M99) and unloop them into one flat list of commands
(the _Unlooped_Code file)."""
import copy
import os
import re
import sys

import numpy as np

from .common import error_message

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


def is_macro_line(line):
    # Square-bracket content is treated as a macro marker (for example
    # "[pressure 20]" or "G1 X10 Y10 [layer 2]"). These are kept in the saved
    # output, but they are stripped from the motion parser so they never get
    # mistaken for X/Y/I/J/F fields.
    stripped = (line or "").strip()
    if not stripped:
        return False
    if stripped.startswith("["):
        return True
    if "[" in stripped and "]" in stripped:
        return True
    return False


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
        if is_macro_line(line):
            # Preserve square-bracket macro lines in the processed output, but do not
            # let them participate in motion parsing later on.
            newline = line.strip()
            if newline != "":
                params["File_contents_edited"].append(newline)
            line_number = line_number + 1
            continue
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
        elif line.strip().upper().startswith("M117"):
            # M117 status/display messages (e.g. "M117 [1][H][1/250][X+][S]") carry no
            # motion data and their bracket text breaks segment_line's tokenizer, so drop them
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
    # Initialize all parameters
    variables["Syringe_Temperature"]    = 0
    variables["Needle_Temperature"]     = 0
    variables["Build_Plate_Temperature"]= 0
    variables["Applied_Voltage"]        = 0
    variables["Applied_Pressure"]       = 0
    variables["Fibre_Diameter"]         = 0
    variables["Material_Density"]       = 0
    variables["global_return_CTS"]      = 0
    variables["Speed_Ratio"]            = 0
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
                    # converting to integer (a whole number written with a decimal point, e.g. L30.0, is fine)
                    isInt = float(program_number) == int(float(program_number))
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
                    # converting to integer (a whole number written with a decimal point, e.g. L30.0, is fine)
                    isInt = float(loop_number) == int(float(loop_number))
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
        # If current line does contain M2 stop, line must contain all of M2 and not a similar line M204
        if current_line.find("M2") == 0 and len(current_line) == 2:
            
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
        # There is an issue when the M98 command is called from within a subprogram, this will cause an infinite loop as the M98 command will call the subprogram and then the M99 command will return to the subprogram and then the M98 command will be called again. This is not supported by this program and will be flagged as an error
        # Especially if the M2 is behind a subprogram as the M2 will not be reached and the program will run infinitely
        # Basically it does not set the loop number to zero when the M99 command is reached and the program will run infinitely
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
        # A line that starts with whitespace is normally a coordinate-only continuation
        # ("  X10 Y5") that inherits the previous command, but " G2 X0 Y-1" already has its
        # own command - strip the indent so it isn't turned into "G3 G2 X0 Y-1"
        if current_line.lstrip()[:1] in ("G", "M", "D", "F", "O"):
            current_line = current_line.lstrip()
        # # Testing
        # # print(params["M98_Array"]_variable)
        # # print( params["O_Array"])

        # Break out the following into another function
        # Add the ability to ignore blank lines like the one at the end of the code
        # This goes through the code and appends it to the unlooped file and ignores the loop commands
        if is_macro_line(current_line):
            # Preserve bracketed macro commands in the final unlooped file exactly as written,
            # but leave them out of the motion model and parser.
            params["Unlooped_contents"].append(current_line)
            params["Text_File"].write(current_line + "\n")
            pointer += 1
            continue
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
