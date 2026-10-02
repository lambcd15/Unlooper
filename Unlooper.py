"""Unlooper - unloops sub-programmed G-code and works out what the print will actually do.

    python Unlooper.py <file> <unloop_only 0|1> [feedrate mm/min] [density g/cm3] [fibre diameter um]
                       [render precise|preview|both|none] [acceleration mm/s2] [junction deviation mm]
                       [skip pixel coords 1|0] [jerk mm/s] [lag prediction 1|0] [CTS mm/min]
                       [write lag-format files 1|0] [lag compensation none|overshoot|pointwise|slowdown|iterative|hybrid|hybrid_constant]
                       [rapid mm/min] [overshoot scale] [slow-down ratio of CTS] [iterations]
                       [pointwise point spacing um, 0 = adaptive] [hybrid corner tolerance um]
                       [hybrid: fibre diameter limit %, 0 = none] [mandrel diameter mm, 0 = flat]
                       [fibre diameter tolerance % for the diameter view]
                       [overshoot / pointwise: time-preserving feeds 1|0]
                       [overshoot / pointwise: swing blend radius um, -1 = automatic, 0 = off]

This file holds the settings, the command line and the order the stages run in; the
stages themselves live in unlooper_core/ (see unlooper_core/__init__.py):
    gcode_reader -> toolpath -> motion_planner -> pixel_coords -> corner_path -> lag_model
                                (run together by scaffold_outputs)   -> rendering -> lag_compensation
"""
import os
import sys
import time

# git commands:
#   git pull
#   git commit -a -m "Message"
#   git push

if __name__ == "__main__":
    from unlooper_core.common import scale_resolution
    from unlooper_core.gcode_reader import (read_in_file, remove_comments, remove_newline, remove_tabs, all_uppercase,
                                            unloop_lines, scan_for_subprogram, check_outputs, line_by_line)
    from unlooper_core.scaffold_outputs import motion_calculations, save_outputs
    from unlooper_core.rendering import Plot_code, render_preview_svg

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

        # User-supplied overrides (mm/min). If 0, fall back to the file's own feed rates /
        # fibre diameter & material density for the time and material calculations.
        "Feedrate_override_mm_min": 0,
        "Fibre_Diameter_override": 0,
        "Material_Density_override": 0,
        "Flow_rate_override_mg_min": 0,

        # Lag compensation
        "compensation_image": [],
        "compensation_complete": False, # This will change after the lag has been compensated

        # Image rendering.
        #   "precise" - exact full-resolution raster (fibre width + pass overlap), slow
        #   "preview" - fast NCViewer-style vector (SVG) render: no line thickness, no
        #               overlap, but no resolution ceiling either - exact arcs, and it
        #               zooms as far as the toolpath data resolves instead of blocking up
        #               into pixels the way a raster preview would (see render_preview_svg())
        #   "both"    - write both
        #   "none"    - skip rendering, just do the timing / material calculations
        # CLI arg 5 overrides this; the GUI always sets it explicitly.
        "render_mode": "precise",
        "Generate_preview_image": False, # Derived from render_mode below

        # Machine dynamics for the acceleration-aware time / real speed estimate (see
        # plan_motion()). 0 acceleration = off, only the plain distance / feed rate time is given.
        # CLI args 8 and 9 override these; the GUI always sets them explicitly.
        "Acceleration_mm_s2": 1000,
        # Corner model: Marlin 2's junction deviation (its default 0.013 mm). The printer runs in
        # constant-velocity mode - it only slows for a corner if it can't take it at speed with
        # this much corner rounding (see plan_motion())
        "Junction_deviation_mm": 0.013,
        "Minimum_planner_speed_mm_s": 0.05,  # Marlin's floor, used for full reversals
        # Classic jerk (mm/s): largest instant speed change per axis at a corner. Applied
        # together with the junction deviation - the lower corner speed wins; 0 = off
        # (see motion_planner.junction_speed(), which also explains how the two are linked)
        "Jerk_mm_s": 5.0,
        # Pixel coords along the planned motion (speed / acceleration overlay, lag model input).
        # CLI arg 10 = 1 skips them.
        "Generate_pixel_coords": True,
        # G4 dwell: P is milliseconds (G4 P1000 = 1 s), S is seconds
        "Dwell_P_is_ms": True,
        # Jet lag prediction (lag_model.py) along the corner-rounded path. Needs acceleration > 0.
        "Lag_prediction": False,
        # Critical translation speed (mm/min) for the lag model and the "below CTS" figure.
        # 0 = use the file's CriticalTranslationSpeed parameter
        "CTS_override_mm_min": 0,
        # Lag compensation (lag_compensation.py): "none", "overshoot" (ISBF corner overshoot
        # and swing), "pointwise" (the nozzle leads the jet by the lag at every point),
        # "slowdown" (slow before corners) or "iterative" (model-driven path correction).
        # Writes <name>_Lag_compensated.txt and processes it too.
        "Lag_compensation": "none",
        "Lag_comp_rapid_mm_min": 3000,  # overshoot / pointwise: speed of the swing round a corner
        "Lag_comp_overshoot_scale": 0.85,  # overshoot / pointwise: overshoot = lag x this (ISBF used 0.85)
        "Lag_comp_slow_ratio": 1.0,  # slowdown: corner speed as a multiple of the CTS
        "Lag_comp_tolerance_mm": 0.05,  # slowdown: jet lag to reach before the corner
        "Lag_comp_iterations": 6,  # overshoot / pointwise / iterative: correction passes (0 = none)
        "Lag_comp_point_spacing_um": 0.0,  # pointwise: distance between the points compensated (0 = adaptive)
        "Lag_comp_corner_um": 20.0,  # hybrid: how far the nozzle may jump round a sharp corner (smaller = closer, slower)
        # hybrid: how much the fibre diameter may change (%), d / d0 = sqrt(v0 / v), 0 = no limit.
        # It sets how far the jet may slow: 5% -> 9.3% slower at most (Lag_comp_speed_change_pct).
        "Lag_comp_diameter_limit_pct": 0.0,
        "Lag_comp_speed_change_pct": 100.0,
        # The tolerance the fibre diameter is reported and coloured against (%)
        "Diameter_tolerance_pct": 5.0,
        # What the fibre diameter is worked out from: "nozzle" (the nozzle's speed over the
        # collector - only slowing thickens the fibre, as printed) or "jet" (the model's contact point)
        "Diameter_basis": "nozzle",
        # overshoot / pointwise options, all off by default:
        #   time-preserving feeds - each command's moves sped up (never slowed) so they take the
        #   time the command was programmed to take, and the path solved again at those feeds
        "Lag_comp_time_preserving": False,
        #   smooth swings - the kinks between each overshoot line, its swing arc and the next line
        #   blended with a fillet arc of this radius (um) so the nozzle never brakes below the feed.
        #   -1 = automatic (pointwise: v^2 / (0.85 x acceleration); overshoot: off, the original
        #   ISBF output), 0 = off
        "Lag_comp_blend_um": -1.0,
        #   adaptive spacing only: swings no smaller than the fibre diameter limit's lag
        "Lag_comp_adaptive_hold_swing": False,
        #   adaptive spacing only: the smallest turn swung round (degrees; ISBF's is 5). The lead
        #   factor can be set apart from the overshoot scale with Lag_comp_adaptive_lead.
        "Lag_comp_adaptive_swing_deg": 30.0,
        #   adaptive spacing only: gentler turns (a curve written as short lines) are steered
        #   round - the nozzle moves across onto each new line faster than the feed, never slower
        "Lag_comp_adaptive_steer": True,
        #   adaptive spacing only: "isbf" = arcs by ISBF's arc joins, "chords" = arcs cut into
        #   short lines and steered round (many more lines; no better on the files tried)
        "Lag_comp_adaptive_arcs": "isbf",
        #   the corrections to the ISBF code (see unlooper_core/isbf.py): every swing onto an arc
        #   at the rapid feed, no divide by zero at a zero-length line after an arc, and every
        #   arc written as a true arc (start and end at the same radius). False = as it was.
        "Lag_comp_isbf_fixes": True,
        # Tubular printing: mandrel diameter (mm). With it set, A (degrees) is read as the distance
        # round the tube's surface (mandrel.py), and the compensated file is written with A.
        "Mandrel_diameter_mm": 0.0,
    }
    
    # ************************************ User Variables ******************************************
    # Commands you can edit
    # Setting to 0 will unloop and generate an image
    # Setting to 1 will unloop only and not generate an image
    variables["unloop_only"] = 0
    # gcode filename enter in your own filename below
    if len(sys.argv) < 2:
        # Everything must contain forward slashes only
        file_name = "motion_printlog_M3_S12_B1_14-01-26_1624_filtered.gcode"
    else:
        # collector speed [mm/min]
        file_name = sys.argv[1]
        variables["unloop_only"] = int(sys.argv[2])
        # Optional: feedrate override (mm/min, dictates the time estimate) and
        # flow rate override (mg/min, dictates the material estimate). Omit or pass 0 to
        # fall back to the file's own feed rates / fibre diameter & material density.
        if len(sys.argv) >= 4:
            variables["Feedrate_override_mm_min"] = float(sys.argv[3])
        if len(sys.argv) >= 5:
            variables["Material_Density_override"] = float(sys.argv[4])
        if len(sys.argv) >= 6:
            variables["Fibre_Diameter_override"] = float(sys.argv[5])
        # Optional: render mode - "precise", "preview", "both" or "none"
        if len(sys.argv) >= 7:
            variables["render_mode"] = str(sys.argv[6]).strip().lower()
        # Optional: acceleration (mm/s^2) and junction deviation (mm) for the acceleration-aware estimate
        if len(sys.argv) >= 8:
            variables["Acceleration_mm_s2"] = float(sys.argv[7])
        if len(sys.argv) >= 9:
            variables["Junction_deviation_mm"] = float(sys.argv[8])
        # Optional: 1 = skip the pixel coords
        if len(sys.argv) >= 10:
            variables["Generate_pixel_coords"] = str(sys.argv[9]).strip() != "1"
        # Optional: classic jerk (mm/s, 0 = off)
        if len(sys.argv) >= 11:
            variables["Jerk_mm_s"] = float(sys.argv[10])
        # Optional: 1 = run the jet lag prediction
        if len(sys.argv) >= 12:
            variables["Lag_prediction"] = str(sys.argv[11]).strip() == "1"
        # Optional: critical translation speed override (mm/min, 0 = the file's)
        if len(sys.argv) >= 13:
            variables["CTS_override_mm_min"] = float(sys.argv[12])
        # Optional: 1 = write the lag-format pixel coords files (pixel coords, corner path, lag)
        if len(sys.argv) >= 14:
            variables["high_speed"] = str(sys.argv[13]).strip() != "1"
        # Optional: lag compensation method and its settings
        if len(sys.argv) >= 15:
            variables["Lag_compensation"] = str(sys.argv[14]).strip().lower()
        if len(sys.argv) >= 16:
            variables["Lag_comp_rapid_mm_min"] = float(sys.argv[15])
        if len(sys.argv) >= 17:
            variables["Lag_comp_overshoot_scale"] = float(sys.argv[16])
        if len(sys.argv) >= 18:
            variables["Lag_comp_slow_ratio"] = float(sys.argv[17])
        if len(sys.argv) >= 19:
            variables["Lag_comp_iterations"] = int(float(sys.argv[18]))
        if len(sys.argv) >= 20:
            variables["Lag_comp_point_spacing_um"] = float(sys.argv[19])
        if len(sys.argv) >= 21:
            variables["Lag_comp_corner_um"] = float(sys.argv[20])
        if len(sys.argv) >= 22:
            variables["Lag_comp_diameter_limit_pct"] = float(sys.argv[21])
        if len(sys.argv) >= 23:
            variables["Mandrel_diameter_mm"] = float(sys.argv[22])
        if len(sys.argv) >= 24:
            variables["Diameter_tolerance_pct"] = float(sys.argv[23])
        if len(sys.argv) >= 25:
            variables["Lag_comp_time_preserving"] = str(sys.argv[24]).strip() == "1"
        if len(sys.argv) >= 26:
            variables["Lag_comp_blend_um"] = float(sys.argv[25])

    # A fibre diameter limit sets how far the jet may slow: d / d0 = sqrt(v0 / v) <= 1 + limit
    if variables["Lag_comp_diameter_limit_pct"] > 0:
        variables["Lag_comp_speed_change_pct"] = (1.0 - 1.0 / (1.0 + variables["Lag_comp_diameter_limit_pct"] / 100.0) ** 2) * 100.0

    # Compensation works from the lag model's predictions
    if variables["Lag_compensation"] != "none":
        variables["Lag_prediction"] = True
    # Resolve the render mode into the two flags the rest of the program uses
    _render_mode = str(variables["render_mode"]).strip().lower()
    if _render_mode not in ("precise", "preview", "both", "none"):
        _render_mode = "precise"
    variables["render_mode"] = _render_mode
    variables["Generate_output_image"] = _render_mode in ("precise", "both")
    variables["Generate_preview_image"] = _render_mode in ("preview", "both")
    # The planned-motion pixel coords replace the old per-command ones from doline() / docircle()
    variables["Motion_pixel_coords"] = (variables["Acceleration_mm_s2"] > 0 and variables["Generate_pixel_coords"]
                                        and (variables["Generate_preview_image"] or variables["Generate_output_image"]
                                             or variables["high_speed"] == False))
    variables["Motion_pixel_coords_written"] = False
    # ************************************ Functions ******************************************
    # Do not touch
    # Functions for reading in gcode:
    # Extract the filename from the input, permit this to work by backwards scanning to allow for different names
    # Remove the file extension

    split_path = os.path.splitext(file_name)[0]

    file_name_only = os.path.basename(split_path)  # This works for any path variation

    params = {
        "Filename": file_name,
        "Filename_only": file_name_only,
        "File_length": "",
        "Text_File": "",
        "Pixel_File": "",
        "Edit_Output": "",
        "Image_name": "",
        "Image": [],
        "Preview_segments": [], # Numeric toolpath geometry plus source line for the fast preview
        "Dwells": [], # G4 dwells: (segments before, seconds, commands before, line)
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
        "commands_used": {}, # Maps (X, Y, Line, Positioning) -> number of times plotted, for O(1) lookup
        "command_check": False, # This is true if the command has been used before
        "Current_X_array": [], # This array is a record of all the x commands 
        "Current_Y_array": [],
        "Distance_array": [], # array for all the distances
        "Filament_array": [], # array for all the filament used
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
        "E_increase": float('NaN'),
        "S_value": float('NaN'), # This is normally used during set temperature commands
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
        if variables["Generate_preview_image"] == True:
            render_preview_svg(params, variables)
        save_outputs(params,variables)
        if variables["Mandrel_diameter_mm"] > 0:
            from unlooper_core.mandrel import render_program
            render_program(params, variables)
        if variables["Lag_compensation"] != "none":
            from unlooper_core.lag_compensation import compensate, run_compensated
            print("************** Lag compensation **************")
            compensated_path = compensate(params, variables)
            if compensated_path:
                run_compensated(compensated_path, params, variables, os.path.abspath(__file__))
        
        # Print the time taken to this point
        previous = time.time()
        # Print the time taken to complete the entire code
        print("It took",round(time.time() - variables["Start_time"], 5),"seconds to complete the program.",)
 
        # Close the console log file
        sys.stdout.close()