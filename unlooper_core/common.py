"""Small helpers shared by every stage: console messages, errors, progress, durations."""
import sys


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


def format_duration(total_seconds):
    seconds = round(total_seconds)
    day = seconds // (24 * 3600)
    seconds = seconds % (24 * 3600)
    hour = seconds // 3600
    seconds %= 3600
    minutes = seconds // 60
    seconds %= 60
    return str(day) + " day " + str(hour) + " hr " + str(minutes) + " min " + str(seconds) + " s"


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
