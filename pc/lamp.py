# PC-side program for logging recorded angles
# Author: Tyler
# Data: May 7, 2019

import os
import sys
import numpy as np
from util import *
from rx import record
from tx import *
from analyze import analyze

def main():
    args = parse_args()
    port = args['port']
    baud = args['baud']
    record_mode = args['record']
    analyze_fname = args['analyze']
    playback_fname = args['playback']
    baseline_fname = args['set_baseline']
    angles = args['set_angles']
    sine_params = args['sine']
    plot_slice = args['plot_slice']
    use_servos = args['use_servos']
    use_imus = args['use_imus']
    verbose = args['verbose']
    
    # Validate args!
    if (playback_fname and analyze_fname) or \
       (playback_fname and baseline_fname) or \
       (analyze_fname and baseline_fname):
        logString("2 or more of: playback, analyze, set_baseline were selected."
                  " Please only choose one of these")
        quit()
    
    if plot_slice:
        if not ',' in plot_slice:
            logString("Invalid arguments for --plot_slice. "
                "Example of valid usage: --plot_slice=10,20")
            quit()
        t_start, t_end = plot_slice.split(',')
        t_start = float(t_start)
        t_end = float(t_end)
        if t_start < 0:
            logString("--plot_slice start time must be >= 0!")
            quit()
        if t_end < 0:
            logString("--plot_slice end time must be >= 0!")
            quit()
        if t_start > t_end:
            logString("--plot_slice start time must be >= end time!")
            quit()
    
    # Call requested function
    if analyze_fname:
        logString("Starting analysis")
        analyze( \
            analyze_fname, \
            args['imu'], \
            args['estimate'], \
            args['use_calibration'], \
            args['use_legacy_sign_convention'], \
            args['use_time_stamps'], \
            plot_slice \
        )
    elif playback_fname:
        logString("Starting playback")
        playback( \
            port, \
            baud, \
            playback_fname, \
            args['loop'], \
            args['use_legacy_sign_convention'], 
            args['use_time_stamps'], \
            verbose \
        )
    elif angles:
        logString("Setting servo angles")
        send_servo_angles(port, baud, angles)
    elif sine_params:
        logString("Sending sine wave")
        send_sine_wave(port, baud, sine_params, args['servo'], verbose)
    elif use_servos != None:
        change_servo_usage(port, baud, use_servos)
    elif use_imus != None:
        change_imu_usage(port, baud, use_imus)
    elif baseline_fname:
        logString("Creating baseline")
        set_baseline( \
            baseline_fname, \
            args['use_legacy_sign_convention'], \
            verbose \
        )
    elif record_mode:
        logString("Starting recording")
        record(port, baud, verbose)
    else:
        logString("No option selected")

if __name__ == "__main__":
    try:
        main()
        sys.exit(0)
    except KeyboardInterrupt as e:
        print("Interrupted: {0}".format(e))
        sys.exit(1)