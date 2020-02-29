# PC-side program for logging recorded angles
# Author: Tyler
# Data: May 7, 2019

import os
import sys
import numpy as np
from util import *
from rx import record
from tx import *
from analyze import analyze, csv2wav
from filters import validate_smoothing_args

def main():
    args = parse_args()
    port = args['port']
    baud = args['baud']
    record_mode = args['record']
    csv2wav_fname = args['csv2wav']
    analyze_fname = args['analyze']
    smoothing_str = args['smoothing']
    animate_str = args['animate']
    playback_fname = args['playback']
    baseline_fname = args['set_baseline']
    angles = args['set_angles']
    origin_angles = args['update_origin']
    sine_params = args['sine']
    plot_slice = args['plot_slice']
    use_servos = args['use_servos']
    use_imus = args['use_imus']
    file_slice = args['file_slice']
    verbose = args['verbose']
    
    # Validate args!
    num_main_args = (playback_fname != '') + (analyze_fname != '') + \
        (baseline_fname != '') + (csv2wav_fname != '')
    if num_main_args > 1:
        logString("2 or more of: playback, analyze, set_baseline were selected."
                  " Please only choose one of these")
        quit()
    if plot_slice:
        validate_slice_args("plot_slice", plot_slice)
    if file_slice:
        file_slice_fname, file_slice_args = file_slice.split(",", 1)
        validate_slice_args("file_slice", file_slice_args)
    anim_data = (False, None, None)
    if animate_str:
        anim_data = validate_anim_args(animate_str)
    smoothing_data = (None, None)
    if smoothing_str:
        smoothing_data = validate_smoothing_args(smoothing_str)
    
    # Call requested function
    if csv2wav_fname:
        logString("Starting .csv to .wav conversion")
        csv2wav(csv2wav_fname, smoothing_data)
    elif analyze_fname:
        logString("Starting analysis")
        analyze( \
            analyze_fname, \
            args['imu'], \
            args['estimate'], \
            args['use_calibration'], \
            args['use_legacy_sign_convention'], \
            args['use_time_stamps'], \
            plot_slice, \
            args['make_wav'], \
            anim_data, \
            smoothing_data
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
    elif origin_angles:
        logString("Updating origin location")
        send_reference_point_update(port, baud, origin_angles)
    elif use_servos != None:
        change_servo_usage(port, baud, use_servos)
    elif use_imus != None:
        change_imu_usage(port, baud, use_imus)
    elif file_slice:
        do_file_slice(file_slice_fname, file_slice_args);
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