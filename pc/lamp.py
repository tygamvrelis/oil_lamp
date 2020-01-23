# PC-side program for logging recorded angles
# Author: Tyler
# Data: May 7, 2019

import os
import sys
import numpy as np
from util import *
from rx import record, record_networked
from tx import *
from analyze import analyze

def main():
    args = parse_args()
    port = args['port']
    baud = args['baud']
    record_mode = args['record']
    analyze_fname = args['analyze']
    is_networked = args['network']
    is_inet_dryrun = args['inet_dryrun']
    ip_addr = args['ip_addr']
    udp_port = args['udp_port']
    animate_str = args['animate']
    playback_fname = args['playback']
    baseline_fname = args['set_baseline']
    angles = args['set_angles']
    sine_params = args['sine']
    plot_slice = args['plot_slice']
    use_servos = args['use_servos']
    use_imus = args['use_imus']
    file_slice = args['file_slice']
    verbose = args['verbose']
    
    # Validate args!
    if (playback_fname and analyze_fname) or \
       (playback_fname and baseline_fname) or \
       (analyze_fname and baseline_fname):
        logString("2 or more of: playback, analyze, set_baseline were selected."
                  " Please only choose one of these")
        quit()
    if playback_fname:
        record_mode = False
    if plot_slice:
        validate_slice_args("plot_slice", plot_slice)
    if file_slice:
        file_slice_fname, file_slice_args = file_slice.split(",", 1)
        validate_slice_args("file_slice", file_slice_args)
    anim_data = (False, None, None)
    if animate_str:
        anim_data = validate_anim_args(animate_str)
    if is_networked:
        if record_mode and not ip_addr:
            print("ERROR: Must specify receiver's IP address")
            quit()
        if udp_port < 1025 or udp_port > 65535:
            print("UDP port is out of range [1025, 65535]")
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
            plot_slice, \
            anim_data
        )
    elif playback_fname:
        logString("Starting playback")
        if not is_networked:
            playback( \
                port, \
                baud, \
                playback_fname, \
                args['loop'], \
                args['use_legacy_sign_convention'], 
                args['use_time_stamps'], \
                verbose \
            )
        else:
            playback_networked( \
                port, \
                baud, \
                udp_port, \
                verbose, \
                is_inet_dryrun \
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
        if not is_networked:
            record(port, baud, verbose)
        else:
            record_networked( \
                port, \
                baud, \
                ip_addr, \
                udp_port, \
                verbose, \
                is_inet_dryrun \
            )
    else:
        logString("No option selected")

if __name__ == "__main__":
    try:
        main()
        sys.exit(0)
    except KeyboardInterrupt as e:
        print("Interrupted: {0}".format(e))
        sys.exit(1)