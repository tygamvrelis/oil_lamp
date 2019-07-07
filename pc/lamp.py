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
    use_servos = args['use_servos']
    use_imus = args['use_imus']
    verbose = args['verbose']
    
    if (playback_fname and analyze_fname) or \
       (playback_fname and baseline_fname) or \
       (analyze_fname and baseline_fname):
        logString("2 or more of: playback, analyze, set_baseline were selected."
                  " Please only choose one of these")
        quit()
    
    if analyze_fname:
        logString("Starting analysis")
        analyze(analyze_fname, args['imu'], args['estimate'], args['use_calibration'], args['use_legacy_sign_convention'])
    elif playback_fname:
        logString("Starting playback")
        playback(port, baud, playback_fname, args['loop'], args['use_legacy_sign_convention'], verbose)
    elif angles:
        logString("Setting servo angles")
        send_servo_angles(port, baud, angles)
    elif sine_params:
        logString("Sending sine wave")
        send_sine_wave(port, baud, sine_params, args['servo'])
    elif use_servos != None:
        if use_servos:
            enable_servos()
        else:
            disable_servos()
    elif use_imus != None:
        if use_servos:
            enable_imus()
        else:
            disable_imus()
    elif baseline_fname:
        logString("Creating baseline")
        set_baseline(baseline_fname, args['use_legacy_sign_convention'], verbose)
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