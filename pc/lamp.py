# PC-side program for logging recorded angles
# Author: Tyler
# Data: May 7, 2019

import os
import sys
import numpy as np
from util import *
from rx import record
from tx import playback
from analyze import analyze

def main():
    args = parse_args()
    port = args['port']
    baud = args['baud']
    log = args['log']
    analyze_fname = args['analyze']
    playback_fname = args['playback']
    baseline_fname = args['set_baseline']
    verbose = args['verbose']
    
    if (playback_fname and analyze_fname) or \
       (playback_fname and baseline_fname) or \
       (analyze_fname and baseline_fname):
        logString("2 or more of: playback, analyze, set_baseline were selected. Please only choose one of these")
        quit()
    
    if analyze_fname:
        logString("Starting analysis")
        analyze(analyze_fname, args['imu'], args['estimate'], args['use_calibration'])
    elif playback_fname:
        logString("Starting playback")
        playback(port, baud, playback_fname, verbose)
    elif baseline_fname:
        logString("Creating baseline")
        set_baseline(baseline_fname, verbose)
    elif log:
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