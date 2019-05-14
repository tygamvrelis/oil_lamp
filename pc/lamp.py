# PC-side program for logging recorded angles
# Author: Tyler
# Data: May 7, 2019

import os
import sys
import glob
import struct
import numpy as np
import matplotlib.pyplot as plt
from util import *
from rx import record
from analyze import analyze

def main():
    args = parse_args()
    port = args['port']
    baud = args['baud']
    log = args['log']
    analyze_fname = args['analyze']
    stream_fname = args['stream']
    verbose = args['verbose']
    
    if stream_fname and analyze_fname:
        logString("Cannot stream and analyze. Please only choose one of these")
        quit()
    
    cwd = os.getcwd()
    if analyze_fname:
        logString("Analysis not implemented")
        return
        analyze(analyze_fname)
    elif stream_fname:
        logString("Stream not implemented")
        return
    elif log:
        logString("Starting recording")
        record(port, baud, verbose)
    else:
        logString("No option selected")
        return

if __name__ == "__main__":
    try:
        main()
        sys.exit(0)
    except KeyboardInterrupt as e:
        print("Interrupted: {0}".format(e))
        sys.exit(1)