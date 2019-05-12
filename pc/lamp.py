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

    
# Array indices
BASE_OUTER = 0
BASE_INNER = 1
LAMP_OUTER = 2
LAMP_INNER = 3
def load_angles_from_file(file_name):
    '''
    Loads data from a file into an array

    ----------
    Parameters
    file_name : str
        Name of the file to load. Example: 07052019_23_28_36.dat.
    '''
    cwd = os.getcwd()
    fname = os.path.join(cwd, file_name)
    logString("Attempting to open angle data " + fname)
    with open(fname, "rb") as f:
        data = f.readlines()[0] # Files shouldn't be more than a few Mb max
    
    num_bytes = os.stat(fname).st_size
    logString("(Has {0} bytes)".format(num_bytes))
    assert(num_bytes % 16 == 0), "Binary has invalid length {0}".format(num_bytes)
    
    num_samples = num_bytes / 16
    
    angles = np.ndarray(shape=(4,num_samples))
    for i in range(num_samples):
        raw = data[i*16:(i + 1)*16]
        angles[:,i], synch = decode(raw)
    return angles, num_samples

def main():
    args = parse_args()
    port = args['port']
    baud = args['baud']
    log = args['log']
    analyze = args['analyze']
    stream = args['stream']
    verbose = args['verbose']
    
    if stream and analyze:
        logString("Cannot stream and analyze. Please only choose one of these")
        quit()
    
    cwd = os.getcwd()
    if analyze:
        logString("Analysis not implemented")
        return
        
        make_data_dir()
        if analyze == "latest":
            files = glob.glob(os.path.join(cwd, 'data' + os.sep + '*.dat'))
            analyze = max(files, key=os.path.getctime)

        SAMPLE_RATE = 100.0 # Hz
        angles, num_samples = load_angles_from_file(analyze)
        t = np.linspace(0, num_samples / SAMPLE_RATE, num=num_samples, endpoint=False)

        fig, ax = plt.subplots()
        size = 2
        
        ax.scatter(t, angles[BASE_OUTER], c="blue",  label="Base (outer)", s=size)
        ax.scatter(t, angles[BASE_INNER], c="red",   label="Base (inner)", s=size)
        ax.scatter(t, angles[LAMP_OUTER], c="green", label="Lamp (outer)", s=size)
        ax.scatter(t, angles[LAMP_INNER], c="black", label="Lamp (inner)", s=size)
        ax.legend()
        
        plt.title('Angles vs time')
        plt.xlabel('Time (s)')
        plt.ylabel('Angle ($^\circ$)')
        
        fig_name = os.path.join(cwd, os.path.splitext(analyze)[0] + '.png')
        plt.savefig(fig_name)
        logString("Saved fig to {0}".format(fig_name))
        plt.close();
    else:
        record(port, baud, verbose)

if __name__ == "__main__":
    try:
        main()
        sys.exit(0)
    except KeyboardInterrupt as e:
        print("Interrupted: {0}".format(e))
        sys.exit(1)