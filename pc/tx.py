# Transmission of logged data for actuation
# Author: Tyler
# Data: June 28, 2019

import time
import serial
import os
# import struct
# import numpy as np
from util import *

def transmit(ser, f):
    '''
    Sends data from the file to the MCU
    '''
    # TODO (tyler): figure out how we want to do this. Should be able to get
    # away with sending 2 bytes (I doubt we need better accuracy than int), but
    # we'll see
    return

def playback(port, baud, fname, verbose):
    '''
    Initiates playback mode
    '''
    logString("Playback not supported yet")
    return

    logString(list_ports())
    
    cwd = os.getcwd()
    fname = os.path.join(cwd, 'data', fname)
    logString("Loading data file " + fname)
    first = True
    with open(fname, "r") as f:
        logString("Attempting connection to embedded")
        logString("\tPort: " + port)
        logString("\tBaud rate: " + str(baud))
        
        num_tries = 0
        while True:
            try:
                with serial.Serial(port, baud, timeout=0) as ser:
                    logString("Connected")
                    while True:
                        transmit(ser, f)
            except serial.serialutil.SerialException as e:
                if(num_tries % 100 == 0):
                    if(str(e).find("FileNotFoundError")):
                        logString("Port not found. Retrying...(attempt {0})".format(num_tries))
                    else:
                        logString("Serial exception. Retrying...(attempt {0})".format(num_tries))
                time.sleep(0.01)
                num_tries = num_tries + 1