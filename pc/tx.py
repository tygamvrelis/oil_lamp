# Transmission of logged data for actuation
# Author: Tyler
# Data: June 28, 2019

import time
import serial
import os
# import struct
import numpy as np
from util import *
from analyze import get_angles

def transmit(ser, a_outer, a_inner):
    '''
    Sends angle data to the MCU
    '''
    # TODO (tyler): figure out how we want to do this. Should be able to get
    # away with sending 2 bytes (I doubt we need better accuracy than int), but
    # we'll see
    a_outer = int(np.round(a_outer))
    a_inner = int(np.round(a_inner))
    
    cmd_id = struct.pack('<c', 'A') # A => Angle payload
    payload = struct.pack('<B', a_outer) + struct.pack('<B', a_inner)
    packet = cmd_id + payload
    
    ser.write(packet)
    return

def playback(port, baud, fname, loop, verbose):
    '''
    Initiates playback mode
    --------
    Arguments:
        port : serial.Serial
            COM port that MCU is connected to
        baud : int
            Symbol rate over COM port
        fname : str
            Name of log file used to derive angles
        loop : bool
            Determines whether to quit or restart after sending all angles
        verbose : bool
            Prints additional messages if True
    '''
    logString("Playback not supported yet")
    return
    
    fname = os.path.join(get_data_dir(), fname)
    logString("Loading data file " + fname)
    imu_data, num_samples = load_data_from_file(fname, True)
    angles = get_angles(imu_data, num_samples)
    
    logString(list_ports())
    logString("Attempting connection to embedded")
    logString("\tPort: " + port)
    logString("\tBaud rate: " + str(baud))
    
    tx_cycle = WaitForMs(10) # 10 ms wait between angle transmissions
    tx_cycle.set_e_gain(1.5)
    tx_cycle.set_e_lim(0, -3.0) # Never wait longer, but allow down to 3 ms less
    
    num_tries = 0
    while True:
        try:
            with serial.Serial(port, baud, timeout=0) as ser:
                logString("Connected")
                if loop:
                    logString("Looping is enabled...will send angles forever")
                else:
                    logString("Looping is disabled...will quit after sending all angles")
                while True:
                    for angle_vec in angles:
                        outer_angle = angle_vec[BASE_OUTER] + angle_vec[LAMP_OUTER]
                        inner_angle = angle_vec[BASE_INNER] + angle_vec[LAMP_INNER]
                        # Send angles and wait a few ms before sending again
                        transmit(ser, outer_angle, inner_angle)
                        tx_cycle.wait()
                    if not loop:
                        return
        except serial.serialutil.SerialException as e:
            if(num_tries % 100 == 0):
                if(str(e).find("FileNotFoundError")):
                    logString("Port not found. Retrying...(attempt {0})".format(num_tries))
                else:
                    logString("Serial exception. Retrying...(attempt {0})".format(num_tries))
            time.sleep(0.01)
            num_tries = num_tries + 1