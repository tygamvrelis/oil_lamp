# Reception & logging of data
# Author: Tyler
# Data: May 12, 2019

from datetime import datetime
import time
import serial
import os
import struct
import numpy as np
from prettytable import PrettyTable
from util import *
    
def print_imu(data):
    '''
    Prints out a numpy vector interpreted as data from the 2 IMUs
    '''
    data = np.round(data, 2)
    t = PrettyTable(['', 'Base Accel (m/s^2)', 'Base Gyro (deg/s)', 'Lamp Accel (m/s^2)', 'Lamp Gyro (deg/s)'])
    t.add_row(["Z", data[0], data[3], data[6], data[9]])
    t.add_row(["Y", data[1], data[4], data[7], data[10]])
    t.add_row(["X", data[2], data[5], data[8], data[11]])
    print(t)

def log_preamble(file):
    preamble = "Order of data on each line is:\n"
    preamble = preamble + ("\tTime\n"
                          "\tStatus\n"
                          "\tBase:\n"
                          "\t\tAz\n"
                          "\t\tAy\n"
                          "\t\tAx\n"
                          "\t\tVz\n"
                          "\t\tVy\n"
                          "\t\tVx\n"
                          "\tLamp:\n"
                          "\t\tAz\n"
                          "\t\tAy\n"
                          "\t\tAx\n"
                          "\t\tVz\n"
                          "\t\tVy\n"
                          "\t\tVx\n"
                          "START\n")
    file.write(preamble)
                          
def log_data(file, buff):
    '''
    Records data to the log file and syncs log file to disk periodically
    '''
    try:
        log_data.n += 1
    except AttributeError:
        log_data.n = 0
    status = decode_status(buff)
    data_to_write = datetime.now().strftime('%H:%M:%S.%f')[:-3] + " "
    data_to_write = data_to_write + "stat=" + str(status) + " "
    file.write(data_to_write)       # Human-readable text, for time + flags
    file.write(get_imu_bytes(buff)) # Binary data. 96% of file if written as str
    file.write("\n")
    if log_data.n > 0 and log_data.n % 10 == 0:
        file.flush()
        os.fsync(file)
        imu = decode_data(buff)
        logString(str(status))
        print_imu(imu)

def receive(ser):
    '''
    Receives a single data packet from the MCU
    '''
    timeout = 0.015 # timeout in [s]
    time_start = time.time()
    time_curr = time_start
    done = False
    data_received = False
    receive_succeeded = False
    num_bytes_available = 0
    buff = bytes(''.encode())
    HEADER = 0xAA
    header_count = 0
    while True:
        # First, we wait until we have received some data, or until the timeout
        # has elapsed
        while(num_bytes_available == 0 and 
            not (data_received and time_curr - time_start >= timeout)):
            time.sleep(0.001)
            time_curr = time.time()
            num_bytes_available = ser.in_waiting
        
        if(num_bytes_available == 0 and
            (data_received and time_curr - time_start >= timeout)):
            break
        else:
            data_received = True
            
            # If we receive some data, we process it here then go back to
            # waiting for more
            raw_data = ser.read(num_bytes_available)
            for i in range(num_bytes_available):
                # 1. Find 0xAA 0xAA
                if header_count < 2:
                    if struct.unpack('<B', raw_data[i:i+1])[0] == HEADER:
                        header_count = header_count + 1
                    else:
                        header_count = 0
                else:
                    # 2. Unpack BUF_SIZE bytes and check for termination
                    buff = buff + raw_data[i:i+1]
                    done = len(buff) == BUF_SIZE + 1
                    if done:
                        # '\n' is used to terminate packets
                        receive_succeeded = struct.unpack('<c', buff[-1:])[0] == b'\n'
                        if receive_succeeded:
                            buff = buff[:-1] # Remove termination character
                        break
            num_bytes_available = 0
            if done:
                break
    assert(not receive_succeeded or len(buff) == BUF_SIZE)
    return (receive_succeeded, buff)

def record(port, baud, verbose):
    '''
    Initiates recording mode
    --------
    Arguments:
        port : serial.Serial
            COM port that MCU is connected to
        baud : int
            Symbol rate over COM port
        verbose : bool
            Prints additional messages if True
    '''
    logString(list_ports())
    make_data_dir()
    
    fname = get_log_file_name()
    logString("Creating data file " + fname)
    first = True
    with open(fname, 'wb') as f:
        log_preamble(f)
        logString("Attempting connection to embedded")
        logString("\tPort: " + port)
        logString("\tBaud rate: " + str(baud))
        
        num_tries = 0
        while True:
            try:
                with serial.Serial(port, baud, timeout=0) as ser:
                    logString("Connected")
                    enable_imus()
                    if first:
                        first = False
                        ser.write(CMD_BLINK.encode()) # L => flash LED
                    while True:
                        success, buff = receive(ser)
                        if success:
                            log_data(f, buff)
            except serial.serialutil.SerialException as e:
                if(num_tries % 100 == 0):
                    if(str(e).find("FileNotFoundError")):
                        logString("Port not found. Retrying...(attempt {0})".format(num_tries))
                    else:
                        logString("Serial exception. Retrying...(attempt {0})".format(num_tries))
                time.sleep(0.01)
                num_tries = num_tries + 1
