# Reception & logging of data
# Author: Tyler
# Data: May 12, 2019

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
    t = PrettyTable(['', 'Base Gyro (deg/s)', 'Base Accel (m/s^2)', 'Lamp Gyro (deg/s)', 'Lamp Accel (m/s^2)'])
    t.add_row(["X", data[0], data[3], data[6], data[9]])
    t.add_row(["Y", data[1], data[4], data[7], data[10]])
    t.add_row(["Z", data[2], data[5], data[8], data[11]])
    print(t)

def log_preamble(file):
    preamble = "Order of data on each line is:\n"
    preamble = preamble + ("\tTime\n"
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
                          "\tStatus")
    file.write(preamble)
                          
def log_data(file, buff):
    try:
        log_data.n += 1
    except AttributeError:
        log_data.n = 0
    imu, status = decode(buff)
    # data_to_write = str(time.time) + " "
    # for i in range(2*6):
    #     data_to_write = data_to_write + str(imu[0]) + " "
    # data_to_write = data_to_write + " " + str(status)
    # file.write(data_to_write)
    if log_data.n > 0 and log_data.n % 10 == 0:
        logString(str(status))
        print_imu(imu)

def receive(ser):
    timeout = 0.015 # ms timeout
    time_start = time.time()
    time_curr = time_start
    receive_succeeded = False
    num_bytes_available = 0
    buff = bytes(''.encode())
    while True:
        # First, we wait until we have received some data, or until the timeout
        # has elapsed
        while((num_bytes_available == 0) and (time_curr - time_start < timeout)):
            time.sleep(0.001)
            time_curr = time.time()
            num_bytes_available = ser.in_waiting
        
        if((num_bytes_available == 0) and (time_curr - time_start >= timeout)):
            break
        else:
            # If we receive some data, we process it here then go back to
            # waiting for more
            rawData = ser.read(num_bytes_available)
            for i in range(num_bytes_available):
                # '\n' is used to terminate packets. If we see this, we need to
                # reset the buffer since the contents must have been shifted
                terminated = struct.unpack('<c', rawData[i:i+1])[0] == b'\n'
                if terminated:
                    if len(buff) == BUF_SIZE:
                        # If we get here, we have received a full packet
                        receive_succeeded = True
                        break
                    else:
                        buff = bytes(''.encode())
                else:
                    buff = buff + rawData[i:i+1]
            num_bytes_available = 0
            if receive_succeeded:
                break
    assert(not receive_succeeded or len(buff) == BUF_SIZE)
    return (receive_succeeded, buff)

def record(port, baud, verbose):
    logString(list_ports())
    make_data_dir()
    
    fname = datetime.now().strftime('data' + os.sep + '%d%m%Y_%H_%M_%S.dat')
    cwd = os.getcwd()
    fname = os.path.join(cwd, fname)
    logString("Creating data file " + fname)
    with open(fname, "wb") as f:
        log_preamble(f)
        logString("Attempting connection to embedded")
        logString("\tPort: " + port)
        logString("\tBaud rate: " + str(baud))
        
        num_tries = 0
        while True:
            try:
                with serial.Serial(port, baud, timeout=0) as ser:
                    logString("Connected")
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
