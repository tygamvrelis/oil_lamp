# Utilities
# Author: Tyler
# Data: May 12, 2019

from datetime import datetime
import serial.tools.list_ports
import argparse
import os
import sys
import struct
import numpy as np
    
def logString(userMsg):
    '''
    Prints the desired string to the shell, precedded by the date and time.
    '''
    print(datetime.now().strftime('%H.%M.%S.%f') + " " + userMsg)

def list_ports():
    ports = serial.tools.list_ports.comports()
    msg = ""
    if(len(ports) == 0):
        msg = "Error: No COM ports have been detected"
    else:
        ports = [port.device for port in ports]
        msg = "Available ports are: " + " ".join(ports)
    return msg

def get_script_path():
    return os.path.dirname(os.path.realpath(sys.argv[0]))
    
def parse_args():
    os.chdir(get_script_path())
    logString("Starting PC-side application")
    
    parser = argparse.ArgumentParser(description='Lamp')
    parser.add_argument(
        '--port',
        help='The serial port used for communication. Default: COM15',
        default='COM15'
    )
    
    parser.add_argument(
        '--baud',
        help='Serial port baud rate. Default: 115200',
        default=115200
    )
    
    parser.add_argument(
        '--log',
        help='Logs angles to a file if True. Default: True',
        default=True
    )
    
    parser.add_argument(
        '--analyze',
        help='Loads the specified binary and plots the data. Must specify the'
             ' full file name (with extension), or "latest" to use the most'
             ' recent .dat. See the plot argument as well.',
        default=''
    )
    
    parser.add_argument(
        '--plot',
        help='Specifies whether to plot lamp or base data when running an'
             ' analysis flow. Arguments: lamp, base, both. Default: base',
        default='base'
    )
    
    parser.add_argument(
        '--estimate',
        help='Specifies whether to plot angle estimates or raw data when running'
             ' an analysis flow. Arguments: ind_angles, comb_angles, none.'
             ' Default: none',
        default='none'
    )
    
    parser.add_argument(
        '--stream',
        help='Streams the specified angle data to the microcontroller for'
             'playback. NOT SUPPORTED YET.',
        default=''
    )

    parser.add_argument(
        '--verbose',
        help='Display extra info/debug messages if True',
        default=False
    )

    return vars(parser.parse_args())

IMU_BUF_SIZE = 2*6*4 # 2 IMUs * 6 floats
BUF_SIZE = IMU_BUF_SIZE + 1 # 1 status byte
IMU_BASE_IDX = 0
IMU_LAMP_IDX = 6
ACC_IDX = 0
GYRO_IDX = 3
Z_IDX = 0
Y_IDX = 1
X_IDX = 2
def get_imu_bytes(buff):
    imu = buff[0:IMU_BUF_SIZE]
    assert(len(imu) == IMU_BUF_SIZE), "Wrong IMU buff size"
    return imu

def get_status_byte(buff):
    return buff[-1]
    
def decode_data(buff):
    l = []
    assert(len(buff) == BUF_SIZE or len(buff) == IMU_BUF_SIZE), "Length is {0}".format(len(buff))
    num_iter = -1
    if (len(buff) == BUF_SIZE):
        num_iter = int((BUF_SIZE - 1) / 4)
    else:
        num_iter = int(IMU_BUF_SIZE / 4)
    for i in range(num_iter):
        l.append(struct.unpack('<f', buff[i * 4:(i + 1) * 4])[0])
    return np.array(l)

def decode_status(buff):
    assert(len(buff) == BUF_SIZE), "Length is {0}".format(len(buff))
    status = struct.unpack('<B', get_status_byte(buff))[0]
    return status

def get_data_dir():
    cwd = os.getcwd()
    return os.path.join(cwd, "data")

def make_data_dir():
    cwd = os.getcwd()
    if not os.path.isdir(get_data_dir()):
        os.mkdir("data")

def load_data_from_file(file_name):
    '''
    Loads data from a file

    ----------
    Parameters
    file_name : str
        Name of the file to load. Example: 07052019_23_28_36.dat.
    '''

    fname = os.path.join(get_data_dir(), file_name)
    logString("Attempting to open data " + fname)
    with open(fname, "rb") as f:
        bin_data = f.readlines() # Files shouldn't be more than a few Mb max
        
    # Find starting index of actual data
    idx = -1
    for i in range(len(bin_data)):
        if bin_data[i].decode().strip() == "START":
            idx = i
            break
    assert(i != -1), "Invalid data file...could not find START marker"
    bin_data = bin_data[i+1:]
    
    num_samples = len(bin_data)
    
    imu_data = np.ndarray(shape=(int(IMU_BUF_SIZE / 4),num_samples))
    for i in range(num_samples):
        imu_data[:,i] = decode_data(bin_data[i][15:-1])
    return imu_data, num_samples
