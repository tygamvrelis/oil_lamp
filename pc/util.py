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
        help='Loads the specified binary and plots the angle data. Must specify'
             ' the full file name (with extension), or "latest" to use the most'
             ' recent .dat',
        default=''
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

BUF_SIZE = 2*6*4 + 1 # 2 IMUs * 6 floats + 1 status byte
def decode(buff):
    l = []
    assert(len(buff) == BUF_SIZE), "Length is {0}".format(len(buff))
    for i in range(2*6):
        l.append(struct.unpack('<f', buff[i * 4:(i + 1) * 4])[0])
    status = struct.unpack('<B', buff[-1])[0]
    return np.array(l), status

def make_data_dir():
    cwd = os.getcwd()
    if not os.path.isdir(os.path.join(cwd, "data")):
        os.mkdir("data")
