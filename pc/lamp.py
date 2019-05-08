# PC-side program for logging recorded angles
# Author: Tyler
# Data: May 7, 2019

import argparse
from datetime import datetime
import time
import serial
import serial.tools.list_ports
import os
import sys
import glob
import struct
import numpy as np
import matplotlib.pyplot as plt
    
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

def decode(buff):
    l = []
    assert(len(buff) == 16), "Length is {0}".format(len(buff))
    for i in range(4):
        l.append(struct.unpack('<f', buff[i * 4:(i + 1) * 4])[0])
    return np.array(l)

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
                    if len(buff) == 16:
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
    assert(not receive_succeeded or len(buff) == 16)
    return (receive_succeeded, buff)
    
# Array indices
BASE_OUTER = 0
BASE_INNER = 1
LAMP_OUTER = 2
LAMP_INNER = 3
def load_angles_from_file(file_name):
    '''
    Loads binary data from a file, converts it back to floats, and stores it in
    an array.

    ----------
    Parameters
    file_name : str
        Name of the file to load. Example: angles_07052019_23_28_36.dat.
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
        angles[:,i] = decode(raw)
    return angles, num_samples

def make_data_dir():
    cwd = os.getcwd()
    if not os.path.isdir(os.path.join(cwd, "data")):
        os.mkdir("data")

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
        logString(list_ports())
        make_data_dir()
        
        fname = datetime.now().strftime('data' + os.sep + 'angles_%d%m%Y_%H_%M_%S.dat')
        cwd = os.getcwd()
        fname = os.path.join(cwd, fname)
        logString("Creating data file " + fname)
        with open(fname, "wb") as f:
            logString("Attempting connection to embedded")
            logString("\tPort: " + port)
            logString("\tBaud rate: " + str(baud))
            
            num_tries = 0
            while True:
                try:
                    with serial.Serial(port, baud, timeout=0) as ser:
                        logString("Connected")
                        n = 0
                        while True:
                            success, buff = receive(ser)
                            if success:
                                n = n + 1
                                f.write(buff)
                                if n > 0 and n % 10 == 0:
                                    angles = decode(buff)
                                    logString(
                                        "\nBase estimates:\n" +
                                        "\tOuter gimbal: " + str(angles[0]) + "\n"
                                        "\tInnter gimbal: " + str(angles[1]) + "\n"
                                        "Lamp estimates:\n" +
                                        "\tOuter gimbal: " + str(angles[2]) + "\n"
                                        "\tInner gimbal: " + str(angles[3]) + "\n"
                                    )
                except serial.serialutil.SerialException as e:
                    if(num_tries % 100 == 0):
                        if(str(e).find("FileNotFoundError")):
                            logString("Port not found. Retrying...(attempt {0})".format(num_tries))
                        else:
                            logString("Serial exception. Retrying...(attempt {0})".format(num_tries))
                    time.sleep(0.01)
                    num_tries = num_tries + 1

if __name__ == "__main__":
    try:
        main()
        sys.exit(0)
    except KeyboardInterrupt as e:
        print("Interrupted: {0}".format(e))
        sys.exit(1)