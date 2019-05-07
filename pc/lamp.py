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
        help='The serial port used for communication. Default: /dev/ttyUSB0',
        default='/dev/ttyACM0'
    )
    
    parser.add_argument(
        '--baud',
        help='Serial port baud rate. Default: 115200',
        default=115200
    )

    parser.add_argument(
        '--verbose',
        help='Display extra info/debug messages if True',
        default=False
    )

    return vars(parser.parse_args())

def decode(buff):
    l = []
    assert(len(buff) == 16)
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

def main():
    args = parse_args();
    port = 'COM15' #args['port']
    baud = args['baud']
    verbose = args['verbose']
    
    logString(list_ports())
    
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
                        if n > 0 and n % 10 == 0:
                            angles = decode(buff)
                            logString(
                                "\nBase estimates:\n" +
                                "\tOuter gimbal: " + str(angles[0]) + "\n"
                                "\tInnter gimbal: " + str(angles[1]) + "\n"
                                "Lamp estimates:\n" +
                                "\tOuter gimbal: " + str(angles[2]) + "\n"
                                "\tInnter gimbal: " + str(angles[3]) + "\n"
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