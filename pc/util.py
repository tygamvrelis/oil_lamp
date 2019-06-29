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
import configparser
    
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
        '--imu',
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
        '--playback',
        help='Streams the specified angle data to the microcontroller for'
             'playback. NOT SUPPORTED YET.',
        default=''
    )

    parser.add_argument(
        '--set_baseline',
        help='Specifies a file to use as a measurement baseline. This works just'
             ' like the reset button on a scale -- it says THIS is the data'
             ' recorded when the lamp is completely level. This is used to'
             ' account for the fact that the IMUs are mounted at angles relative'
             ' to the lamp and base. Only affects analysis and playback modes'
             ' (does not interefere with raw data that is being recorded). Only'
             ' changed ADDITIVE baseline factor',
        default=''
    )
    
    parser.add_argument(
        '--use_baseline_calibration',
        help='Uses the baseline data in settings.ini to adjust the frame of'
             ' reference to account for IMU orientation relative to the base and'
             ' lamp, if True. If False, uses vanilla data. Using baseline'
             ' calibration will not modify raw data from a recording',
        default=False
    )

    parser.add_argument(
        '--verbose',
        help='Display extra info/debug messages if True',
        default=False
    )

    return vars(parser.parse_args())

IMU_BUF_SIZE = 2*6*4 # 2 IMUs * 6 floats
BUF_SIZE = IMU_BUF_SIZE + 1 # 1 status byte
LOGGED_BUF_SIZE = BUF_SIZE + 20 # 20 bytes of plaintext for time + status
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
    '''
    Converts the binary buffer to floats
    '''
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

def load_data_from_file(file_name, use_baseline_calibration=False):
    '''
    Loads data from a file

    ----------
    Arguments
        file_name : str
            Name of the file to load. Example: 07052019_23_28_36.dat
        use_baseline_calibration : bool
            If True, loads the baseline data (if it exists) and applies the
            transformations to the data just loaded
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
    assert(idx != -1), "Invalid data file...could not find START marker"
    bin_data = bin_data[idx+1:]
    
    # May need to adjust lines due to data looking like newline character
    too_small = [(line, idx) for idx, line in enumerate(bin_data) if len(line) != LOGGED_BUF_SIZE]
    idx_to_del = [e[1] for e in too_small]
    assert(len(too_small) == 0 or len(too_small) > 1), "Invalid number of lines are too small"
    if len(too_small) > 0:
        cur_line, cur_ind = too_small.pop(0)
    while len(too_small) > 0:
        line, ind = too_small.pop(0)
        if len(bin_data[cur_ind]) < LOGGED_BUF_SIZE:
            bin_data[cur_ind] = bin_data[cur_ind] + line
        else:
            cur_ind = ind
    for index in sorted(idx_to_del, reverse=True):
        del bin_data[index]
    
    num_samples = len(bin_data)
    
    # Interpret the binary data as floats
    imu_data = np.ndarray(shape=(int(IMU_BUF_SIZE / 4),num_samples))
    for i in range(num_samples):
        imu_data[:,i] = decode_data(bin_data[i][20:-1])
    
    if use_baseline_calibration:
        # Apply transformations if requested
        imu_data = apply_baseline_transformations(imu_data)
    
    return imu_data, num_samples

def get_calibration_file_name():
    return 'settings.ini';

def get_calibration_file_preamble():
    '''
    Writes a comment into the settings file to describe what the constants do.
    INI file comments are not extracted by configparser, so this has to be
    added in manually each time we update the settings file
    '''
    message = (""
        "# These constants are added or multiplied to all data loaded for analysis or\n"
        "# playback. They account for the fact that the IMUs are mounted at angles\n"
        "# relative to the lamp and base. These constants were derived from a recording\n"
        "# where the lamp and base were stationary and level.\n"
        "#\n"
        "# Example: let Az denote the raw acceleration in the z direction and Az' denote\n"
        "#     this same acceleration after applying the constants. Then the following\n"
        "#     relation is true:\n"
        "#         Az' = Az * mult_z + add_Az\n"
        "#\n"
        "# For the gyroscope data, there is no need for offsets since all the data is\n"
        "# high-pass.\n"
        "#\n"
        "# Multiplicative constants are the same for accelerometer and gyroscope\n")
    return message

def set_baseline(baseline_fname, verbose):
    '''
    Uses data from a file to create a new baseline (only changes additive factor)
    --------
    Arguments:
        baseline_fname : str
            The name of the data file to use as a baseline
        verbose : bool
            Prints additional messages if True
    '''
    # Load data to use for new baseline. Skip first 100 samples or so in order
    # to reach the steady state filter output
    imu_data, num_samples = load_data_from_file(baseline_fname, False)
    imu_data = imu_data[:,100:]
    
    # Load existing baseline for the multiplicative factors
    lamp_mult, lamp_add_a, base_mult, base_add_a = load_calibration_data()
    
    # Compute new baseline
    target = np.array([9.81, 0, 0])
    # Lamp
    IDX = IMU_LAMP_IDX + ACC_IDX
    imu_data[IDX:IDX+3,:] = lamp_mult.dot(imu_data[IDX:IDX+3,:])
    lamp_med = np.median(imu_data[IDX:IDX+3,:], axis=1)
    lamp_acc_err = np.round(target - lamp_med, 3)
    if verbose:
        print("Median lamp acceleration values: ", lamp_med)
        print("\tError: ", lamp_acc_err)
    
    # Base
    IDX = IMU_BASE_IDX + ACC_IDX
    imu_data[IDX:IDX+3,:] = base_mult.dot(imu_data[IDX:IDX+3,:])
    base_med = np.median(imu_data[IDX:IDX+3,:], axis=1)
    base_acc_err = np.round(target - base_med, 3)
    if verbose:
        print("Median base acceleration values: ", base_med)
        print("\tError: ", base_acc_err)
    
    # Update calibration data
    config = configparser.ConfigParser()
    config.read(get_calibration_file_name())
    config.set('Lamp IMU', 'add_Az', str(lamp_acc_err[0]))
    config.set('Lamp IMU', 'add_Ay', str(lamp_acc_err[1]))
    config.set('Lamp IMU', 'add_Ax', str(lamp_acc_err[2]))
    config.set('Base IMU', 'add_Az', str(base_acc_err[0]))
    config.set('Base IMU', 'add_Ay', str(base_acc_err[1]))
    config.set('Base IMU', 'add_Ax', str(base_acc_err[2]))
    with open(get_calibration_file_name(), 'w') as f:
        f.write(get_calibration_file_preamble())
    with open(get_calibration_file_name(), 'a+') as f:
        config.write(f)

def load_calibration_data():
    '''
    Loads previously-saved calibration data, if it exists'
    '''
    config = configparser.ConfigParser()
    config.read(get_calibration_file_name())
    
    # Equ'n: q' = Aq+b
    lamp_mult = np.identity(3)
    lamp_add_a = np.array([0, 0, 0])

    base_mult = np.identity(3)
    base_add_a = np.array([0, 0, 0])
    if len(config.sections()) == 0:
        logString("WARNING: calibration data is empty!")
    else:
        lamp_mult[0,0] = config['Lamp IMU']['mult_z']
        lamp_mult[1,1] = config['Lamp IMU']['mult_y']
        lamp_mult[2,2] = config['Lamp IMU']['mult_x']
        lamp_add_a[0] = config['Lamp IMU']['add_Az']
        lamp_add_a[1] = config['Lamp IMU']['add_Ay']
        lamp_add_a[2] = config['Lamp IMU']['add_Ax']
        base_mult[0,0] = config['Base IMU']['mult_z']
        base_mult[1,1] = config['Base IMU']['mult_y']
        base_mult[2,2] = config['Base IMU']['mult_x']
        base_add_a[0] = config['Base IMU']['add_Az']
        base_add_a[1] = config['Base IMU']['add_Ay']
        base_add_a[2] = config['Base IMU']['add_Ax']
    
    return lamp_mult, lamp_add_a, base_mult, base_add_a

def apply_baseline_transformations(data):
    '''
    Transforms the raw sensor data to account for the orientation of the IMUs in
    the setup -- calibration.
    --------
    Arguments:
        data : np.ndarray
            Array of IMU data
    '''
    lamp_mult, lamp_add_a, base_mult, base_add_a = load_calibration_data()
    
    # Lamp
    IDX = IMU_LAMP_IDX + ACC_IDX
    data[IDX:IDX+3,:] = lamp_mult.dot(data[IDX:IDX+3,:]) + lamp_add_a
    
    IDX = IMU_LAMP_IDX + GYRO_IDX
    data[IDX:IDX+3,:] = lamp_mult.dot(data[IDX:IDX+3,:])
    
    # Base
    IDX = IMU_BASE_IDX + ACC_IDX
    data[IDX:IDX+3,:] = base_mult.dot(data[IDX:IDX+3,:]) + base_add_a
    
    IDX = IMU_BASE_IDX + GYRO_IDX
    data[IDX:IDX+3,:] = base_mult.dot(data[IDX:IDX+3,:])

    return data
    