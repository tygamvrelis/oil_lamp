# Utilities
# Author: Tyler
# Data: May 12, 2019

from datetime import datetime
import serial.tools.list_ports
import serial
import argparse
import os
import sys
import struct
import numpy as np
import configparser
import time

def logString(userMsg):
    '''
    Prints the desired string to the shell, precedded by the date and time.
    '''
    print(datetime.now().strftime('%H.%M.%S.%f') + " " + userMsg)

def list_ports():
    '''
    Lists the available serial ports
    '''
    ports = serial.tools.list_ports.comports()
    msg = ""
    if(len(ports) == 0):
        msg = "Error: No COM ports have been detected"
    else:
        ports = [port.device for port in ports]
        msg = "Available ports are: " + " ".join(ports)
    return msg

def get_script_path():
    '''
    Gets the path where the script is running
    '''
    return os.path.dirname(os.path.realpath(sys.argv[0]))

def str2bool(v):
    '''
    https://stackoverflow.com/questions/15008758/parsing-boolean-values-with-argparse
    '''
    if isinstance(v, bool):
       return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

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
        '--record',
        help='Records raw data to a file if True. Default: True',
        type=str2bool,
        default=True
    )
    
    parser.add_argument(
        '--analyze',
        help='Loads the specified binary and plots the data. Must specify the'
             ' full file name (with extension), or "latest" to use the most'
             ' recent .dat.',
        default=''
    )
    
    parser.add_argument(
        '--imu',
        help='(analyze option) Specifies whether to plot lamp or base data when'
             ' Arguments: lamp, base, both. Default: base',
        default='base'
    )
    
    parser.add_argument(
        '--estimate',
        help='(analyze option) Specifies whether to plot angle estimates or raw'
             ' data. Arguments: ind_angles, comb_angles, none.'
             ' Default: none',
        default='none'
    )
    
    parser.add_argument(
        '--set_baseline',
        help=' Specifies a file to use for generating calibration offsets. This '
             ' can be used to account for the IMUs being mounted at angles '
             ' relative to the lamp and base',
        default=''
    )
    
    parser.add_argument(
        '--use_calibration',
        help='(set_baseline option) Uses the calibration data in settings.ini to '
             ' account for IMU orientation relative to the base and lamp, if '
             ' True',
        type=str2bool,
        default=True
    )
    
    parser.add_argument(
        '--use_legacy_sign_convention',
        help='(analyze, playback, and set_baseline option) Set this to True for '
             ' data recorded using microcontroller firmware older than July '
             ' 2019. Set to False otherwise',
        type=str2bool,
        default=False
    )

    parser.add_argument(
        '--use_time_stamps',
        help='(analyze and playback option) Specifies whether to use time '
             ' stamps (true time) to construct the time series, or to blindly '
             ' trust sampling rate * number of samples'
             ' Default: True',
        default=True
    )
    
    parser.add_argument(
        '--playback',
        help='Streams the specified angle data to the microcontroller for'
             ' playback',
        default=''
    )
    
    parser.add_argument(
        '--loop',
        help='(playback option) If we go through all the angles, begin again if'
             ' this is True. Otherwise, quit',
        type=str2bool,
        default=True
    )
    
    parser.add_argument(
        '--set_angles',
        help='Sets the servos to the specified angles. The first argument is the'
             ' outer gimbal angle and the second argument is the inner gimbal'
             ' angle. If only one of the angles is specified, the other will'
             ' default to 0. Example: --set_angles=-10,5',
        default=''
    )
    
    parser.add_argument(
        '--sine',
        help='Sends a sine wave of a given frequency and amplitude to the'
             ' servos. For example --sine=1.0 sends a sine wave of 1 Hz, and'
             ' --sine=1,22.5 sends a sine wave of 1 Hz with amplitude 22.5.'
             ' Default frequency is 1.0 Hz and default amplitude is 40.0. The'
             ' third argument is a time constant for the envelope (default 0).'
             ' For example, --sine=1,22.5,1 decreases in amplitude by a factor'
             ' of 2 at time t=ln(2)/1 = 0.693 seconds',
        default=''
    )
    
    parser.add_argument(
        '--servo',
        help='(sine option) Specifies which servo to send the angles to. Options'
             ' are: outer, inner, both. Default: both. If only one servo is'
             ' specified, the other will default to 0',
        default='both'
    )
    
    parser.add_argument(
        '--use_servos',
        help='Enables servo actuation on MCU if True, disables it if False',
        type=str2bool
    )
    
    parser.add_argument(
        '--use_imus',
        help='Enables IMU sensing on MCU if True, disables it if False',
        type=str2bool
    )
    
    parser.add_argument(
        '--verbose',
        help='Display extra info/debug messages if True',
        type=str2bool,
        default=False
    )

    return vars(parser.parse_args())

CMD_BLINK = 'L'
CMD_CTRL_DI = '0'
CMD_CTRL_EN = '1'
CMD_SENS_DI = '2'
CMD_SENS_EN = '3'
CMD_ANGLE = 'A'
def enable_servos(ser):
    '''
    Sends the MCU a command to enable servo actuation
    --------
    Arguments:
        ser : serial.Serial
            COM port that MCU is connected to
    '''
    logString("Enabling servos")
    ser.write(CMD_CTRL_EN.encode())

def disable_servos(ser):
    '''
    Sends the MCU a command to disable servo actuation
    --------
    Arguments:
        ser : serial.Serial
            COM port that MCU is connected to
    '''
    logString("Disabling servos")
    ser.write(CMD_CTRL_DI.encode())

def enable_imus(ser):
    '''
    Sends the MCU a command to enable IMU sensing
    --------
    Arguments:
        ser : serial.Serial
            COM port that MCU is connected to
    '''
    logString("Enabling IMUs")
    ser.write(CMD_SENS_EN.encode())

def disable_imus(ser):
    '''
    Sends the MCU a command to disable IMU sensing
    --------
    Arguments:
        ser : serial.Serial
            COM port that MCU is connected to
    '''
    logString("Disabling IMUs")
    ser.write(CMD_SENS_DI.encode())

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
    '''
    Gets the part of the buffer containing IMU data
    '''
    imu = buff[0:IMU_BUF_SIZE]
    assert(len(imu) == IMU_BUF_SIZE), "Wrong IMU buff size"
    return imu

def get_status_byte(buff):
    '''
    Gets the part of the buffer containing the status byte
    '''
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
    '''
    Decodes the status byte from binary
    '''
    assert(len(buff) == BUF_SIZE), "Length is {0}".format(len(buff))
    status = struct.unpack('<B', get_status_byte(buff))[0]
    return status

def is_raspberry_pi():
    '''
    Returns True if running on the Raspberry Pi
    '''
    return "linux" in sys.platform.lower()

def get_data_dir():
    '''
    Returns the directory path where motion data is stored
    '''
    if is_raspberry_pi():
        return "/boot/lamp_data"
    else:
        cwd = os.getcwd()
        return os.path.join(cwd, "data")

def make_data_dir():
    '''
    Created the directory for data storage if it doesn't already exist
    '''
    if not os.path.isdir(get_data_dir()):
        os.mkdir(get_data_dir())

def uniquify(file_name):
    '''
    Makes a file name unique, if needed
    '''
    dir_name = os.path.dirname(file_name)
    files = os.listdir(dir_name)
    if len(files) == 0:
        return file_name
    # Remove extensions from names
    files = [os.path.splitext(name)[0] for name in files]
    file_name_no_ext = os.path.splitext(os.path.basename(file_name))[0]
    name = file_name_no_ext
    idx = 0
    while name in files:
        name = file_name_no_ext + "_" + str(idx)
        idx = idx + 1
    return os.path.join(dir_name, name)

def get_log_file_name():
    '''
    Gets the name of the file to use for recording data
    '''
    if is_raspberry_pi():
        fname = 'lamp_data'
    else:
        fname = datetime.now().strftime('%d%m%Y_%H_%M_%S')
    fname = os.path.join(get_data_dir(), fname)
    fname = uniquify(fname)
    fname = fname + '.dat'
    return fname

def get_calibration_file_name():
    '''
    Returns the name of the file used for storing calibration settings
    '''
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

def set_baseline(baseline_fname, use_legacy_sign_convention, verbose):
    '''
    Uses data from a file to create a new baseline (only changes additive factor)
    --------
    Arguments:
        baseline_fname : str
            The name of the data file to use as a baseline
        use_legacy_sign_convention : bool
            If True, transforms the data set from the old acceleration sign
            convention to the new one. Meant for data sets recorded prior to
            July 2019
        verbose : bool
            Prints additional messages if True
    '''
    # Load data to use for new baseline. Skip first 100 samples or so in order
    # to reach the steady state filter output
    imu_data, num_samples = load_data_from_file(baseline_fname, False)
    imu_data = imu_data[:,100:]
    
    # Load existing baseline for the multiplicative factors
    lamp_mult, lamp_add_a, base_mult, base_add_a = load_calibration_data(use_legacy_sign_convention)
    
    # Compute new baseline
    target = np.array([-9.81, 0, 0])
    # Lamp
    IDX = IMU_LAMP_IDX + ACC_IDX
    lamp_med = np.median(imu_data[IDX:IDX+3,:], axis=1)
    if verbose:
        logString("Median lamp acceleration values: {0}".format(lamp_med))
    lamp_med = lamp_mult.dot(lamp_med)
    lamp_acc_err = np.round(target - lamp_med, 3)
    logString("\tNew lamp calibration values: {0}".format(lamp_acc_err))
    
    # Base
    IDX = IMU_BASE_IDX + ACC_IDX
    base_med = np.median(imu_data[IDX:IDX+3,:], axis=1)
    if verbose:
        logString("Median base acceleration values: {0}".format(base_med))
    base_med = base_mult.dot(base_med)
    base_acc_err = np.round(target - base_med, 3)
    logString("\tNew base calibration values: {0}".format(base_acc_err))
    
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
        f.write("#-------------------------------------------------------------------------------\n")
        f.write("# Calibrated from file {0} on {1}\n".format(baseline_fname, datetime.now()))
        f.write("#-------------------------------------------------------------------------------\n")
    with open(get_calibration_file_name(), 'a+') as f:
        config.write(f)

def load_calibration_data(use_legacy_sign_convention=False):
    '''
    Loads previously-saved calibration data, if it exists
    --------
    Arguments
        use_legacy_sign_convention : bool
            If True, adjusts the calibration transformations to account for the
            legacy sign convention. This is meant for data sets recorded prior
            to July 2019
    '''
    config = configparser.ConfigParser()
    config.read(get_calibration_file_name())
    
    # Equ'n: q' = mq+a
    lamp_mult = np.identity(3)
    lamp_add_a = np.array([0, 0, 0], dtype=np.float)

    base_mult = np.identity(3)
    base_add_a = np.array([0, 0, 0], dtype=np.float)
    if len(config.sections()) == 0:
        logString("WARNING: calibration data is empty!")
    else:
        lamp_mult[0,0] = float(config['Lamp IMU']['mult_z'])
        lamp_mult[1,1] = float(config['Lamp IMU']['mult_y'])
        lamp_mult[2,2] = float(config['Lamp IMU']['mult_x'])
        lamp_add_a[0]  = float(config['Lamp IMU']['add_az'])
        lamp_add_a[1]  = float(config['Lamp IMU']['add_ay'])
        lamp_add_a[2]  = float(config['Lamp IMU']['add_ax'])
        base_mult[0,0] = float(config['Base IMU']['mult_z'])
        base_mult[1,1] = float(config['Base IMU']['mult_y'])
        base_mult[2,2] = float(config['Base IMU']['mult_x'])
        base_add_a[0]  = float(config['Base IMU']['add_az'])
        base_add_a[1]  = float(config['Base IMU']['add_ay'])
        base_add_a[2]  = float(config['Base IMU']['add_ax'])
        logString("Loaded existing calibration data")
    if use_legacy_sign_convention:
        lamp_mult *= -1.0
        base_mult *= -1.0
    return lamp_mult, lamp_add_a, base_mult, base_add_a

def apply_baseline_transformations(data, use_legacy_sign_convention):
    '''
    Transforms the raw sensor data to account for the orientation of the IMUs in
    the setup -- calibration.
    --------
    Arguments:
        data : np.ndarray
            Array of IMU data
        use_legacy_sign_convention : bool
            If True, adjusts the calibration transformations to account for the
            legacy sign convention. This is meant for data sets recorded prior
            to July 2019
    '''
    lamp_mult, lamp_add_a, base_mult, base_add_a = load_calibration_data(use_legacy_sign_convention)
    
    # Lamp
    IDX = IMU_LAMP_IDX + ACC_IDX
    data[IDX:IDX+3,:] = (lamp_mult.dot(data[IDX:IDX+3,:]).T + lamp_add_a[:,]).T
    
    IDX = IMU_LAMP_IDX + GYRO_IDX
    data[IDX:IDX+3,:] = lamp_mult.dot(data[IDX:IDX+3,:])
    
    # Base
    IDX = IMU_BASE_IDX + ACC_IDX
    data[IDX:IDX+3,:] = (base_mult.dot(data[IDX:IDX+3,:]).T + base_add_a[:,]).T
    
    IDX = IMU_BASE_IDX + GYRO_IDX
    data[IDX:IDX+3,:] = base_mult.dot(data[IDX:IDX+3,:])

    return data

def apply_legacy_sign_convention(data):
    '''
    Data recorded before July 2019 used the opposite sign convention for the
    acceleration vector. This function is meant to operate on data sets recorded
    before July 2019 to make them compatible with the rest of the code.
    --------
    Arguments:
        data : np.ndarray
            Array of IMU data
    '''
    # Lamp
    IDX = IMU_LAMP_IDX + ACC_IDX
    data[IDX:IDX+3,:] = -1.0 * data[IDX:IDX+3,:]

    # Base
    IDX = IMU_BASE_IDX + ACC_IDX
    data[IDX:IDX+3,:] = -1.0 * data[IDX:IDX+3,:]

    return data

# https://stackoverflow.com/questions/6518811/interpolate-nan-values-in-a-numpy-array
def nan_helper(y):
    """Helper to handle indices and logical indices of NaNs.

    Input:
        - y, 1d numpy array with possible NaNs
    Output:
        - nans, logical indices of NaNs
        - index, a function, with signature indices= index(logical_indices),
          to convert logical indices of NaNs to 'equivalent' indices
    Example:
        >>> # linear interpolation of NaNs
        >>> nans, x= nan_helper(y)
        >>> y[nans]= np.interp(x(nans), x(~nans), y[~nans])
    """
    return np.isnan(y), lambda z: z.nonzero()[0]

def load_data_from_file(file_name, use_calibration=False, interp_nan=True, use_legacy_sign_convention=False):
    '''
    Loads data from a file

    ----------
    Arguments
        file_name : str
            Name of the file to load. Example: 07052019_23_28_36.dat
        interp_nan : bool
            Interpolates nans if True
        use_baseline_calibration : bool
            If True, loads the baseline data (if it exists) and applies the
            transformations to the data just loaded
        use_legacy_sign_convention : bool
            If True, transforms the data set from the old acceleration sign
            convention to the new one. Meant for data sets recorded prior to
            July 2019
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
    idx_to_del = list()
    assert(len(too_small) == 0 or len(too_small) > 1), "Invalid number of lines are too small"
    if len(too_small) > 0:
        cur_line, cur_ind = too_small.pop(0)
    while len(too_small) > 0:
        line, ind = too_small.pop(0)
        if len(bin_data[cur_ind]) < LOGGED_BUF_SIZE:
            bin_data[cur_ind] = bin_data[cur_ind] + line
            idx_to_del.append(ind)
        else:
            cur_ind = ind
    for index in sorted(idx_to_del, reverse=True):
        del bin_data[index]
    
    num_samples = len(bin_data)
    
    # Interpret the binary data as floats
    imu_data = np.ndarray(shape=(int(IMU_BUF_SIZE / 4), num_samples))
    time_stamps = list()
    for i in range(num_samples):
        imu_data[:,i] = decode_data(bin_data[i][20:-1])
        if sys.version_info > (3, 0):
            # Python 3
            time_stamps.append(datetime.strptime(str(bin_data[i][0:12])[2:-1], '%H:%M:%S.%f'))
        else:
            # Python 2
            time_stamps.append(datetime.strptime(str(bin_data[i][0:12]), '%H:%M:%S.%f'))
    
    # Interpolate NaN, if requested and needed
    if interp_nan:
        num_base_nans = np.isnan(imu_data[IMU_BASE_IDX,:]).sum()
        num_lamp_nans = np.isnan(imu_data[IMU_LAMP_IDX,:]).sum()
        if num_base_nans + num_lamp_nans > 0:
            logString("Interpolating NaNs (Base: {0}|Lamp: {1})".format(
                num_base_nans,num_lamp_nans)
            )
            for i in range(imu_data.shape[0]):
                nans, idx = nan_helper(imu_data[i,:])
                imu_data[i,nans]= np.interp(idx(nans), idx(~nans), imu_data[i,~nans])
    
    # Apply transformations if requested
    if use_calibration:
        imu_data = apply_baseline_transformations(imu_data, use_legacy_sign_convention)
    elif use_legacy_sign_convention:
        imu_data = apply_legacy_sign_convention(imu_data)
    
    return imu_data, num_samples, time_stamps

def make_time_series(imu_data, num_samples, time_stamps, use_time_stamps):
    '''
    Constructs a time series from the given IMU data. Can construct based on
    true time (time stamps) or sampling rate * number of samples. Use true time
    for the best accuracy if you trust your computer's clock!

    Note: as long as your computer can measure time *intervals* accurately,
        the time stamps are the most robust way to construct the time series

    ----------
    Arguments
        imu_data : np.ndarray
            Array of IMU data
        num_samples : int
            number of samples loaded from recording file
        time_stamps : list of datetime
            Time stamp for each IMU data sample
        use_time_stamps : bool
            If True, constructs time series based on time stamps. Interpolates
            missing data as needed. Otherwise, creates time series of size
            num_samples
    
    Returns:
        t : np.ndarray
            Array of time indexes for IMU data
        imu_data : np.ndarray
            Updated IMU data (e.g. may contain interpolated points)
        
    '''
    SAMPLE_RATE = 100.0 # Hz
    if not use_time_stamps:
        # Use sample rate as source of truth for timing info
        t = np.linspace(0, num_samples / SAMPLE_RATE, num=num_samples, endpoint=False)
    else:
        # Use time stamps as source of truth for timing info (should be more
        # reliable)
        ts = time_stamps[0]
        tf = time_stamps[-1]
        delta_t = tf - ts # Total elapsed time
        num_time_slots = int( \
            np.ceil( \
                SAMPLE_RATE * delta_t.seconds + \
                delta_t.microseconds / (1000.0 * (1000.0 / SAMPLE_RATE)) \
            ) + 1 \
        )
        t = np.linspace(0, num_time_slots, num=num_time_slots, endpoint=False)
        # Now we need to make a new array to hold IMU info. This array will
        # contain perfect 10 ms spacing between samples. The IMU info will need
        # to be copied into the closest unoccupied slot, and then we'll need to
        # interpolate over missing slots
        imu_data_ts = np.empty(shape=(int(IMU_BUF_SIZE / 4), t.shape[0]))
        imu_data_ts.fill(np.nan)
        for i in range(imu_data.shape[1]):
            dt = time_stamps[i] - ts
            idx = int(
                np.round( \
                    SAMPLE_RATE * dt.seconds + \
                    dt.microseconds / (1000.0 * (1000.0 / SAMPLE_RATE)) \
                ) \
            )
            imu_data_ts[:,idx] = imu_data[:,i]
        # Interpolate missing points (identified as NaN), if they exist
        for i in range(imu_data_ts.shape[0]):
            nans, idx = nan_helper(imu_data_ts[i,:])
            imu_data_ts[i,nans] = np.interp(idx(nans), idx(~nans), imu_data_ts[i,~nans])
        # Print out number of interpolated points
        logString("Interpolated {0} points in time series".format( \
            len([n for n in nans if n == True])) \
        )
        # Update previously-assigned variables that will be used (generically)
        # below
        imu_data = imu_data_ts
        num_samples = num_time_slots
    return t, imu_data, num_samples

class WaitForMs:
    '''
    Delays using time.sleep() have a tendency to overshoot the target wait time.
    This class uses feedback to adjust the wait time to achieve the target.
    '''
    def __init__(self, ms):
        self._t_ref = ms / 1000.0
        self._t_err = 0
        self._e_gain = 0.0
        self._e_lim_high = 0.0
        self._e_lim_low = 0.0
        self._debug = False
    
    def set_e_gain(self, e):
        '''
        Sets the error amplification factor.
        '''
        self._e_gain = e
    
    def set_e_lim(self, high, low):
        '''
        Sets the max and min error bounds. Arguments in ms.
        '''
        self._e_lim_high = high / 1000.0
        self._e_lim_low = low / 1000.0
        
    def _limit(self, err):
        '''
        Bounds the error feedback, if necessary, so that
        _e_lim_high >= err >= _e_lim_low.
        '''
        if err > self._e_lim_high:
            return self._e_lim_high
        elif err < self._e_lim_low:
           return self._e_lim_low
        else:
            return err

    def wait(self):
        '''
        Wait with feedback and limiting. Behaves like time.sleep if the gain and
        limits are not set.
        '''
        ts = time.time()
        
        # Adjust wait time based on previous error
        t_wait = self._t_ref + self._t_err
        time.sleep(t_wait)
        
        # Update error term
        t_waited = time.time() - ts
        err = self._t_ref - t_waited
        self._t_err = self._t_err + self._e_gain*err
        self._t_err = self._limit(err)
            
        if self._debug:
            print(
                np.round(t_wait*1000,4), np.round(t_waited*1000,4),
                np.round(err*1000,4), np.round(self._t_err*1000,4)
            )