# Controls servo actuation via logged or streamed data
# Author: Tyler
# Data: June 28, 2019

import time
import serial
import os
import struct
import numpy as np
import socket
import select
from threading import Thread, Event, Lock
from util import *
from analyze import get_angles, cFilt, make_complementary_filters

def transmit_angles(ser, a_outer, a_inner, dryrun=False):
    '''
    Sends angle data to the MCU
    
    Arguments
    --------
        port : serial.Serial
            COM port that MCU is connected to
        a_outer : float
            Outer gimbal angle
        a_inner : float
            Inner gimbal angle
        dryrun : bool
            Prints angles to shell if True, otherwise sends to MCU
    '''
    if dryrun:
        logString("Outer: {0}|Inner: {1}".format(a_outer, a_inner))
    else:
        cmd_id = CMD_ANGLE.encode() # A => Angle payload
        # It turns out that rounding error for ints is noticable, especially at
        # low playback frequencies (e.g. 10*sin(2*pi*0.1*t)) and for small angles. So
        # we're sticking with floats
        payload = struct.pack('<f', a_outer) + struct.pack('<f', a_inner)
        packet = cmd_id + payload
        ser.write(packet)
    return

def send_servo_angles(port, baud, angles):
    '''
    Sends a command to the microcontroller to set the servos to the specified
    angles.
    
    Arguments
    --------
        port : serial.Serial
            COM port that MCU is connected to
        baud : int
            Symbol rate over COM port
        angles : string
            string of the form "outer_gimbal_angle,inner_gimbal_angle",
            e.g. "-10,7". Arguments may be positive or negative.
    '''
    has_sep=False
    if "," in angles:
        has_sep=True

    if has_sep:
        a_outer, a_inner = angles.split(",")
        try:
            a_outer = float(a_outer)
        except ValueError:
            a_outer = 0
        try:
            a_inner = float(a_inner)
        except ValueError:
            a_inner = 0
    else:
        try:
            a_outer = float(angles)
        except ValueError:
            a_outer = 0
        a_inner = 0
    
    logString(list_ports())
    logString("Attempting connection to embedded")
    logString("\tPort: " + port)
    logString("\tBaud rate: " + str(baud))
    
    try:
        with serial.Serial(port, baud, timeout=0) as ser:
            enable_servos(ser)
            transmit_angles(ser, a_outer, a_inner)
        logString("Sent outer angle={0} and inner angle={1}".format(
            np.round(a_outer,2),  np.round(a_inner,2))
        )
    except serial.serialutil.SerialException as e:
        if(str(e).find("FileNotFoundError")):
            logString("Port not found")
        else:
            logString("Serial exception")

def change_servo_usage(port, baud, use_servos):
    '''
    Sends a command to the MCU to enable or disable servo usage
    
    Arguments
    --------
        port : serial.Serial
            COM port that MCU is connected to
        baud : int
            Symbol rate over COM port
        use_servos : bool
            Indicates whether to enable or disable servo usage
    '''
    logString(list_ports())
    logString("Attempting connection to embedded")
    logString("\tPort: " + port)
    logString("\tBaud rate: " + str(baud))
    
    try:
        with serial.Serial(port, baud, timeout=0) as ser:
            if use_servos:
                enable_servos(ser)
                logString("Sent command to enable servos")
            else:
                disable_servos(ser)
                logString("Sent command to disable servos")
    except serial.serialutil.SerialException as e:
        if(str(e).find("FileNotFoundError")):
            logString("Port not found")
        else:
            logString("Serial exception")

def change_imu_usage(port, baud, use_imus):
    '''
    Sends a command to the MCU to enable or disable IMU sensing
    
    Arguments
    --------
        port : serial.Serial
            COM port that MCU is connected to
        baud : int
            Symbol rate over COM port
        use_imus : bool
            Indicates whether to enable or disable IMU usage
    '''
    logString(list_ports())
    logString("Attempting connection to embedded")
    logString("\tPort: " + port)
    logString("\tBaud rate: " + str(baud))
    
    try:
        with serial.Serial(port, baud, timeout=0) as ser:
            if use_imus:
                enable_imus(ser)
                logString("Sent command to enable IMU sensing")
            else:
                disable_imus(ser)
                logString("Sent command to disable IMU sensing")
    except serial.serialutil.SerialException as e:
        if(str(e).find("FileNotFoundError")):
            logString("Port not found")
        else:
            logString("Serial exception")

def send_sine_wave(port, baud, params, servo, verbose):
    '''
    Sends a sine wave to the microcontroller for servo actuation
    
    output = amp * exp(-tau * t) * sin(2 * pi * f * t)
    
    Arguments
    --------
        port : serial.Serial
            COM port that MCU is connected to
        baud : int
            Symbol rate over COM port
        params : string
            String containing amplitude and frequency of sine wave
        servo : string
            Specifies which servo(s) to send the waveform to
        verbose : bool
            Prints additional messages if True
    '''
    f_default = 1.0
    amp_default = 40.0
    tau_default = 0
    
    has_sep=False
    if "," in params:
        has_sep=True

    if has_sep:
        # Ugly way of doing this, but it works...
        if len(params.split(",")) == 3:
            freq, amp, tau = params.split(",")
            try:
                tau = float(tau)
            except ValueError:
                tau = tau_default
        else:
            freq, amp = params.split(",")
            tau = tau_default
        try:
            amp = float(amp)
        except ValueError:
            amp = amp_default
        try:
            freq = float(freq)
        except ValueError:
            freq = f_default
    else:
        try:
            freq = float(params)
        except ValueError:
            freq = f_default
        amp = amp_default
        tau = tau_default

    logString(list_ports())
    logString("Attempting connection to embedded")
    logString("\tPort: " + port)
    logString("\tBaud rate: " + str(baud))
    
    assert(servo == 'outer' or servo == 'inner' or servo == 'both'), "Invalid servo argument"
    
    t_s = time.time()
    try:
        with serial.Serial(port, baud, timeout=0) as ser:
            enable_servos(ser)
            while True:
                t = time.time() - t_s
                val = amp * np.exp(-tau * t) * np.sin(2 * np.pi * freq * t)
                if servo == 'outer':
                    transmit_angles(ser, val, 0)
                elif servo == 'inner':
                    transmit_angles(ser, 0, val)
                else:
                    transmit_angles(ser, val, val)
                time.sleep(0.01)
                if verbose:
                    print(val)
    except serial.serialutil.SerialException as e:
        if(str(e).find("FileNotFoundError")):
            logString("Port not found")
        else:
            logString("Serial exception")

def playback(port, baud, fname, loop, use_legacy_sign_convention, \
    use_time_stamps, verbose):
    '''
    Initiates playback mode
    
    Arguments
    --------
        port : serial.Serial
            COM port that MCU is connected to
        baud : int
            Symbol rate over COM port
        fname : str
            Name of log file used to derive angles
        loop : bool
            Determines whether to quit or restart after sending all angles
        use_legacy_sign_convention : bool
            If True, transforms the data set from the old acceleration sign
            convention to the new one. Meant for data sets recorded prior to
            July 2019
        use_time_stamps : bool
            See analyze in analyze.py
        verbose : bool
            Prints additional messages if True
    '''
    fname = os.path.join(get_data_dir(), fname)
    logString("Loading data file " + fname)
    imu_data, num_samples, time_stamps = load_data_from_file( \
        fname, \
        use_calibration=True, \
        use_legacy_sign_convention=use_legacy_sign_convention
    )
    t, imu_data, num_samples = make_time_series( \
        imu_data, \
        num_samples, \
        time_stamps, \
        use_time_stamps \
    )
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
                enable_servos(ser)
                if loop:
                    logString("Looping is enabled...will send angles forever")
                else:
                    logString("Looping is disabled...will quit after sending all angles")
                while True:
                    for angle_vec in angles.T:
                        outer_angle = angle_vec[BASE_OUTER] + angle_vec[LAMP_OUTER]
                        inner_angle = angle_vec[BASE_INNER] + angle_vec[LAMP_INNER]
                        if verbose:
                            logString("Outer: {0}|Inner: {1}".format(outer_angle, inner_angle))
                        # Send angles and wait a few ms before sending again
                        transmit_angles(ser, outer_angle, inner_angle)
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

class NetworkedReceiver:
    '''
    Methods for receiving data over the internet, and safely sharing it between
    threads
    '''
    def __init__(self, dryrun):
        self.__imu_data = np.zeros(IMU_BUF_SIZE) + float('inf')
        self.__lock = Lock()
        self.__stop_event = Event()
        self.__last_seq_num = float('-inf')
        self.__dryrun = dryrun
        self.__cnt = 0

    def stop(self):
        '''
        Causes the thread to exit after the next receive iteration
        '''
        self.__stop_event.set()

    def __stop_requested(self):
        return self.__stop_event.is_set()

    def get_latest_imu_data(self):
        '''
        Returns the latest raw IMU data in a thread-safe manner
        '''
        if self.__lock.acquire(True):
            imu_data = self.__imu_data
            self.__lock.release()
        return imu_data

    def recv_loop(self, sock):
        '''
        Loop for receiving raw data over network
        '''
        while(1):
            if self.__stop_requested():
                break
            ready = select.select([sock], [], [], 1.0 / get_sample_rate())
            if ready[0]:
                packet, (clt_ip, clt_port) = sock.recvfrom(BUF_SIZE_INET)
                # We use a sequence number so that if any old packets arrive at
                # a later time (out of order) we know to discard them
                seq_num, imu_data = decode_data_inet(packet)
                self.__cnt += 1
                if self.__dryrun and (self.__cnt % 10 == 0):
                    logString(str(imu_data))
                    sock.sendto(str(self.__cnt).encode(), (clt_ip, clt_port))
                if seq_num > self.__last_seq_num and self.__lock.acquire(True):
                    self.__imu_data = imu_data
                    self.__lock.release()
                    self.__last_seq_num = seq_num


def playback_networked(port, baud, udp_port, verbose, dryrun):
    '''
    Initiates playback mode for data received over the internet
    
    Arguments
    --------
        port : serial.Serial
            COM port that MCU is connected to
        baud : int
            Symbol rate over COM port
        udp_port : int
            The UDP port that the program will listen to for data
        verbose : bool
            Prints additional messages if True
        dryrun : bool
            Prints received angles and bypasses MCU connection
    '''
    bind_ip = '' # All addresses
    bind_port = udp_port
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.setblocking(False)
        try:
            sock.bind((bind_ip, bind_port))
        except socket.error:
            logString(
                "Failed to bind to UDP port %d. Letting OS choose port." % udp_port
            )
            sock.bind((bind_ip, 0))
        ip, udp_port_str = sock.getsockname()
        logString("Receiver listening on {0}:{1} (UDP)".format(ip, udp_port_str))
        
        if not dryrun:
            logString(list_ports())
            logString("Attempting connection to embedded")
            logString("\tPort: " + port)
            logString("\tBaud rate: " + str(baud))
        else:
            logString("Dryrun -- bypassing connection to embedded")
        
        tx_cycle = WaitForMs(10) # 10 ms wait between angle transmissions
        tx_cycle.set_e_gain(1.5)
        tx_cycle.set_e_lim(0, -3.0) # Never wait longer, but allow down to 3 ms less

        base_filt, lamp_filt = make_complementary_filters()
        
        num_tries = 0
        nr = NetworkedReceiver(dryrun)
        recv_thread = Thread(
            name="NetworkReceiver", target=nr.recv_loop, args=(sock, )
        )
        recv_thread.start()
        try:
            while True:
                if not dryrun:
                    try:
                        with serial.Serial(port, baud, timeout=0) as ser:
                            logString("Connected")
                            enable_servos(ser)
                            angles = np.zeros(shape=(4, 1))
                            while True:
                                imu_data = nr.get_latest_imu_data()
                                angles[BASE_OUTER:BASE_INNER+1] = base_filt.update(
                                    imu_data[IMU_BASE_IDX + GYRO_IDX:],
                                    imu_data[IMU_BASE_IDX + ACC_IDX:],
                                    1000.0 / get_sample_rate()
                                )
                                angles[LAMP_OUTER:LAMP_INNER+1] = lamp_filt.update(
                                    imu_data[IMU_LAMP_IDX + GYRO_IDX:],
                                    imu_data[IMU_LAMP_IDX + ACC_IDX:],
                                    1000.0 / get_sample_rate()
                                )
                                outer_angle = angles[BASE_OUTER] + angles[LAMP_OUTER]
                                inner_angle = angles[BASE_INNER] + angles[LAMP_INNER]
                                if verbose:
                                    logString("Outer: {0}|Inner: {1}".format(outer_angle, inner_angle))
                                # Send angles and wait a few ms before sending again
                                transmit_angles(ser, outer_angle, inner_angle)
                                tx_cycle.wait()
                    except serial.serialutil.SerialException as e:
                        if(num_tries % 100 == 0):
                            if(str(e).find("FileNotFoundError")):
                                logString("Port not found. Retrying...(attempt {0})".format(num_tries))
                            else:
                                logString("Serial exception. Retrying...(attempt {0})".format(num_tries))
                        time.sleep(0.01)
                        num_tries = num_tries + 1
                else:
                    angles = np.zeros(shape=(4, 1))
                    while True:
                        continue # Todo: wait until we get a buffer of data (e.g. 1 msg for now)
                    while True:
                        imu_data = nr.get_latest_imu_data()
                        angles[BASE_OUTER:BASE_INNER+1] = base_filt.update(
                            imu_data[IMU_BASE_IDX + GYRO_IDX:],
                            imu_data[IMU_BASE_IDX + ACC_IDX:],
                            1000.0 / get_sample_rate()
                        )
                        angles[LAMP_OUTER:LAMP_INNER+1] = lamp_filt.update(
                            imu_data[IMU_LAMP_IDX + GYRO_IDX:],
                            imu_data[IMU_LAMP_IDX + ACC_IDX:],
                            1000.0 / get_sample_rate()
                        )
                        outer_angle = angles[BASE_OUTER] + angles[LAMP_OUTER]
                        inner_angle = angles[BASE_INNER] + angles[LAMP_INNER]
                        logString("Outer: {0}|Inner: {1}".format(outer_angle, inner_angle))
                        tx_cycle.wait()
        except KeyboardInterrupt as e:
            nr.stop()
            recv_thread.join()
            logString("Receive thread joined")
            raise KeyboardInterrupt