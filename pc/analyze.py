# Plotting and analysis utilities
# Author: Tyler
# Data: May 12, 2019

import numpy as np
import glob
from util import *
import animate as anim
try:
    import matplotlib.pyplot as plt
except:
    # Raspberry Pi doesn't have matplotlib
    logString("Failed to import matplotlib.pyplot")

class cFilt:
    ''' Complementary filter '''
    def __init__(self, alpha_p, alpha_r, theta_p=0, theta_r=0, verbose=False):
        '''
        Initializes the filter. Alpha is the factor that weighs the velocity
        integral term, and 1-alpha is the factor that weighs the acceleration
        term. Generally, large alpha is desired so that the velocity term is
        primarily used while the acceleration term corrects drift.
        --------
        Arguments
            alpha_p : double
                Pitch (outer gimbal) weighting
            alpha_r : double
                Roll (inner gimbal) weighting
            theta_p : double
                Initial pitch
            theta_r : double
                Initial roll
            verbose : bool
                Prints debug messages if True
        '''
        self.__alpha_p = alpha_p
        self.__theta_p = theta_p
        self.__alpha_r = alpha_r
        self.__theta_r = theta_r
        self.__verbose = verbose
    
    def update(self, v, a, dt):
        '''
        Updates the angle estimates.
        --------
        Arguments
            v : np.ndarray
                A 3-vector containing vz, vy, vx
            a : np.ndarray
                A 3-vector containing az, ay, ax
            dt : double
                Sampling period in milliseconds
        '''
        if self.__verbose:
            print(v[0:3], a[0:3])

        # Outer gimbal angle (pitch)
        a_term = (1.0 - self.__alpha_p) * np.arctan2(a[X_IDX], -a[Z_IDX]) * 180.0 / np.pi
        v_term = self.__alpha_p * (self.__theta_p + v[Y_IDX] * (dt / 1000.0))
        self.__theta_p = v_term + a_term
        
        # Inner gimbal angle (roll)
        a_term = (1.0 - self.__alpha_r) * np.arctan2(-a[Y_IDX], -a[Z_IDX]) * 180.0 / np.pi
        v_term = self.__alpha_r * (self.__theta_r + v[X_IDX] * (dt / 1000.0))
        self.__theta_r = v_term + a_term
        
        return self.__theta_p, self.__theta_r
        
    def theta(self):
        '''
        Gets the pitch and roll angles
        '''
        return self.__theta_p, self.__theta_r

def get_angles(raw_imu_data, num_samples):
    '''
    Computes a time series of angles given a time series of raw IMU data
    '''
    # tau = alpha*dt/(1-alpha)
    # so alpha = tau/(tau + dt)
    # Swing period is about 0.51 [s] and dt is 0.01 [s]. Use tau = 0.255 [s]
    alpha_p = 0.96226415094
    alpha_r = 0.96226415094
    base_filt = cFilt(alpha_p, alpha_r)
    lamp_filt = cFilt(alpha_p, alpha_r)
    
    assert(raw_imu_data.shape[0] == 12), "Invalid IMU data size"

    angles = np.ndarray(shape=(4, num_samples))
    for i in range(num_samples):
        angles[BASE_OUTER:BASE_INNER+1,i] = base_filt.update(
            raw_imu_data[IMU_BASE_IDX + GYRO_IDX:,i],
            raw_imu_data[IMU_BASE_IDX + ACC_IDX:,i],
            10
        )
        angles[LAMP_OUTER:LAMP_INNER+1,i] = lamp_filt.update(
            raw_imu_data[IMU_LAMP_IDX + GYRO_IDX:,i],
            raw_imu_data[IMU_LAMP_IDX + ACC_IDX:,i],
            10
        )
    return angles

def analyze(fname, imu_to_plot, estimate, use_calibration, \
    use_legacy_sign_convention, use_time_stamps, plot_slice, \
    make_wav, anim_data):
    '''
    Visualizes logged data
    --------
    Arguments:
        fname : str
            Name of log file whose data is to be analyzed
        imu_to_plot : str
            Indicates whether the data for the base IMU or the lamp IMU is to
            be analzed, or both
        estimate : bool
            Estimates angles if True, otherwise plots raw data
        use_calibration : bool
            If True, applies rotations and offsets to the raw data, based on the
            contents of the .ini files. This can be used to account for the fact
            that the IMUs are mounted at angles relative to the lamp and base
        use_legacy_sign_convention : bool
            If True, transforms the data set from the old acceleration sign
            convention to the new one. Meant for data sets recorded on MCU
            firmware older than July 2019
        use_time_stamps : bool
            If True, uses the time stamps in the data log to construct the time
            series. Assuming the clock on the computer doing the recording is
            trustworthy, this should be very robust. Missing entries are
            accounted for. Otherwise, the time series is constructed based on
            sampling rate * number of samples, and has no connection to "true
            time"
        plot_slice : string
            String containing start time and end time to plot between
        make_wav : bool
            Indicates whether to make .wav file from angle data
        anim_data : tuple
            3-tuple containing (1) bool indicating whether or not to animate,
            (2) animation type, and (3) animation arguments
    '''
    make_data_dir()
    if fname == "latest":
        glob_str = os.path.join(get_data_dir(), '*.dat')
        files = glob.glob(glob_str)
        fname = max(files, key=os.path.getctime)

    imu_data, num_samples, time_stamps = load_data_from_file( \
        fname, \
        use_calibration=use_calibration, \
        use_legacy_sign_convention=use_legacy_sign_convention \
    )
    t, imu_data, num_samples = make_time_series( \
        imu_data, \
        num_samples, \
        time_stamps, \
        use_time_stamps \
    )
    angles = get_angles(imu_data, num_samples)

    if make_wav:
        # Combine the pitch and roll from each IMU into a single value for each
        angles[OUTER:INNER+1,:] = angles[BASE_OUTER:BASE_INNER+1,:] + \
                                  angles[LAMP_OUTER:LAMP_INNER+1,:]
        fname_base = os.path.splitext(fname)[0]
        fname = os.path.join(get_data_dir(), fname_base)
        write_wave(angles[OUTER,:], fname + "_outer")
        write_wave(angles[INNER,:], fname + "_inner")
        return

    if plot_slice:
        t_start, t_end = plot_slice.split(',')
        t_start = np.round(float(t_start), 2)
        t_end = np.round(float(t_end), 2)
        # Bounds check!
        if t_end > t[-1]:
            logString( \
                "--plot slice end time exceeds end time of data ({0})".format(t[-1]) \
            )
            quit()
        start_idx = int(np.round(t_start * (num_samples / (t[-1] - t[0]))))
        end_idx = int(np.round(t_end * (num_samples / (t[-1] - t[0]))))
        # Slice!
        t = t[start_idx:end_idx+1]
        imu_data = imu_data[:,start_idx:end_idx+1]
        angles = angles[:,start_idx:end_idx+1]
        num_samples = end_idx - start_idx + 1
    
    do_animate, anim_type, anim_args = anim_data
    if do_animate:
        aa = anim.Animate(t, angles, fname)
        if anim_type == 'phase':
            aa.do_phase_space_animation(imu_to_plot)
        elif anim_type == 'top_down':
            do_decomp = anim_args == 'decomp'
            aa.do_birds_eye_view_animation(imu_to_plot, do_decomp)
        elif anim_type == 'pendulum':
            if imu_to_plot == 'lamp' or imu_to_plot == 'both':
                if anim_args == 'outer' or anim_args == 'both':
                    aa.do_pendulum_animation(LAMP_OUTER)
                if anim_args == 'inner' or anim_args == 'both':
                    aa.do_pendulum_animation(LAMP_INNER)
            if imu_to_plot == 'base' or imu_to_plot == 'both':
                if anim_args == 'outer' or anim_args == 'both':
                    aa.do_pendulum_animation(BASE_OUTER)
                if anim_args == 'inner' or anim_args == 'both':
                    aa.do_pendulum_animation(BASE_INNER)
        else:
            assert(False), "Animation processing messed up!"
        return

    fig, ax = plt.subplots()
    size = 2
    if estimate == "none":
        if imu_to_plot == "base" or imu_to_plot == "both":
            ax.plot(t, imu_data[IMU_BASE_IDX + ACC_IDX + X_IDX],  c="blue",
                label="Base Ax")
            ax.plot(t, imu_data[IMU_BASE_IDX + ACC_IDX + Y_IDX],  c="red",
                label="Base Ay")
            ax.plot(t, imu_data[IMU_BASE_IDX + ACC_IDX + Z_IDX],  c="green",
                label="Base Az")
            ax.plot(t, imu_data[IMU_BASE_IDX + GYRO_IDX + X_IDX], c="gold",
                label="Base Vx")
            ax.plot(t, imu_data[IMU_BASE_IDX + GYRO_IDX + Y_IDX], c="black",
                label="Base Vy")
            ax.plot(t, imu_data[IMU_BASE_IDX + GYRO_IDX + Z_IDX], c="magenta",
                label="Base Vz")
        if imu_to_plot == "lamp" or imu_to_plot == "both":
            ax.plot(t, imu_data[IMU_LAMP_IDX + ACC_IDX + X_IDX],  c="blue",
                label="Lamp Ax")
            ax.plot(t, imu_data[IMU_LAMP_IDX + ACC_IDX + Y_IDX],  c="red",
                label="Lamp Ay")
            ax.plot(t, imu_data[IMU_LAMP_IDX + ACC_IDX + Z_IDX],  c="green",
                label="Lamp Az")
            ax.plot(t, imu_data[IMU_LAMP_IDX + GYRO_IDX + X_IDX], c="gold",
                label="Lamp Vx")
            ax.plot(t, imu_data[IMU_LAMP_IDX + GYRO_IDX + Y_IDX], c="black",
                label="Lamp Vy")
            ax.plot(t, imu_data[IMU_LAMP_IDX + GYRO_IDX + Z_IDX], c="magenta",
                label="Lamp Vz")
        ax.legend()
        
        plt.title('Raw data vs time')
        plt.xlabel('Time (s)')
        plt.ylabel('Raw data ($m/s^2$ and $^\circ$)')
        
        fig_name = "raw_" + imu_to_plot + "_"
        fig_name += os.path.splitext(os.path.basename(fname))[0]
        if plot_slice:
            fig_name += "_from%.2fto%.2f" % (t_start, t_end)
        fig_name += '.png'
        fig_name = os.path.join(get_data_dir(), os.path.dirname(fname), fig_name)
        plt.savefig(fig_name)
        logString("Saved fig to {0}".format(fig_name))
        plt.close();
    else:
        if estimate == "ind_angles":
            # Plot pitch and roll separately for each IMU
            if imu_to_plot == "base" or imu_to_plot == "both":
                ax.plot(t, angles[BASE_OUTER], c="blue",  label="Base (outer)")
                ax.plot(t, angles[BASE_INNER], c="red",   label="Base (inner)")
            if imu_to_plot == "lamp" or imu_to_plot == "both":
                ax.plot(t, angles[LAMP_OUTER], c="green", label="Lamp (outer)")
                ax.plot(t, angles[LAMP_INNER], c="gold",  label="Lamp (inner)")
            fig_name = estimate + "_" + imu_to_plot + "_imu_"
        else:
            # Combine the pitch and roll from each IMU into a single value for each
            angles[OUTER:INNER+1,:] = angles[BASE_OUTER:BASE_INNER+1,:] + \
                                      angles[LAMP_OUTER:LAMP_INNER+1,:]
            ax.plot(t, angles[OUTER], c="blue",  label="Outer gimbal")
            ax.plot(t, angles[INNER], c="red",   label="Inner gimbal")
            fig_name = estimate + "_"
        ax.legend()
            
        plt.title('Angles vs time')
        plt.xlabel('Time (s)')
        plt.ylabel('Angle ($^\circ$)')

        fig_name += os.path.splitext(os.path.basename(fname))[0]
        if plot_slice:
            fig_name += "_from%.2fto%.2f" % (t_start, t_end)
        if use_calibration:
            fig_name += "_calibrated"
        fig_name += '.png'
        fig_name = os.path.join(get_data_dir(), os.path.dirname(fname), fig_name)
        plt.savefig(fig_name)
        logString("Saved fig to {0}".format(fig_name))
        plt.close();