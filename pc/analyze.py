# Plotting and analysis utilities
# Author: Tyler
# Data: May 12, 2019

import numpy as np
import glob
import matplotlib.pyplot as plt
from util import *

class cFilt:
    ''' Complementary filter '''
    def __init__(self, alpha_p, alpha_r, theta_p=0, theta_r=0):
        '''
        Initializes the filter. Alpha is the factor that weighs the velocity
        integral term, and 1-alpha is the factor that weighs the acceleration
        term. Generally, large alpha is desired so that the velocity term is
        primarily used while the acceleration term corrects drift.
        --------
        Arguments
            alpha_p : double
                Pitch weighting
            alpha_r : double
                Roll weighting
            theta_p : double
                Initial pitch
            theta_r : double
                Initial roll
        '''
        self.__alpha_p = alpha_p
        self.__theta_p = theta_p
        self.__alpha_r = alpha_r
        self.__theta_r = theta_r
    
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
        # Outer gimbal angle (pitch)
        a_term = (1.0 - self.__alpha_p) * np.arctan2(a[X_IDX], a[Z_IDX]) * 180.0 / np.pi
        v_term = self.__alpha_p * (self.__theta_p + v[Y_IDX] * (dt / 1000.0))
        self.__theta_p = v_term + a_term
        
        # Inner gimbal angle (roll)
        a_term = (1.0 - self.__alpha_r) * np.arctan2(a[Y_IDX], a[Z_IDX]) * 180.0 / np.pi
        v_term = self.__alpha_r * (self.__theta_r + v[X_IDX] * (dt / 1000.0))
        self.__theta_r = v_term + a_term
        
        return self.__theta_p, self.__theta_r
        
    def theta(self):
        '''
        Gets the pitch and roll angles
        '''
        return self.__theta_p, self.__theta_r


def analyze(fname, imu_to_plot, estimate):
    make_data_dir()
    if fname == "latest":
        glob_str = os.path.join(get_data_dir(), '*.dat')
        files = glob.glob(glob_str)
        fname = max(files, key=os.path.getctime)

    SAMPLE_RATE = 100.0 # Hz
    imu_data, num_samples = load_data_from_file(fname)
    t = np.linspace(0, num_samples / SAMPLE_RATE, num=num_samples, endpoint=False)

    fig, ax = plt.subplots()
    size = 2
    if estimate == "none":
        if imu_to_plot == "base" or imu_to_plot == "both":
            ax.scatter(t, imu_data[IMU_BASE_IDX + ACC_IDX + X_IDX],  c="blue",
                label="Base Ax", s=size)
            ax.scatter(t, imu_data[IMU_BASE_IDX + ACC_IDX + Y_IDX],  c="red",
                label="Base Ay", s=size)
            ax.scatter(t, imu_data[IMU_BASE_IDX + ACC_IDX + Z_IDX],  c="green",
                label="Base Az", s=size)
            ax.scatter(t, imu_data[IMU_BASE_IDX + GYRO_IDX + X_IDX], c="gold",
                label="Base Vx", s=size)
            ax.scatter(t, imu_data[IMU_BASE_IDX + GYRO_IDX + Y_IDX], c="black",
                label="Base Vy", s=size)
            ax.scatter(t, imu_data[IMU_BASE_IDX + GYRO_IDX + Z_IDX], c="magenta",
                label="Base Vz", s=size)
        if imu_to_plot == "lamp" or imu_to_plot == "both":
            ax.scatter(t, imu_data[IMU_LAMP_IDX + ACC_IDX + X_IDX],  c="blue",
                label="Lamp Ax", s=size)
            ax.scatter(t, imu_data[IMU_LAMP_IDX + ACC_IDX + Y_IDX],  c="red",
                label="Lamp Ay", s=size)
            ax.scatter(t, imu_data[IMU_LAMP_IDX + ACC_IDX + Z_IDX],  c="green",
                label="Lamp Az", s=size)
            ax.scatter(t, imu_data[IMU_LAMP_IDX + GYRO_IDX + X_IDX], c="gold",
                label="Lamp Vx", s=size)
            ax.scatter(t, imu_data[IMU_LAMP_IDX + GYRO_IDX + Y_IDX], c="black",
                label="Lamp Vy", s=size)
            ax.scatter(t, imu_data[IMU_LAMP_IDX + GYRO_IDX + Z_IDX], c="magenta",
                label="Lamp Vz", s=size)
        ax.legend()
        
        plt.title('Raw data vs time')
        plt.xlabel('Time (s)')
        plt.ylabel('Raw data ($m/s^2$ and $^\circ$)')
        
        fig_name = "raw_" + imu_to_plot + "_"
        fig_name = fig_name + os.path.splitext(os.path.basename(fname))[0]
        fig_name = fig_name + '.png'
        fig_name = os.path.join(get_data_dir(), fig_name)
        print(fig_name)
        plt.savefig(fig_name)
        logString("Saved fig to {0}".format(fig_name))
        plt.close();
    else:
        base_filt = cFilt(0.70, 0.70)
        lamp_filt = cFilt(0.70, 0.70)
        
        OUTER = 0
        INNER = 1
        BASE_OUTER = 0
        BASE_INNER = 1
        LAMP_OUTER = 2
        LAMP_INNER = 3
        angles = np.ndarray(shape=(4, num_samples))
        for i in range(num_samples):
            angles[BASE_OUTER:BASE_INNER+1,i] = base_filt.update(
                imu_data[IMU_BASE_IDX + GYRO_IDX:,i],
                imu_data[IMU_BASE_IDX + ACC_IDX:,i],
                10
            )
            angles[LAMP_OUTER:LAMP_INNER+1,i] = lamp_filt.update(
                imu_data[IMU_LAMP_IDX + GYRO_IDX:,i],
                imu_data[IMU_LAMP_IDX + ACC_IDX:,i],
                10
            )

        if estimate == "ind_angles":
            # Plot pitch and roll separately for each IMU
            if imu_to_plot == "base" or imu_to_plot == "both":
                ax.scatter(t, angles[BASE_OUTER], c="blue",  label="Base (outer)", s=size)
                ax.scatter(t, angles[BASE_INNER], c="red",   label="Base (inner)", s=size)
            if imu_to_plot == "lamp" or imu_to_plot == "both":
                ax.scatter(t, angles[LAMP_OUTER], c="green", label="Lamp (outer)", s=size)
                ax.scatter(t, angles[LAMP_INNER], c="gold",  label="Lamp (inner)", s=size)
            fig_name = estimate + "_" + imu_to_plot + "_"
        else:
            # Combine the pitch and roll from each IMU into a single value for each
            angles[OUTER:INNER+1,:] = angles[BASE_OUTER:BASE_INNER+1,:] + angles[LAMP_OUTER:LAMP_INNER+1,:]
            ax.scatter(t, angles[OUTER], c="blue",  label="Outer gimbal", s=size)
            ax.scatter(t, angles[INNER], c="red",   label="Inner gimbal", s=size)
            fig_name = estimate + "_"
        ax.legend()
            
        plt.title('Angles vs time')
        plt.xlabel('Time (s)')
        plt.ylabel('Angle ($^\circ$)')

        fig_name = fig_name + os.path.splitext(os.path.basename(fname))[0]
        fig_name = fig_name + '.png'
        fig_name = os.path.join(get_data_dir(), fig_name)
        plt.savefig(fig_name)
        logString("Saved fig to {0}".format(fig_name))
        plt.close();