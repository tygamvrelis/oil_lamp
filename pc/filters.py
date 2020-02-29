# Filtering classes
# Author: Tyler
# Date: January 11, 2019

import numpy as np
from scipy.signal import butter, lfilter, freqz
from util import Z_IDX, Y_IDX, X_IDX

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

class lpf:
    ''' Low-pass filter '''
    def __init__(self, cutoff, fs, order=5):
        '''
        https://stackoverflow.com/questions/25191620/creating-lowpass-filter-in-scipy-understanding-methods-and-units
        '''
        nyq = 0.5 * fs
        normal_cutoff = cutoff / nyq
        self.__b, self.__a = butter( \
            order, normal_cutoff, btype='low', analog=False \
        )

    def process(self, data):
        '''
        Low-pass filters the given data
        '''
        y = lfilter(self.__b, self.__a, data)
        return y

def smooth(x,window_len=11,window='hanning'):
    '''
    Signal smoothing utility function
    https://stackoverflow.com/questions/5515720/python-smooth-time-series-data
    '''
    if x.ndim != 1:
            raise ValueError("smooth only accepts 1 dimension arrays.")
    if x.size < window_len:
            raise ValueError("Input vector needs to be bigger than window size.")
    if window_len<3:
            return x
    if not window in ['flat', 'hanning', 'hamming', 'bartlett', 'blackman']:
            raise ValueError("Window is on of 'flat', 'hanning', 'hamming', 'bartlett', 'blackman'")
    s=np.r_[2*x[0]-x[window_len-1::-1],x,2*x[-1]-x[-1:-window_len:-1]]
    if window == 'flat': #moving average
            w=np.ones(window_len,'d')
    else:  
            w=eval('np.'+window+'(window_len)')
    y=np.convolve(w/w.sum(),s,mode='same')
    return y[window_len:-window_len+1]