# Filtering classes
# Author: Tyler
# Date: January 11, 2019

import numpy as np

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

class SoftClipper:
    ''' Soft clipping filter. Given a sequence of input values, this system
        ensures that the output value does not exceed a specified threshold,
        while hopefully introducing little (perceptible) distortion in the
        overall shape of the signal '''
    def __init__(self, M, alpha=0.125):
        '''
        Initializes the filter
        --------
        Arguments
            M : float
                Maximum permitted value (output shall not exceed this)
            alpha : float
                The factor that weighs the current derivative value (1-alpha
                weighs the previous derivatives)
        '''
        self.__eps = 0
        self.__alpha = alpha
        self.__dtheta_dt = 0
        self.__x_prev = float('inf')
    
    def update(self, x)
        '''
        Produces a new output for a given input
        --------
        Arguments
            x : float
                Value inputted into system for soft clipping
        '''
        # Compute derivative approximation + update prev
        if self.__x_pre == float('inf'):
            self.__x_prev = x
        deriv = x - self.__x_prev # 1st order backward apprx
        self.__dtheta_dt = self.__alpha * deriv + (1 - self.__alpha) * self.__dtheta_dt
        self.__x_prev = x

        # Dynamically update height of transition region
        self.__eps = self.__dtheta_dt * self.__M / 4
        if self.__eps > (2 / 3) * self.__M:
            # Hard limit on how large the height of the transition region can
            # be. We don't want to start clipping too early, as this could
            # distort the shape of small-amplitude transients
            self.__eps = (2 / 3) * self.__M
        
        # Output
        mme = self.__M - self.__eps
        if x < mme:
            y = x
        else:
            dist = self.__M - x
            if dist <= 0:
                # Hard limiting, for weird cases
                y = M
            else:
                # Smooth limiting using sinusoidal mapping
                y = M - 0.5 * self.__eps * np.cos(mme * dist / np.pi)
        return y
        