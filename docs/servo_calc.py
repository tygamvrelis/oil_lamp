# Runs calculations for servo requirements, under the following assumptions
#
# - A good model for the lamp is a pendulum of length 6.8 [cm] with mass
#   0.831 [kg]. In reality, the center of mass is probably less than 6.8 [cm]
#   away from the pivot point, but better to overestimate than underestimate
# - The maximum deflection the lamp will experience is 40 degrees
# - The maximum (angular) speed the lamp will experience is that reached by
#   free-falling from 40 degrees under the influence of gravity
#
# See http://www.endmemo.com/physics/spendulum.php for help with pendulums
#
# Author: Tyler
# Date: June 25, 2019

import numpy as np
import scipy.constants as sci

def pendulum_force(m, theta):
    '''
    Calculates force due to gravity for a pendulum, in N
    
    Arguments
    --------
        m : double
            Mass in kg
        theta : double
            Angle of deflection in degrees
    '''
    return -1.0 * m * sci.g * np.sin(np.pi * theta / 180.0)

def pendulum_torque(r, m, theta):
    '''
    Calculates torque due to gravity for a pendulum, in N*m
    
    Arguments
    --------
        r : double
            Length of arm in cm
        m : double
            Mass in kg
        theta : double
            Angle of deflection in degrees
    '''
    return pendulum_force(m, theta) * (r / 100.0)

def dynamic_torque(r, m, a):
    '''
    Calculates torque due to changes in velocity, in N*m
    
    Arguments
    --------
        r : double
            Length of arm in cm
        m : double
            Mass in kg
        a : double
            Acceleration in m/s
    '''
    return (r / 100.0) * m * a

def torque_nm_to_kgcm(nm):
    '''
    Converts torque from N*m to kg*cm
    '''
    return nm * 100.0 / sci.g;

def torque_kgcm_to_nm(kgcm):
    '''
    Converts torque from kg*cm to N*m
    '''
    return nm * sci.g / 100.0;

def compute_spec(arm_len, mass, max_angle, fos, name):
    '''
    Computes servos specs and prints the key parameters
    
    Arguments
    --------
        arm_len : double
            Length of arm in cm
        mass : double
            Mass in kg
        max_angle : double
            Max angle of deflection in degrees
        fos : double
            Factor of safety to use in torque calculations
        name : string
            Name of the spec (e.g. "Outer servo")
    '''
    # Mass-independent pendulum constants:
    # 1. Pendulum period [s]
    period = 2 * np.pi * np.sqrt((arm_len / 100.0) / sci.g)
    # 2. Maximum linear velocty in [m/s]
    v_max = np.sqrt(
        2.0 * sci.g * (arm_len / 100.0) * (1.0 - np.cos(np.pi * max_angle / 180.0))
    )
    # 3. Same as above but in [deg/s]
    #    w = v/r = 2*pi*f -> f = (v/r)/(2*pi)
    v_max_angular = (v_max / (arm_len / 100.0)) * 180.0 / (2 * np.pi)
    
    T_gravity = np.abs(pendulum_torque(arm_len, mass, max_angle))
    T_dynamic = np.abs(dynamic_torque(arm_len, mass, v_max / (period / 2)))
    T_tot = T_gravity + T_dynamic
    T_fos = T_tot * fos
    T_tot = np.round(T_tot, 3)
    T_fos = np.round(T_fos, 3)
    
    P = np.abs(pendulum_force(mass, max_angle) * v_max)
    P_losses = 1 # Estimate power loss in Watts due to friction etc.
    P = P + P_losses
    
    print(name)
    print("\tRaw torque: " + str(T_tot) + " [N*m] " +
          "(" + str(np.round(torque_nm_to_kgcm(T_tot), 3)) + " [kg*cm])"
    )
    print("\tTorque with {0}x factor of safety: ".format(fos) + str(T_fos) + " [N*m] " +
          "(" + str(np.round(torque_nm_to_kgcm(T_fos), 3)) + " [kg*cm])"
    )
    print("\tRange of motion: at least {0} degrees".format(int(max_angle * 2)))
    print("\tMax speed: {0} deg/s".format(np.round(v_max_angular, 3)))
    print("\tPower: {0} [W]".format(np.round(P, 3)))
    print("")


if __name__ == "__main__":
    m_lamp = 0.854  # Lamp mass [kg]
    m_outer = 0.345 # Additional mass for outer servo
    m_inner = 0.068 # Additional mass for inner servo
    l_arm = 6.8     # Length of arm in [cm]
    deg_max_deflection = 40.0 # Maximum deflection angle in degrees
    FOS = 6                   # Factor of safety
    
    # Calculate specs for outer servo
    m_tot_outer = m_lamp + m_outer
    compute_spec(l_arm, m_tot_outer, deg_max_deflection, FOS, "Outer servo")
    
    # Calculate specs for inner servo
    m_tot_inner = m_lamp + m_inner
    compute_spec(l_arm, m_tot_inner, deg_max_deflection, FOS, "Inner servo")