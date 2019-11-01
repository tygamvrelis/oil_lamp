
import os
import cv2
import numpy as np
from cv2 import VideoWriter, VideoWriter_fourcc
from util import *

class Animate:
    def __init__(self, t, angles, FPS=60, width=640, height=480):
        self.__t = t
        self.__angles = angles
        self.FPS = FPS
        self.width = width
        self.height = height
        # Hardcoded video & pendulum settings
        self.mid_x = width // 2
        self.mid_y = height // 2
        self.radius = 25 # Size of the circle at end of pendulum
        SCALE_FACTOR = 400
        self.L = 0.52 * SCALE_FACTOR # Make it look large in the frame
        self.TS = 1 / get_sample_rate()

    def do_animate(self, angle_idx):
        '''
        Creates an animation based on the given angle data and time samples

        Arguments
        --------
            angle_idx : int
                Specifies which angles are to be animated (e.g. BASE_OUTER, )
        '''
        fname = './pendulum'
        if angle_idx == BASE_OUTER:
            fname += '_base_outer'
        elif angle_idx == BASE_INNER:
            fname += '_base_inner'
        elif angle_idx == LAMP_OUTER:
            fname += '_lamp_outer'
        elif angle_idx == LAMP_INNER:
            fname += '_lamp_inner'
        fname += '.avi'
        fname = os.path.join(get_data_dir(), fname) # TODO: fix this so it goes to right subdir
        fourcc = VideoWriter_fourcc(*'MP42')
        video = VideoWriter(fname, fourcc, float(self.FPS), (self.width, self.height))
        for i in range(0, self.__t.shape[0], int(1 / (self.TS * self.FPS))):
            angle = self.__angles[angle_idx, i] * np.pi / 180.0
            circ_x = self.mid_x + int(self.L * np.sin(angle));
            circ_y = self.mid_y // 2 + int(self.L * np.cos(angle));

            frame = np.full((self.height, self.width, 3), 65535, dtype=np.uint8) # Fill white
            cv2.line(
                frame,
                (self.mid_x, self.mid_y // 2),
                (circ_x, circ_y),
                (100,100,100),
                thickness=10,
                lineType=cv2.LINE_AA
            )
            cv2.circle(
                frame,
                (circ_x, circ_y),
                self.radius, (0, 0, 0),
                -1, 
                lineType=cv2.LINE_AA
            )
            video.write(frame)
        video.release()

    def do_birds_eye_view_animation(self):
        '''
        Project motion onto a 2D plane below the lamp
        '''
        pass
    
    def do_polar_animation(self, imu):
        '''
        Animate the value of theta1 and theta2 vs time, as a vector on a 2D
        plane
        '''
        # TODO:tyler
        # Also, what use cases do we want to support anyway? Individual IMUs?
        # combined angles? Just one of these? Both?
        assert(imu == 'base' or imu == 'lamp' or imu == 'both'), "Invalid imu value"
        fname = './polar_plot' + '_' + imu + '.avi'
        fname = os.path.join(get_data_dir(), fname) # TODO: fix this so it goes to right subdir
        fourcc = VideoWriter_fourcc(*'MP42')
        video = VideoWriter(fname, fourcc, float(self.FPS), (self.width, self.height))
        MAX_ANGLE = 40.0
        SCALE_FACTOR = min(self.width, self.height) / 2.0 / MAX_ANGLE
        for i in range(0, self.__t.shape[0], int(1 / (self.TS * self.FPS))):
            frame = np.full((self.height, self.width, 3), 65535, dtype=np.uint8) # Fill white
            if imu == 'base' or imu == 'both':
                angle_outer = self.__angles[BASE_OUTER, i] * SCALE_FACTOR
                angle_inner = self.__angles[BASE_INNER, i] * SCALE_FACTOR
                angle_outer = int(angle_outer)
                angle_inner = int(angle_inner)
                cv2.arrowedLine(
                    frame,
                    (self.mid_x, self.mid_y),
                    (angle_outer + self.mid_x, angle_inner + self.mid_y),
                    (100,0,255),
                    thickness=3,
                    line_type=cv2.LINE_AA
                )
            if imu == 'lamp' or imu == 'both':
                angle_outer = self.__angles[LAMP_OUTER, i] * SCALE_FACTOR
                angle_inner = self.__angles[LAMP_INNER, i] * SCALE_FACTOR
                angle_outer = int(angle_outer)
                angle_inner = int(angle_inner)
                cv2.arrowedLine(
                    frame,
                    (self.mid_x, self.mid_y),
                    (angle_outer + self.mid_x, angle_inner + self.mid_y),
                    (100,100,100),
                    thickness=3,
                    line_type=cv2.LINE_AA
                )
            video.write(frame)
        video.release()

def generate_standalone_pendulum_video():
    FPS=60
    width=640
    height=480

    radius = 25
    SCALE_FACTOR = 400
    L = 0.52 * SCALE_FACTOR # Make it look large in the frame
    T = 2
    A = 30 * np.pi / 180
    TMAX = 10
    TS = 0.01

    offset_x = width // 2
    offset_y = height // 4

    fourcc = VideoWriter_fourcc(*'MP42')
    video = VideoWriter('./pendulum.avi', fourcc, float(FPS), (width, height))

    for t in range(0, int(10 / TS), int(1 / (TS * FPS))):
        frame = np.full((height, width, 3), 65535, dtype=np.uint8) # Fill white
        
        angle = A * np.sin(2 * np.pi * t * TS / T)
        circ_x = offset_x + int(L * np.sin(angle));
        circ_y = offset_y + int(L * np.cos(angle));
        cv2.line(frame, (offset_x, offset_y), (circ_x, circ_y), (100,100,100), 10)
        cv2.circle(frame, (circ_x, circ_y), radius, (0, 0, 0), -1)
        video.write(frame)

    video.release()