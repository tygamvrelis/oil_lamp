
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
        self.offset_x = width // 2
        self.offset_y = height // 4
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
        fname = os.join(get_data_dir(), fname)
        fourcc = VideoWriter_fourcc(*'MP42')
        video = VideoWriter(fname, fourcc, float(self.FPS), (self.width, self.height))
        for i in range(0, self.__t.shape[0], int(1 / (self.TS * self.FPS))):
            angle = self.__angles[angle_idx, i] * np.pi / 180.0
            circ_x = self.offset_x + int(self.L * np.sin(angle));
            circ_y = self.offset_y + int(self.L * np.cos(angle));

            frame = np.full((self.height, self.width, 3), 65535, dtype=np.uint8) # Fill white
            cv2.line(frame, (self.offset_x, self.offset_y), (circ_x, circ_y), (100,100,100), 10)
            cv2.circle(frame, (circ_x, circ_y), self.radius, (0, 0, 0), -1)
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