
import os
import cv2
import numpy as np
from cv2 import VideoWriter, VideoWriter_fourcc
from util import *

font = cv2.FONT_HERSHEY_SIMPLEX
LAMP_COLOR = (100, 100, 100)
BASE_COLOR = (100, 0, 255)
BLACK = (0, 0, 0)
GREY = (211, 211, 211)
MAX_ANGLE = 40.0

class Animate:
    def __init__(self, t, angles, fname, FPS=50, width=640, height=480):
        self.__t = t
        self.__angles = angles
        self.FPS = FPS
        self.width = width
        self.height = height
        # Hardcoded video & pendulum settings
        self.mid_x = width // 2
        self.mid_y = height // 2
        self.L = 5.2 # Length of pendulum [cm]
        self.TS = 1 / get_sample_rate()
        self.base_name = os.path.join(
            get_data_dir(),
            os.path.dirname(fname)
        )
        self.fname = os.path.splitext(os.path.basename(fname))[0]

    def do_animate(self, angle_idx):
        '''
        Creates an animation based on the given angle data and time samples

        Arguments
        --------
            angle_idx : int
                Specifies which angles are to be animated (e.g. BASE_OUTER, )
        '''
        fname = 'pendulum'
        if angle_idx == BASE_OUTER:
            fname += '_base_outer'
        elif angle_idx == BASE_INNER:
            fname += '_base_inner'
        elif angle_idx == LAMP_OUTER:
            fname += '_lamp_outer'
        elif angle_idx == LAMP_INNER:
            fname += '_lamp_inner'
        fname += '.avi'
        fname = os.path.join(self.base_name, self.fname + '_' + fname)
        fourcc = VideoWriter_fourcc(*'MP42')
        video = VideoWriter(fname, fourcc, float(self.FPS), (self.width, self.height))
        self.L *= 400 # Make it look big in the frame
        radius = 25
        for i in range(0, self.__t.shape[0], int(1 / (self.TS * self.FPS))):
            angle = self.__angles[angle_idx, i] * np.pi / 180.0
            circ_x = self.mid_x + int(self.L * np.sin(angle));
            circ_y = self.mid_y // 2 + int(self.L * np.cos(angle));

            frame = np.full((self.height, self.width, 3), 65535, dtype=np.uint8) # Fill white
            cv2.putText(
                frame, 'Time: %d' % self.__t[i],
                (10, self.height - 30),
                font, 0.5, BLACK, 1, cv2.LINE_AA
            )
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
                radius, (0, 0, 0),
                -1, 
                lineType=cv2.LINE_AA
            )
            video.write(frame)
        video.release()

    def add_phase_space_axis_labels(self, frame, scale):
        '''
        Add some axis labels to frame
        '''
        # Add +/- labels for some points
        for i in range (-30, 40, 10):
            if i == 0:
                continue
            outer = int(i * scale) + self.mid_x
            inner = -1 * int(i * scale) + self.mid_y
            cv2.line(
                frame, (outer, 0), (outer, self.height),
                GREY, thickness=1, lineType=cv2.LINE_AA
            )
            cv2.line(
                frame, (0, inner), (self.width, inner),
                GREY, thickness=1, lineType=cv2.LINE_AA
            )
            cv2.putText(
                frame, str(i),
                (outer, self.mid_y + 15),
                font, 0.5, BLACK, 1, cv2.LINE_AA
            )
            cv2.putText(
                frame, str(i),
                (self.mid_x + 10, inner),
                font, 0.5, BLACK, 1, cv2.LINE_AA
            )

    def plot_axes(self, frame, xlab, ylab, xlab_offset, ylab_offset):
        '''
        Adds axis lines to frame
        '''
        # X-axis (outer)
        cv2.arrowedLine(
            frame, (0, self.mid_y), (int(self.width * 0.99), self.mid_y),
            BLACK, thickness=1, line_type=cv2.LINE_AA, tipLength=0.01
        )
        cv2.putText(
            frame, xlab, (self.width - xlab_offset, self.mid_y + 15),
            font, 0.5, BLACK, 1, cv2.LINE_AA
        )
        # Y-axis (inner)
        cv2.arrowedLine(
            frame,
            (self.mid_x, self.height), (self.mid_x, int(self.height * 0.01)),
            BLACK, thickness=1, line_type=cv2.LINE_AA, tipLength=0.01
        )
        cv2.putText(
            frame, ylab, (self.mid_x + 10, ylab_offset),
            font,  0.5, BLACK, 1, cv2.LINE_AA
        )
    
    def do_phase_space_animation(self, imu):
        '''
        Animate the value of theta1 and theta2 vs time, as a vector on a 2D
        plane
        '''
        assert(imu == 'base' or imu == 'lamp' or imu == 'both'), "Invalid imu value"
        fname = 'phase_space' + '_' + imu + '.avi'
        fname = os.path.join(self.base_name, self.fname + '_' + fname)
        fourcc = VideoWriter_fourcc(*'MP42')
        video = VideoWriter(fname, fourcc, float(self.FPS), (self.width, self.height))
        SCALE_FACTOR = min(self.width, self.height) / 2.0 / MAX_ANGLE
        for i in range(0, self.__t.shape[0], int(1 / (self.TS * self.FPS))):
            frame = np.full((self.height, self.width, 3), 65535, dtype=np.uint8) # Fill white
            self.add_phase_space_axis_labels(frame, SCALE_FACTOR)
            self.plot_axes(frame, "Outer (deg)", "Inner (deg)", 100, 30)
            cv2.putText(
                frame, 'Time: %d' % self.__t[i],
                (10, self.height - 30),
                font, 0.5, BLACK, 1, cv2.LINE_AA
            )
            # Color legend
            if imu == 'base' or imu == 'both':
                cv2.putText(
                    frame, 'Base', (10,30),
                    font, 1,  BASE_COLOR, 2, cv2.LINE_AA
                )
            if imu == 'lamp' or imu == 'both':
                if imu == 'both':
                    start_y = 60
                else:
                    start_y = 30
                cv2.putText(
                    frame, 'Lamp', (10,start_y),
                    font, 1, LAMP_COLOR, 2, cv2.LINE_AA
                )
            if imu == 'base' or imu == 'both':
                base_angle_outer = self.__angles[BASE_OUTER, i] * SCALE_FACTOR
                base_angle_inner = -1 * self.__angles[BASE_INNER, i] * SCALE_FACTOR
                base_angle_outer = int(base_angle_outer)
                base_angle_inner = int(base_angle_inner)
                base_end_x = base_angle_outer + self.mid_x
                base_end_y = base_angle_inner + self.mid_y
                cv2.arrowedLine(
                    frame,
                    (self.mid_x, self.mid_y),
                    (base_end_x, base_end_y),
                    BASE_COLOR,
                    thickness=2,
                    line_type=cv2.LINE_AA
                )
            if imu == 'lamp' or imu == 'both':
                lamp_angle_outer = self.__angles[LAMP_OUTER, i] * SCALE_FACTOR
                lamp_angle_inner = -1 * self.__angles[LAMP_INNER, i] * SCALE_FACTOR
                lamp_angle_outer = int(lamp_angle_outer)
                lamp_angle_inner = int(lamp_angle_inner)
                if imu == 'both':
                    start_x = base_end_x
                    start_y = base_end_y
                else:
                    start_x = self.mid_x
                    start_y = self.mid_y
                cv2.arrowedLine(
                    frame,
                    (start_x, start_y),
                    (start_x + lamp_angle_outer, start_y + lamp_angle_inner),
                    LAMP_COLOR,
                    thickness=2,
                    line_type=cv2.LINE_AA
                )
            video.write(frame)
        video.release()

    def plot_birds_eye_view_reference_wall(self, frame):
        '''
        Plots wall upon which base is mounted, for reference
        '''
        cv2.line(frame, (10, 0), (10, self.height), BLACK, 20)

    def plot_birds_eye_view_axis_labels(self, frame, scale):
        '''
        Add some axis labels to frame
        '''
        # Add +/- labels for some points
        for i in range (-30, 40, 10):
            if i == 0:
                continue
            angle = i * 1.0 * np.pi / 180.0
            dist = self.L * np.sin(angle);
            dist_x = int(dist * scale) + self.mid_x
            dist_y = int(dist * scale) + self.mid_y
            dist = np.round(dist, 2)
            cv2.line(
                frame, (dist_x, 0), (dist_x, self.height),
                GREY, thickness=1, lineType=cv2.LINE_AA
            )
            cv2.line(
                frame, (0, dist_y), (self.width, dist_y),
                GREY, thickness=1, lineType=cv2.LINE_AA
            )
            cv2.putText(
                frame, str(dist),
                (dist_x, self.mid_y + 15),
                font, 0.5, BLACK, 1, cv2.LINE_AA
            )
            cv2.putText(
                frame, str(dist),
                (self.mid_x + 10, dist_y),
                font, 0.5, BLACK, 1, cv2.LINE_AA
            )
    
    def do_birds_eye_view_animation(self, imu, decomp=False):
        '''
        Project motion onto a 2D plane below the lamp

        Arguments
        --------
            imu : int
                Index of IMU to plot
            decomp : bool
                Shows vector decomposition if true, otherwise only the
                resultant is shown
        '''
        assert(imu == 'base' or imu == 'lamp' or imu == 'both'), "Invalid imu value"
        fname = 'birds_eye_view' + '_' + imu
        if decomp:
            fname += '_decomp'
        fname += '.avi'
        fname = os.path.join(self.base_name, self.fname + '_' + fname)
        fourcc = VideoWriter_fourcc(*'MP42')
        video = VideoWriter(fname, fourcc, float(self.FPS), (self.width, self.height))
        SCALE_FACTOR = 80
        L = self.L * SCALE_FACTOR
        for i in range(0, self.__t.shape[0], int(1 / (self.TS * self.FPS))):
            frame = np.full((self.height, self.width, 3), 65535, dtype=np.uint8) # Fill white
            self.plot_birds_eye_view_axis_labels(frame, SCALE_FACTOR)
            self.plot_birds_eye_view_reference_wall(frame)
            self.plot_axes(frame, "x [cm]", "y [cm]", 60, 15)
            cv2.putText(
                frame, 'Time: %d' % self.__t[i],
                (25, self.height - 30),
                font, 0.5, BLACK, 1, cv2.LINE_AA
            )
            pos = []
            if imu == 'base' or imu == 'both':
                angle_outer = self.__angles[BASE_OUTER, i] * np.pi / 180.0
                angle_inner = self.__angles[BASE_INNER, i] * np.pi / 180.0
                d_outer = int(L * np.sin(angle_outer));
                d_inner = int(L * np.sin(angle_inner));
                pos.append((d_outer, d_inner))
            if imu == 'lamp' or imu == 'both':
                angle_outer = self.__angles[LAMP_OUTER, i] * np.pi / 180.0
                angle_inner = self.__angles[LAMP_INNER, i] * np.pi / 180.0
                d_outer = int(L * np.sin(angle_outer));
                d_inner = int(L * np.sin(angle_inner));
                pos.append((d_outer, d_inner))
            if decomp:
                # Color legend
                if imu == 'base' or imu == 'both':
                    cv2.putText(
                        frame, 'Base', (10,30),
                        font, 1,  BASE_COLOR, 2, cv2.LINE_AA
                    )
                if imu == 'lamp' or imu == 'both':
                    if imu == 'both':
                        start_y = 60
                    else:
                        start_y = 30
                    cv2.putText(
                        frame, 'Lamp', (10,start_y),
                        font, 1, LAMP_COLOR, 2, cv2.LINE_AA
                    )
            last_x, last_y = self.mid_x, self.mid_y
            first = True
            for x,y in pos:
                new_x = last_x + x
                new_y = last_y + y
                if decomp:
                    if imu == 'base' or (imu == 'both' and first):
                        first = False
                        cv2.arrowedLine(
                            frame, (last_x, last_y), (new_x, new_y),
                            BASE_COLOR, thickness=2, line_type=cv2.LINE_AA
                        )
                    elif imu == 'lamp' or (imu == 'both' and not first):
                        cv2.arrowedLine(
                            frame, (last_x, last_y), (new_x, new_y),
                            LAMP_COLOR, thickness=2, line_type=cv2.LINE_AA
                        )
                last_x, last_y = new_x, new_y
            cv2.circle(
                frame,
                (last_x, last_y),
                10, (0, 0, 255),
                -1, 
                lineType=cv2.LINE_AA
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