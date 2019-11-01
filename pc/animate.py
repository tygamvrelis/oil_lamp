
import cv2
import numpy as np
from cv2 import VideoWriter, VideoWriter_fourcc

width = 640
height = 480
FPS = 60
seconds = 10
radius = 25

# Lamp-specific stuff
SCALE_FACTOR = 400
L = 0.52 * SCALE_FACTOR # Make it look large in the frame
m = 2.0
g = 9.81
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