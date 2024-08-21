import numpy as np
from pynput.keyboard import Controller, Key
import time

# Initialize mouse controller
keyboard = Controller()

from collections import deque

class FrameBuffer:
    def __init__(self, m, n, w):
        self.m = m  # Number of rows in each frame
        self.n = n  # Number of columns in each frame
        self.w = w  # Number of frames to maintain
        self.buffer = deque(maxlen=w)  # Deque to store the frames
    
    def add_frame(self, frame):
        # Ensure the frame is of correct shape
        if frame.shape != (self.m, self.n):
            raise ValueError(f"Frame must be of shape ({self.m}, {self.n})")
        
        # Add the new frame to the buffer (automatically removes oldest if needed)
        self.buffer.append(frame)
    
    def get_moving_average(self):
        # Calculate the moving average if there are frames in the buffer
        if not self.buffer:
            raise ValueError("Buffer is empty. Add frames before calculating the average.")
        
        # Stack all frames and calculate the mean along the first axis
        stacked_frames = np.stack(self.buffer)
        return np.mean(stacked_frames, axis=0)

def startController(sensors):
    print(len(sensors))
    # Constants
    MAT_SIZE = 32
    MAX_PRESSURE = 1000

    volumeUpRegion = (slice(0,8), slice(16,32))
    volumeDownRegion = (slice(24,32), slice(16,32))
    playRegion = (slice(0,10),slice(0,16))
    pauseRegion = (slice(20,32),slice(0,10))

    playPauseThreshold = 2300
    # (calibrated)
    # volumeUpThreshold=1800 
    #  volumeDownThreshold = 1900
    # (uncalibrated)
    volumeUpThreshold = 1400
    volumeDownThreshold = 1800

   
    minVolumeTheshold = 1550
    #50 presses in 4 seconds (0-100 volume)
    # Main drawing loop
    running = True
    paused = False
    while running:
        pressureGrid = sensors[0].pressure.reshape(sensors[0].selWires, sensors[0].readWires)
        if sensors[0].init:
            volumeUpAvg = np.mean(pressureGrid[volumeUpRegion[0],volumeUpRegion[1]])
            volumeDownAvg = np.mean(pressureGrid[volumeDownRegion[0],volumeDownRegion[1]])
            playAvg = np.mean(pressureGrid[playRegion[0],playRegion[1]])
            pauseAvg = np.mean(pressureGrid[pauseRegion[0],pauseRegion[1]])

            if playAvg <= playPauseThreshold and paused:
                print("Play")
                print(playAvg)
                keyboard.press(Key.media_play_pause)
                paused=False
            elif pauseAvg <= playPauseThreshold and not paused:
                print("Pause")
                print(pauseAvg)
                keyboard.press(Key.media_play_pause)
                paused = True

            if volumeUpAvg <volumeUpThreshold:
                print("Volume Up")
                sleepFactor = 3
                diff = volumeUpThreshold - volumeUpAvg
                if diff < 150:
                    sleepFactor = 5
                elif diff < 300:
                    sleepFactor = 15
                else:
                    sleepFactor = 30
                keyboard.press(Key.media_volume_up)
                print(sleepFactor)
                time.sleep(1/sleepFactor)
            elif volumeDownAvg < volumeDownThreshold:
                sleepFactor = 3
                diff = volumeDownThreshold - volumeDownAvg
                if diff < 150:
                    sleepFactor = 5
                elif diff < 300:
                    sleepFactor = 15
                else:
                    sleepFactor = 30
                print("Volume Down")
                print(sleepFactor)
                keyboard.press(Key.media_volume_down)
                time.sleep(1/sleepFactor)

