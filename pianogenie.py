import numpy as np
from pynput.keyboard import Controller, Key
import time

# Initialize mouse controller
keyboard = Controller()

class real_time_peak_detection():
    def __init__(self, array, lag, threshold, influence):
        self.y = list(array)
        self.length = len(self.y)
        self.lag = lag
        self.threshold = threshold
        self.influence = influence
        self.signals = [0] * len(self.y)
        self.filteredY = np.array(self.y).tolist()
        self.avgFilter = [0] * len(self.y)
        self.stdFilter = [0] * len(self.y)
        self.avgFilter[self.lag - 1] = np.mean(self.y[0:self.lag]).tolist()
        self.stdFilter[self.lag - 1] = np.std(self.y[0:self.lag]).tolist()

    def thresholding_algo(self, new_value):
        self.y.append(new_value)
        i = len(self.y) - 1
        self.length = len(self.y)
        if i < self.lag:
            return 0
        elif i == self.lag:
            self.signals = [0] * len(self.y)
            self.filteredY = np.array(self.y).tolist()
            self.avgFilter = [0] * len(self.y)
            self.stdFilter = [0] * len(self.y)
            self.avgFilter[self.lag] = np.mean(self.y[0:self.lag]).tolist()
            self.stdFilter[self.lag] = np.std(self.y[0:self.lag]).tolist()
            return 0

        self.signals += [0]
        self.filteredY += [0]
        self.avgFilter += [0]
        self.stdFilter += [0]

        if abs(self.y[i] - self.avgFilter[i - 1]) > (self.threshold * self.stdFilter[i - 1]):

            if self.y[i] > self.avgFilter[i - 1]:
                self.signals[i] = 1
            else:
                self.signals[i] = -1

            self.filteredY[i] = self.influence * self.y[i] + \
                (1 - self.influence) * self.filteredY[i - 1]
            self.avgFilter[i] = np.mean(self.filteredY[(i - self.lag):i])
            self.stdFilter[i] = np.std(self.filteredY[(i - self.lag):i])
        else:
            self.signals[i] = 0
            self.filteredY[i] = self.y[i]
            self.avgFilter[i] = np.mean(self.filteredY[(i - self.lag):i])
            self.stdFilter[i] = np.std(self.filteredY[(i - self.lag):i])

        return self.signals[i]

#Example custom method run using WiReSens Toolkit to use pressure sensor as a remote control

def actuateKeys(pressureGrid, handRegions, handState, keys):
    handAvg = [np.mean(pressureGrid[region[1],region[0]]) for region in handRegions]
        # Key press/release logic
    for i in range(len(handRegions)):
        handRead = handAvg[i]
        changed =handState[i].thresholding_algo(handRead)
        # if i==0:
        #     print(f"rightRead: {handRead}") 
        if changed==-1:
            # print(f"Key changed: {keys[i]}")
            keyboard.press(keys[i])
        else:
            keyboard.release(keys[i])
def startController(sensors):
    thumbTop = (slice(13,16), slice(12,16))
    indexTop =(slice(9,11), slice(0,3))
    middleTop = (slice(6,8),slice(0,3))
    ringTop = (slice(3,5),slice(0,3))
    pinkyTop = (slice(0,2),slice(0,3))
    indexTopL = (slice(13,16), slice(5,7))
    middleTopL = (slice(13,16), slice(8,10))
    ringTopL = (slice(13,16), slice(11,13))
    pinkyTopL = (slice(13,16), slice(14,16))
    handRegions = [indexTop, middleTop, ringTop, pinkyTop]
    handRegionsL = [indexTopL, middleTopL, ringTopL, pinkyTopL]

    # Initialize peak detector
    lags = [200,200,200,200]
    thresholds = [5,5,5,5.5]
    thresholdsl = [8,8,8,7]
    influences = [0,0,0,0]
    rightHandState = [real_time_peak_detection([2100]*lags[i], lags[i], thresholds[i], influences[i]) for i in range(4)]
    leftHandState =[real_time_peak_detection([2100]*lags[i], lags[i], thresholdsl[i], influences[i]) for i in range(4)]
    rightKeys=['j','k','l',';']
    leftKeys = ['f','d','s','a']
    running = True
    while running:
        pressureGridL = sensors[0].pressure.reshape(sensors[0].selWires, sensors[0].readWires)
        pressureGridR = sensors[1].pressure.reshape(sensors[1].selWires, sensors[1].readWires)
        actuateKeys(pressureGridL,handRegionsL,leftHandState,leftKeys)
        actuateKeys(pressureGridR,handRegions,rightHandState,rightKeys)
        
        time.sleep(0.005)
                # else:
                #     if handRead < rightThreshold and handRead!=0 and not self.rightHandState[i]:
                #         self.player.note_on(rightNotes[i],127)
                #         self.rightHandState[i]=True
                #     else:
                #         if handRead> rightThreshold and self.rightHandState[i]:
                #             self.player.note_off(rightNotes[i],127)
                #             self.rightHandState[i] = False
