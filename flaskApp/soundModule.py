import numpy as np
import pygame

import pygame.midi

class midiPlayer():
    def __init__(self):
        pygame.midi.init()
        self.player = pygame.midi.Output(0)
        self.player.set_instrument(0)  # Set instrument, e.g., Acoustic Grand Piano
         # Set activation regions
        leftIndexRegion = (slice(0,5), slice(4,7))
        leftPinkieRegion = (slice(0,4), slice(13,15))
        leftThumbRegion = (slice(12,15), slice(1,4))
        lefttMiddleRegion = (slice(0,5),slice(7,10))
        leftRingRegion = (slice(0,5),slice(10,13))

        self.handRegions = [leftPinkieRegion, leftRingRegion, lefttMiddleRegion, leftIndexRegion, leftThumbRegion]
        self.leftHandState = [False, False, False, False, False]
        self.rightHandState = [False, False,False,False,False]
        # rightHand = [rightIndexRegion, rightPinkieRegion, rightThumbRegion,rightMiddleRegion,rightRingRegion]


    def play_sound(self, pressure, left=False):
        handAvg = [np.mean(pressure[region[0],region[1]]) for region in self.handRegions]

        # Set pressure thresholds
        rightThreshold=1400
        leftThreshold = 1350

        #Set notes
        C=60
        D=62
        E=64
        F=65
        G=67
        leftNotes = [C,D,E,F,G]
        rightNotes = [G+12, F+12, E+12, D+12, C+12]
        for i in range(len(self.handRegions)):
            handRead = handAvg[i]
            if i==0:
                if left: 
                    print(f"leftRead: {handRead}") 
            if left:
                if handRead < leftThreshold and handRead!=0 and not self.leftHandState[i]:
                    self.player.note_on(leftNotes[i],127)
                    self.leftHandState[i]=True
                else:
                    if handRead > leftThreshold and self.leftHandState[i]:
                        self.player.note_off(leftNotes[i],127)
                        self.leftHandState[i] = False
            else:
                if handRead < rightThreshold and handRead!=0 and not self.rightHandState[i]:
                    self.player.note_on(rightNotes[i],127)
                    self.rightHandState[i]=True
                else:
                    if handRead> rightThreshold and self.rightHandState[i]:
                        self.player.note_off(rightNotes[i],127)
                        self.rightHandState[i] = False