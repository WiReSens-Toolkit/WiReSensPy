import numpy as np
class stepCountTracker():
    def __init__(self):
        self.step=False
        self.stepCount = 0
    def checkStep(self,socket, pressure):
        avg = np.mean(pressure)
        threshold = 1200
        if avg < threshold and not self.step:
            self.step=True
            self.stepCount+=1
            print(self.stepCount)
            socket.emit('step', self.stepCount)
        elif avg >= threshold:
            self.step = False


