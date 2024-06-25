import numpy as np
import h5py
import time
import struct
from datetime import datetime
from Sensor import Sensor
from matplotlib import pyplot as plt
from matplotlib import animation

class GenericReceiverClass():
    def __init__(self, readWires, selWires, sensor_ids):
        self.frameRate = None
        self.startTime = time.time()
        self.readWires = readWires
        self.selWires = selWires
        self.sensors = {sensor_id: Sensor(readWires, selWires, sensor_id) for sensor_id in sensor_ids}
        self.caxs = []

        self.block_size = readWires*selWires
        self.pressure_min = 0
        self.pressure_max = 4096
        self.use_log = True

    def startReceiver(self):
        raise NotImplementedError("Receivers must implement a startReceiver method")
    
    def unpackBytes(self, byteString):
        format_string = 'bb' + 'H' * self.selWires  # 'b' for int8_t, 'H' for uint16_t
        tupData = struct.unpack(format_string,byteString)
        sendId = tupData[0]
        yIndex = tupData[1]
        sensorReadings = tupData[2:]
        return sendId, yIndex, sensorReadings
    def unpackBytesPacket(self, byteString):
        format_string = '=b' + 'H' * 121 + 'I'  # 'b' for int8_t, 'H' for uint16_t
        tupData = struct.unpack(format_string,byteString)
        sendId = tupData[0]
        startIdx = tupData[1]
        sensorReadings = tupData[2:-1]
        packetNumber = tupData[-1]
        return sendId, startIdx,sensorReadings, packetNumber

    

    def startViz(self):
        self.fig, axes = plt.subplots(1, len(self.sensors))  # Create subplots for each heatmap
        if type(axes) is list:
            self.axes= axes
        else:
            self.axes = [axes]
        sensIds = list(self.sensors.keys())
        for i in range(len(sensIds)):
            self.axes[i].set_title(f"Sensor: {sensIds[i]}")
            self.axes[i].axis('off')
            plt.tight_layout()
            self.caxs.append(self.axes[i].imshow(np.zeros((self.readWires, self.selWires)), cmap='viridis', vmin=self.pressure_min, vmax=self.pressure_max))
            plt.colorbar(self.caxs[i])
        def updateFrame(frame):
            for i in range(len(self.sensors)):
                self.caxs[i].set_array(self.sensors[sensIds[i]].pressure.reshape(self.readWires,self.selWires))
            return self.caxs
        ani = animation.FuncAnimation(self.fig, updateFrame, interval=1000/60)  # ~60 FPS
        plt.show()



    
    