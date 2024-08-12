import numpy as np
import time
import struct
from Sensor import Sensor
from matplotlib import pyplot as plt
from matplotlib import animation
import aioconsole
import asyncio
from typing import List

class GenericReceiverClass():
    def __init__(self, numNodes, sensors: List[Sensor], viz):
        self.frameRate = None
        self.startTime = time.time()
        self.numNodes = numNodes
        self.sensors = {sensor.id: sensor for sensor in sensors}
        self.caxs = []

        self.viz = viz
        self.pressure_min = 0
        self.pressure_max = 4096
        self.use_log = True

        self.partialData = b''
        self.buffer = asyncio.Queue()

    def startReceiver(self):
        raise NotImplementedError("Receivers must implement a startReceiver method")
    async def stopReceiver(self):
        raise NotImplementedError("Receivers must implement a stop receiver method")
    

    def unpackBytesPacket(self, byteString):
        format_string = '=b' + 'H' * (1+self.numNodes) + 'I'  # 'b' for int8_t, 'H' for uint16_t
        tupData = struct.unpack(format_string,byteString)
        sendId = tupData[0]
        startIdx = tupData[1]
        sensorReadings = tupData[2:-1]
        packetNumber = tupData[-1]
        return sendId, startIdx,sensorReadings, packetNumber

    

    def startViz(self):
        self.fig, axes = plt.subplots(1, len(self.sensors))  # Create subplots for each heatmap
        print(type(axes))
        if isinstance(axes,list) or isinstance(axes,np.ndarray):
            self.axes= axes
        else:
            self.axes = [axes]
        sensIds = list(self.sensors.keys())
        for i in range(len(sensIds)):
            self.axes[i].set_title(f"Sensor: {sensIds[i]}")
            self.axes[i].axis('off')
            plt.tight_layout()
            self.caxs.append(self.axes[i].imshow(np.zeros((self.sensors[sensIds[i]].readWires, self.sensors[sensIds[i]].selWires)), cmap='viridis', vmin=self.pressure_min, vmax=self.pressure_max))
            plt.colorbar(self.caxs[i])
        def updateFrame(frame):
            for i in range(len(self.sensors)):
                self.caxs[i].set_array(self.sensors[sensIds[i]].pressure.reshape(self.sensors[sensIds[i]].readWires, self.sensors[sensIds[i]].selWires))
            return self.caxs
        ani = animation.FuncAnimation(self.fig, updateFrame, interval=1000/60)  # ~60 FPS
        plt.show()

    async def process_line(self, line):
        if len(line) == 1+(1+self.numNodes)*2+4:
            sendId, startIdx, readings, packetID = self.unpackBytesPacket(line)
            sensor = self.sensors[sendId]
            await sensor.processRowAsync(startIdx, readings, packetID)

    async def read_lines(self):
        print("Reading lines")
        while not self.stopFlag.is_set():
            data = await self.buffer.get()
            self.partialData += data
            lines = self.partialData.split(b'wr')
            self.partialData = lines.pop()
            for line in lines:
                await self.process_line(line)

    async def listen_for_stop(self):
        print("Listening for stop")
        while not self.stopFlag.is_set():
            input_str = await aioconsole.ainput("Press Enter to stop...\n")
            if input_str == "":
                self.stopFlag.set()
                await self.stopReceiver()



    
    