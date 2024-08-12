import numpy as np
import json5
import serial
import time
from matplotlib import pyplot as plt
from datetime import datetime
from datetime import date
from GenericReceiver import GenericReceiverClass
import socket
import select
import threading
from typing import List
from Sensor import Sensor
from matplotlib import animation
import aioconsole

import asyncio
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
import serial_asyncio
import concurrent.futures

def getUnixTimestamp():
    return np.datetime64(datetime.now()).astype(np.int64) / 1e6  # unix TS in secs and microsecs

class WifiReceiver(GenericReceiverClass):
    def __init__(self,numNodes,sensors:List[Sensor], tcp_ip="10.0.0.67", tcp_port=7000, viz=False, stopFlag=None):
        super().__init__(numNodes,sensors,viz)
        self.TCP_IP = tcp_ip
        self.tcp_port = tcp_port
        self.connection_is_open = False
        self.connections = {}
        self.setup_TCP()
        self.stopFlag = stopFlag
    
    def setup_TCP(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.TCP_IP, self.tcp_port))
        sock.listen(len(self.sensors))  # Listen for N connections
        print("Waiting for connections")
        while len(self.connections) < len(self.sensors):
            connection, client_address = sock.accept()
            print("Connection found")
            sensorId = self.getSensorIdFromBuffer(connection)
            print(f"Connection found from {sensorId}")
            self.connections[sensorId]=connection
        sock.settimeout(30)
        print("All connections found")

    def getSensorIdFromBuffer(self, connection):
        while True:
            ready_to_read, ready_to_write, in_error = select.select([connection], [], [], 30)
            if len(ready_to_read)>0:
                numBytes = 1+(self.numNodes+1)*2+4
                inBuffer =   connection.recv(numBytes, socket.MSG_PEEK)
                if len(inBuffer) >= numBytes:
                    sendId, startIdx, sensorReadings, packet = self.unpackBytesPacket(inBuffer)
                return sendId

    def reconnect(self, sensorId):
        print(f"Reconnecting to sensor {sensorId}")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.TCP_IP, self.tcp_port))
        sock.listen(len(self.sensors))
        print("waiting for connections")
        found_Conn = False
        while not found_Conn:
            connection, client_address = sock.accept()
            connectionSensorId = self.getSensorIdFromBuffer(connection)
            if connectionSensorId == sensorId:
                print(f"Connection from Sensor {sensorId} found")
                self.connections[sensorId]=connection
                found_Conn = True
            else:
                print(f"Connection refused from {client_address}")
                connection.close()

    async def receiveData(self, sensorId):
        print("Receiving Data")
        while not self.stopFlag.is_set():
            connection = self.connections[sensorId]
            ready_to_read, ready_to_write, in_error = await asyncio.get_event_loop().run_in_executor(
                None, select.select, [connection], [], [], 30)
            if len(ready_to_read)>0:
                numBytes = 1+(self.numNodes+1)*2+4
                inBuffer =   await asyncio.get_event_loop().run_in_executor(None, connection.recv, numBytes, socket.MSG_PEEK)
                if len(inBuffer) >= numBytes:
                    data = await asyncio.get_event_loop().run_in_executor(None, connection.recv, numBytes)
                    sendId, startIdx, sensorReadings, packet = self.unpackBytesPacket(data)
                    sensor = self.sensors[sendId]
                    if (sensor.intermittent):
                        sensor.processRowIntermittent(startIdx,sensorReadings,packet)
                    else:
                        sensor.processRow(startIdx,sensorReadings,packet)
            else:
                print(f"Sensor {sensorId} is disconnected: Reconnecting...")
                await asyncio.get_event_loop().run_in_executor(None, connection.shutdown, 2)
                await asyncio.get_event_loop().run_in_executor(None, connection.close)
                self.reconnect(sensorId)

    def startReceiverThreads(self):
        tasks = []
        for sensorId in self.connections:
            task = self.receiveData(sensorId)
            tasks.append(task)
        return tasks
    
    def startReceiver(self):
        threads = []
        for sensorId in self.connections:
            thread = threading.Thread(target=self.receiveData, args=(sensorId,))
            thread.start()
            threads.append(thread)
        if self.viz:
            thread = threading.Thread(target=self.startViz)
            thread.start()
            threads.append(thread)
        
        for thread in threads:
            thread.join()


class BLEReceiver(GenericReceiverClass):
    def __init__(self, numNodes, sensors: List[Sensor],viz=False):
        super().__init__(numNodes, sensors, viz)
        self.deviceNames = [sensor.deviceName for sensor in sensors]
        self.clients={}
        self.viz = viz

    def collectData(self):
        asyncio.run(self.startReceiverAsync())
    
    def startReceiver(self):
        threads=[]
        captureThread = threading.Thread(target=self.collectData)
        captureThread.start()
        threads.append(captureThread)
        if self.viz:
            thread = threading.Thread(target=self.startViz)
            thread.start()
            threads.append(thread)
        for thread in threads:
            thread.join()

    async def connect_to_device(self, lock, deviceName):
        def on_disconnect(client):
            print(f"Device {deviceName} disconnected, attempting to reconnect...")
            asyncio.create_task(self.connect_to_device(lock, deviceName))
        async with lock:
            device = await BleakScanner.find_device_by_name(deviceName,timeout=30)
            if device:
                print(f"Found device: {deviceName}")
                client = BleakClient(device)
                client.set_disconnected_callback(on_disconnect)
                self.clients[deviceName] = client
                await client.connect()

        def notification_handler(characteristic: BleakGATTCharacteristic, data: bytearray):
            sendId, startIdx, sensorReadings, packet = self.unpackBytesPacket(data)
            sensor = self.sensors[sendId]
            if(sensor.intermittent):
                sensor.processRowIntermittent(startIdx,sensorReadings,packet)
            else:
                sensor.processRow(startIdx,sensorReadings,packet)

        await client.start_notify("1766324e-8b30-4d23-bff2-e5209c3d986f", notification_handler)
        print(f"Connected to {deviceName}")
    
    async def startReceiverAsync(self):
        lock = asyncio.Lock()
        tasks = [self.connect_to_device(lock, name) for name in self.deviceNames]
        await asyncio.gather(*tasks)
        print("All devices connected and notifications started.")
        await self.listen_for_stop()
    
    async def stopReceiver(self):
        for client in self.clients.values():
            await client.stop_notify("1766324e-8b30-4d23-bff2-e5209c3d986f")
            await client.disconnect()
        print("All notifications stopped and devices disconnected.")

    def startReceiverThreads(self):
        lock = asyncio.Lock()
        tasks = [self.connect_to_device(lock, name) for name in self.deviceNames]
        return tasks


    


class SerialReceiver(GenericReceiverClass):
    def __init__(self, numNodes, sensors, port, baudrate, viz =False):
        super().__init__(numNodes, sensors, viz)
        self.port = port #update serial port
        self.baudrate = baudrate
        self.stop_capture_event = False
        self.reader = None
        self.stopStr = bytes('wr','utf-8')

    async def read_serial(self):
        print("Reading Serial")
        self.reader, _ = await serial_asyncio.open_serial_connection(url=self.port, baudrate=self.baudrate)
        while not self.stopFlag.is_set():
            data = await self.reader.read(2048)  # Read available bytes
            if data:
                await self.buffer.put(data)

    

    async def stopReceiver(self):
        print("Stopped reading")


    async def startReceiverAsync(self):
        tasks=[]
        tasks.append(asyncio.create_task(self.read_serial()))
        tasks.append(asyncio.create_task(self.read_lines()))
        tasks.append(asyncio.create_task(self.listen_for_stop()))
        if self.viz:
            loop = asyncio.get_running_loop()
            executor = concurrent.futures.ThreadPoolExecutor()
            tasks.append(loop.run_in_executor(executor, self.startViz))
        print("Tasks appended")
        await asyncio.gather(*tasks)

    def startReceiverThreads(self):
        tasks=[]
        tasks.append(asyncio.create_task(self.read_serial()))
        tasks.append(asyncio.create_task(self.read_lines()))
        tasks.append(asyncio.create_task(self.listen_for_stop()))
        return tasks

        


    def startReceiver(self):
        asyncio.run(self.startReceiverAsync())


def readConfigFile(file):
    with open(file, 'r') as file:
        data = json5.load(file)
    return data

class MultiProtocolReceiver():
    def __init__(self, configFilePath="./WiSensConfigClean.json"):
        self.config = readConfigFile(configFilePath)
        self.sensors = self.config['sensors']
        self.bleSensors = []
        self.wifiSensors = []
        self.serialSensors = []
        self.allSensors = []
        self.stopFlag = asyncio.Event()
        for sensorConfig in self.sensors:
            sensorKeys = list(sensorConfig.keys())
            intermittent = False
            p = 15

            if 'intermittent' in sensorKeys:
                intermittent = sensorConfig['intermittent']['enabled']
                p = sensorConfig['intermittent']['p']

            deviceName = "Esp1"
            numNodes = 120

            match sensorConfig['protocol']:
                case 'wifi':
                    numNodes = self.config['wifiOptions']['numNodes']
                case 'ble':
                    deviceName = sensorConfig['deviceName']
                    numNodes = self.config['bleOptions']['numNodes']
                case 'serial':
                    numNodes = self.config['serialOptions']['numNodes']
                


            newSensor = Sensor(sensorConfig['numGroundWires'], sensorConfig['numReadWires'],numNodes,sensorConfig['id'],deviceName=deviceName,intermittent=intermittent, p=p)
            
            match sensorConfig['protocol']:
                case 'wifi':
                    self.wifiSensors.append(newSensor)
                case 'ble':
                    self.bleSensors.append(newSensor)
                case 'serial':
                    self.serialSensors.append(newSensor)
            self.allSensors.append(newSensor)

        self.receivers = []
        self.receiveTasks = []
        if len(self.bleSensors)!=0:
            bleReceiver = BLEReceiver(self.config['bleOptions']['numNodes'],self.bleSensors)
            self.receivers.append(bleReceiver)
            self.receiveTasks += bleReceiver.startReceiverThreads()
        if len(self.wifiSensors)!=0:
            wifiReceiver = WifiReceiver(self.config['wifiOptions']['numNodes'],self.wifiSensors,self.config['wifiOptions']['tcp_ip'],self.config['wifiOptions']['port'], stopFlag=self.stopFlag)
            self.receivers.append(wifiReceiver)
            self.receiveTasks += wifiReceiver.startReceiverThreads()
        if len(self.serialSensors)!=0:
            serialReceiver = SerialReceiver(self.config['serialOptions']['numNodes'],self.serialSensors,self.config['serialOptions']['port'],self.config['serialOptions']['baudrate'])
            self.receivers.append(serialReceiver)
            self.receiveTasks += serialReceiver.startReceiverThreads()
        self.receiveTasks.append(self.listen_for_stop())
    
    async def startReceiversAsync(self):
        await asyncio.gather(*self.receiveTasks)
        # await self.listen_for_stop()

    async def listen_for_stop(self):
        print("Listening for stop")
        stop_flag = False
        while not stop_flag:
            input_str = await aioconsole.ainput("Press Enter to stop...\n")
            if input_str == "":
                stop_flag = True
                self.stopFlag.set()

    def collectData(self):
        asyncio.run(self.startReceiversAsync())

    def startViz(self):
        pressure_min = 0
        pressure_max = 4096
        fig, axes = plt.subplots(1, len(self.allSensors))  # Create subplots for each heatmap
        thisAxes = None
        thisCaxs=[]
        if isinstance(axes,list) or isinstance(axes,np.ndarray):
            thisAxes= axes
        else:
            thisAxes = [axes]
        for i in range(len(self.allSensors)):
            thisAxes[i].set_title(f"Sensor: {self.allSensors[i].id}")
            thisAxes[i].axis('off')
            plt.tight_layout()
            thisCaxs.append(thisAxes[i].imshow(np.zeros((self.allSensors[i].selWires, self.allSensors[i].readWires)), cmap='viridis', vmin=pressure_min, vmax=pressure_max))
            plt.colorbar(thisCaxs[i])
        def updateFrame(frame):
            for i in range(len(self.allSensors)):
                thisCaxs[i].set_array(self.allSensors[i].pressure.reshape(self.allSensors[i].selWires, self.allSensors[i].readWires))
            return thisCaxs
        ani = animation.FuncAnimation(fig, updateFrame, interval=1000/60)  # ~60 FPS
        plt.show()
    

if __name__ == "__main__":
    receiverModule = MultiProtocolReceiver()
    threads=[]
    captureThread = threading.Thread(target=receiverModule.collectData)
    captureThread.start()
    threads.append(captureThread)
    thread = threading.Thread(target=receiverModule.startViz)
    thread.start()
    threads.append(thread)

    for thread in threads:
        thread.join()
