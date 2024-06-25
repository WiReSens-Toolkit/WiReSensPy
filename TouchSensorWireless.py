import numpy as np
import serial
import time
from matplotlib import pyplot as plt
from datetime import datetime
from datetime import date
from GenericReceiver import GenericReceiverClass
import socket
import select
import keyboard
import threading
from typing import List

import asyncio
from bleak import BleakClient, BleakScanner
from bleak.backends.characteristic import BleakGATTCharacteristic
import aioconsole

def getUnixTimestamp():
    return np.datetime64(datetime.now()).astype(np.int64) / 1e6  # unix TS in secs and microsecs

class WifiReceiver(GenericReceiverClass):
    def __init__(self,readWires, selWires,sendIds, tcp_ip="10.0.0.67", tcp_port=7000, viz=False):
        super().__init__(readWires, selWires, sendIds)
        self.TCP_IP = tcp_ip
        self.tcp_port = tcp_port
        self.connection_is_open = False
        self.connections = {}
        self.setup_TCP()
        self.stop_capture_event = False
        self.viz = viz
    
    def setup_TCP(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.TCP_IP, self.tcp_port))
        sock.listen(len(self.sensors))  # Listen for N connections
        print("Waiting for connections")
        while len(self.connections) < len(self.sensors):
            connection, client_address = sock.accept()
            print("Connection found from", client_address)
            self.connections[client_address]=connection
        sock.settimeout(30)
        print("All connections found")

    def reconnect(self, address):
        print(f"Reconnecting to {address}")
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((self.TCP_IP, self.tcp_port))
        sock.listen(len(self.sensors))
        print("waiting for connections")
        found_Conn = False
        while not found_Conn:
            connection, client_address = sock.accept()
            if client_address == address:
                print(f"Connection from {address} found")
                self.connections[client_address]=connection
                found_Conn = True
            else:
                print(f"Connection refused from {client_address}")
                connection.close()




    def receiveData(self, address):
        connection = self.connections[address]
        while not self.stop_capture_event:
            ready_to_read, ready_to_write, in_error = select.select([connection,],[],[],30)
            if len(ready_to_read)>0:
                numBytes = 1+121*2+4
                inBuffer =  connection.recv(numBytes, socket.MSG_PEEK)
                if len(inBuffer) >= numBytes:
                    data = connection.recv(numBytes)
                    sendId, startIdx, sensorReadings, packet = self.unpackBytesPacket(data)
                    sensor = self.sensors[sendId]
                    sensor.processRow(startIdx,sensorReadings,packet)
            else:
                print(f"ESP32 on address {address} disconnected: Reconnecting...")
                connection.shutdown(2)
                connection.close()
                self.reconnect(address)
            if keyboard.is_pressed("enter"):
                self.stop_capture_event=True

    
    def startReceiver(self):
        threads = []
        for address in self.connections:
            thread = threading.Thread(target=self.receiveData, args=(address,))
            thread.start()
            threads.append(thread)
        if self.viz:
            thread = threading.Thread(target=self.startViz)
            thread.start()
            threads.append(thread)
        
        for thread in threads:
            thread.join()


class BLEReceiver(GenericReceiverClass):
    def __init__(self, readWires, selWires, sendIds, deviceNames: List[str], viz=False):
        super().__init__(readWires, selWires, sendIds)
        self.deviceNames = deviceNames
        self.stopFlag = asyncio.Event()
        self.clients={}
        self.viz = viz

    def notification_handler(self, characteristic: BleakGATTCharacteristic, data: bytearray):
        sendId, startIdx, sensorReadings, packet = self.unpackBytesPacket(data)
        sensor = self.sensors[sendId]
        sensor.processRow(startIdx,sensorReadings,packet)
        # print(sendId, yIndex, sensorData)

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

        

    async def connect_to_device(self, deviceName):
        device = await BleakScanner.find_device_by_name(deviceName,timeout=30)
        if device:
            print(f"Found device: {deviceName}")
            client = BleakClient(device)
            self.clients[deviceName] = client
            await client.connect()
            await client.start_notify("1766324e-8b30-4d23-bff2-e5209c3d986f", self.notification_handler)
            print(f"Connected to {deviceName}")
            self.startTime = time.time()
    
    async def startReceiverAsync(self):
        tasks = [self.connect_to_device(name) for name in self.deviceNames]
        await asyncio.gather(*tasks)
        print("All devices connected and notifications started.")

        await self.listen_for_stop()
    
    async def stopReceiver(self):
        for client in self.clients.values():
            await client.stop_notify("1766324e-8b30-4d23-bff2-e5209c3d986f")
            await client.disconnect()
        print("All notifications stopped and devices disconnected.")

    async def listen_for_stop(self):
        while not self.stopFlag.is_set():
            input_str = await aioconsole.ainput("Press Enter to stop...\n")
            if input_str == "":
                self.stopFlag.set()
                await self.stopReceiver()


class SerialReceiver(GenericReceiverClass):
    def __init__(self, readWires, selWires, sendIds, port, baudrate, viz =False):
        super().__init__(readWires, selWires, sendIds)
        self.port = port #update serial port
        self.baudrate = baudrate
        self.stop_capture_event = False
        self.viz = viz
        
    def startReceiver(self):
        threads=[]
        captureThread = threading.Thread(target=self.captureData)
        captureThread.start()
        threads.append(captureThread)
        if self.viz:
            thread = threading.Thread(target=self.startViz)
            thread.start()
            threads.append(thread)
        for thread in threads:
            thread.join()

        

    def captureData(self):
        self.ser = serial.Serial(self.port, baudrate=self.baudrate, timeout=1.0)
        assert self.ser.is_open, 'Failed to open COM port!'
        stopStr = bytes('wr','utf-8')
        while True:
            byteData = self.ser.read_until(expected=stopStr)[:-2]
            if len(byteData) == 1+121*2+4:
                sendId, startIdx, readings, packetID = self.unpackBytesPacket(byteData)
                sensor = self.sensors[sendId]
                sensor.processRow(startIdx, readings, packet=packetID)
                if keyboard.is_pressed('enter'):
                    print("keyboard press")
                    self.stop_capture_event=True



        
    
        
            
    




if __name__ == "__main__":
    # myReceiver = WifiReceiver(32,32,[1],tcp_ip="128.31.36.181",viz=False)
    myReceiver = BLEReceiver(32, 32, [1],["Esp1"],viz=False)
    # myReceiver = SerialReceiver(32,32,[1],"COM9",921600,viz=False)
    myReceiver.startReceiver()
