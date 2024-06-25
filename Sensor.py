import numpy as np
import h5py
import time
from datetime import datetime

def getUnixTimestamp():
    return np.datetime64(datetime.now()).astype(np.int64) / 1e6  # unix TS in secs and microsecs
class Sensor():
    def __init__(self, readWires:int, selWires:int, id):
        self.id = id
        self.readWires = readWires
        self.selWires = selWires
        self.pressure = np.zeros(32*32)
        path = f'./recordings_{id}_{str(time.time())}.hdf5'
        self.file = h5py.File(path, 'w')
        self.fc = 0
        self.init = False
        self.filledSize = 0
        self.bufferSize = 120
        self.left_to_fill = 1024
        self.block_size=1024
        self.packetCount = 0
        self.maxPackets = int(np.ceil(32*32/120))
        self.receivedPackets = np.zeros(self.maxPackets)

    def append_data(self, ts,reading, packet):
        f = self.file
        block_size = self.block_size
        fc = self.fc
        init = self.init
        if not init:
            self.init=True
            sz = [block_size, ]
            maxShape = sz.copy()
            maxShape[0] = None
            maxshapePressure = [None,self.readWires,self.selWires]
            f.create_dataset('frame_count', (1,),maxshape=maxShape, dtype=np.uint32)
            f.create_dataset('ts', tuple([block_size, ]), maxshape = maxShape, dtype=ts.dtype, chunks=True)
            f.create_dataset('pressure', tuple([block_size, self.readWires, self.selWires]), maxshape=maxshapePressure, dtype=np.int32, chunks=True)
            if packet is not None:
                maxshapePackets = [None, self.maxPackets]
                f.create_dataset('packetNumber', tuple([block_size, self.maxPackets]), maxshape=maxshapePackets, dtype=np.uint32, chunks=True)

        # Check size
        oldSize = f['ts'].shape[0]
        if oldSize == fc:
            newSize = oldSize + block_size
            f['ts'].resize(newSize, axis=0)
            f['pressure'].resize(newSize, axis=0)
            if packet is not None:
                f['packetNumber'].resize(newSize, axis=0)

        f['frame_count'][0] = fc
        f['ts'][fc] = ts
        f['pressure'][fc] = reading.reshape(1, self.readWires, self.selWires)
        if packet is not None:
            f['packetNumber'][fc] = self.receivedPackets
        f.flush()


    def fillBuffer(self, startIdx, amountToFill, readings):
        if startIdx + amountToFill <= 1024:
            self.pressure[startIdx:startIdx+amountToFill] = np.array(readings[:amountToFill])
        else:
            firstSize = 1024 - startIdx
            secondSize = amountToFill- firstSize
            self.pressure[startIdx:]=np.array(readings[:firstSize])
            self.pressure[:secondSize]=np.array(readings[firstSize:amountToFill])


    def processRow(self, startIdx,readings, packet=None):
        if packet is not None:
            self.receivedPackets[self.packetCount]=packet
            self.packetCount+=1
        if self.left_to_fill <= self.bufferSize:
            if self.left_to_fill > 0:
                self.fillBuffer(startIdx,self.left_to_fill,readings)
            ts = getUnixTimestamp()
            self.append_data(ts,self.pressure,packet)
            self.fc+=1
            self.packetCount = 0
            self.receivedPackets=np.zeros(self.maxPackets)
            remaining = self.bufferSize - self.left_to_fill
            self.fillBuffer((startIdx+self.left_to_fill)%1024, remaining, readings[self.left_to_fill:])
            self.left_to_fill = 1024-remaining
        else:
            self.fillBuffer(startIdx,self.bufferSize, readings)
            self.left_to_fill -= self.bufferSize
            
