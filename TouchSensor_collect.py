import numpy as np
import serial
import time
import h5py
from matplotlib import pyplot as plt
from matplotlib import animation
from datetime import datetime
from datetime import date
import cv2
import struct
import threading

def getUnixTimestamp():
    return np.datetime64(datetime.now()).astype(np.int64) / 1e6  # unix TS in secs and microsecs
pressureReadouts = np.zeros((2,32,32))
stop_capture_event = threading.Event()

def readPressure(ser:serial, axes, pressure_min,pressure_max, use_log, f, init, block_size, fc, ts):
    global pressureReadouts
    stopStr = bytes('wr','utf-8')
    rowsRead = 0
    while not stop_capture_event.is_set():
        byteData = ser.read_until(expected=stopStr)[:-2]
        if len(byteData) == 66:
            format_string = 'bb' + 'H' * 32  # 'b' for int8_t, 'H' for uint16_t
            tupData = struct.unpack(format_string,byteData)
            sendId = tupData[0]
            yIndex = tupData[1]
            sensorReadings = tupData[2:]
            pressureReadouts[sendId-1,yIndex,:]=np.array(sensorReadings)
            if yIndex==31:
                ts = getUnixTimestamp()
                append_data(ts,pressureReadouts[sendId-1,:,:])
            # viz_data(axes, pressureReadouts,pressure_min, pressure_max, use_log)
    #estimatedFrameRate = (rowsRead/32) * 1/(end-start)
    # print(len(buffer))
    # estimatedFrameRate = len(buffer)/(68*32)*1/(end-start)
    # print("Estimated Frame Rate: ", estimatedFrameRate)




    # # Receive data
    # w, h = 32, 32
    # length = 2 * w * h
    # input_string = ser.read(length)
    # x = np.frombuffer(input_string, dtype=np.uint8).astype(np.uint16)
    # if not len(input_string) == length:
    #     return None

    # x = x[0::2] * 32 + x[1::2]
    # x = x.reshape(h, w).transpose(1, 0)

    # return x




def getHeatmap(pressureArr,pressure_min, pressure_max, use_log):
    pressure = (pressureArr.astype(np.float32) - pressure_min) / (pressure_max - pressure_min)
    pressure = np.clip(pressure, 0, 1)
    if use_log:
        pressure = np.log(pressure + 1) / np.log(2.0)
    im = cv2.applyColorMap((np.clip(pressure, 0, 1) * 255).astype('uint8'), cv2.COLORMAP_JET)
    im = cv2.resize(im, (500, 500)) 
    return im


def viz_data(axes, pressureReadouts, pressure_min, pressure_max, use_log):
    left = getHeatmap(pressureReadouts[0,:,:],pressure_min,pressure_max,use_log)
    right = getHeatmap(pressureReadouts[1,:,:],pressure_min,pressure_max,use_log)
    axes[0].imshow(left)
    axes[1].imshow(right)
    plt.draw()
    plt.pause(0.05)  # Pause to update the plot







def main():
    path = './recordings_5k' +str(time.time())+ '.hdf5'
    f = h5py.File(path, 'w')

    block_size = 1024
    fc = 0
    pressure_min = 800
    pressure_max = 3400
    use_log = True
    viz = True

    port = 'COM7' #update serial port
    baudrate = 250000

    ser = serial.Serial(port, baudrate=baudrate, timeout=1.0)
    init = False
    def append_data(f, ts, reading):
        if not init:
            sz = [block_size, ]
            maxShape = sz.copy()
            maxShape[0] = None
            f.create_dataset('frame_count', (1,), dtype=np.uint32)
            f.create_dataset('ts', tuple([block_size, ]), maxshape=tuple(maxShape),dtype=ts.dtype, chunks=True)
            maxshapePressure = [None,32,32]
            f.create_dataset('pressure', tuple([block_size, 32, 32]), maxshape=tuple(maxshapePressure),dtype=np.int32, chunks=True)

        # Check size
        oldSize = f['ts'].shape[0]
        if oldSize == fc:
            newSize = oldSize + block_size
            f['ts'].resize(newSize, axis=0)
            f['pressure'].resize(newSize, axis=0)

        f['frame_count'][0] = fc
        f['ts'][fc] = ts
        f['pressure'][fc] = reading.reshape(1,32,32)

        f.flush()
        
    assert ser.is_open, 'Failed to open COM port!'

    stopStr = bytes('wr','utf-8')
    rowsRead = 0
    while not stop_capture_event.is_set():
        byteData = ser.read_until(expected=stopStr)[:-2]
        if len(byteData) == 66:
            format_string = 'bb' + 'H' * 32  # 'b' for int8_t, 'H' for uint16_t
            tupData = struct.unpack(format_string,byteData)
            sendId = tupData[0]
            yIndex = tupData[1]
            sensorReadings = tupData[2:]
            pressureReadouts[sendId-1,yIndex,:]=np.array(sensorReadings)
            if yIndex==31:
                ts = getUnixTimestamp()
                print(ts)
                append_data(f,ts,pressureReadouts[sendId-1,:,:])
                fc+=1
                if not init:
                    init = True
    # fig, axes = plt.subplots(1, 2)  # Create subplots for each heatmap
    # for i in range(2):
    #     if i == 0:
    #         axes[i].set_title('Left Hand')
    #     else:
    #         axes[i].set_title('Right Hand')
    #     axes[i].axis('off')
    # plt.tight_layout()
    # cax1 = axes[0].imshow(np.zeros((32, 32)), cmap='viridis', vmin=pressure_min, vmax=pressure_max)
    # plt.colorbar(cax1)

    # def update(frame):
    #     stopStr = bytes('wr','utf-8')
    #     rowsRead = 0
    #     try:
    #         while rowsRead < 32:
    #             byteData = ser.read_until(expected=stopStr)[:-2]
    #             if len(byteData) == 66:
    #                 format_string = 'bb' + 'H' * 32  # 'b' for int8_t, 'H' for uint16_t
    #                 tupData = struct.unpack(format_string,byteData)
    #                 sendId = tupData[0]
    #                 yIndex = tupData[1]
    #                 sensorReadings = tupData[2:]
    #                 pressureReadouts[sendId-1,yIndex,:]=np.array(sensorReadings)
    #                 rowsRead+=1
    #         cax1.set_array(pressureReadouts[0,:,:])
    #     except serial.SerialException as e:
    #         print(f'Serial error: {e}')
    #     except Exception as e:
    #         print(f'Error: {e}')
    #     return cax1
    
    # ani = animation.FuncAnimation(fig, update, interval=1000/10)  # ~60 FPS
    # plt.show()

    # Close the serial port
    ser.close()




    # pressure_thread = threading.Thread(target=readPressure, args=(ser,))
    # visualization_thread = threading.Thread(target=viz_data,args=(pressure_min,pressure_max,use_log))
    # init = False
    # pressure_thread.start()
    # visualization_thread.start()
    # # Wait for visualization thread to finish

    # visualization_thread.join()

    # # Wait for pressure reading thread to finish
    # pressure_thread.join()

    # Close OpenCV windows
    cv2.destroyAllWindows()



        # fc += 1
        # ts = getUnixTimestamp()
        # reading = readPressure(ser)

        # if reading is not None:
        #     append_data(f, init, block_size, fc, ts, reading)
        #     init = True


if __name__ == "__main__":
    main()