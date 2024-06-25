import serial
from matplotlib import pyplot as plt
import numpy as np
import struct
import time
import h5py
from matplotlib import animation


def append_data(f, init, block_size, fc, ts, reading):
    if not init:
        sz = [block_size, ]
        maxShape = sz.copy()
        maxShape[0] = None
        f.create_dataset('frame_count', (1,), dtype=np.uint32)
        f.create_dataset('ts', tuple([block_size, ]), dtype=ts.dtype, chunks=True)
        f.create_dataset('pressure', tuple([block_size, 32, 32]), dtype=np.int32, chunks=True)

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

def main():
    numRows = 32
    numCols = 32
    pressure_min = 0
    pressure_max = 3400
    pressureReadouts = np.zeros((2,numRows,numCols))
    port = 'COM7' #update serial port
    baudrate = 250000

    path = './recordings' + str(time.time()) + '.hdf5'
    f = h5py.File(path, 'w')

    block_size = 1024
    fc = 0
    pressure_min = 800
    pressure_max = 3400
    use_log = True
    viz = True
    init = False

    ser = serial.Serial(port, baudrate=baudrate, timeout=1.0)
    assert ser.is_open, 'Failed to open COM port!'
    fig, axes = plt.subplots(1, 2)  # Create subplots for each heatmap
    for i in range(2):
        if i == 0:
            axes[i].set_title('Left Hand')
        else:
            axes[i].set_title('Right Hand')
        axes[i].axis('off')
    plt.tight_layout()
    cax1 = axes[0].imshow(np.zeros((numRows, numCols)), cmap='viridis', vmin=pressure_min, vmax=pressure_max)
    plt.colorbar(cax1)

    def update(frame):
        stopStr = bytes('wr','utf-8')
        rowsRead = 0
        try:
            while rowsRead < numRows:
                byteData = ser.read_until(expected=stopStr)[:-2]
                expectedLength = 2+2*numCols
                if len(byteData) == expectedLength:
                    format_string = '=bb' + 'H' * numCols  # 'b' for int8_t, 'H' for uint16_t
                    print(struct.calcsize(format_string))
                    tupData = struct.unpack(format_string,byteData)
                    sendId = tupData[0]
                    yIndex = tupData[1]
                    sensorReadings = tupData[2:]
                    pressureReadouts[sendId-1,yIndex,:]=np.array(sensorReadings)
                    rowsRead+=1

            cax1.set_array(pressureReadouts[0,:,:])
            print(np.min(pressureReadouts[0,:,:]))
        except serial.SerialException as e:
            print(f'Serial error: {e}')
        except Exception as e:
            print(f'Error: {e}')
        return cax1
    
    ani = animation.FuncAnimation(fig, update, interval=1000/60)  # ~60 FPS
    plt.show()

    # Close the serial port
    ser.close()


if __name__ == "__main__":
    main()