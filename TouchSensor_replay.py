import numpy as np
import serial
import time
import h5py
from datetime import datetime
from datetime import date
import cv2


def tactile_reading(path):
    f = h5py.File(path, 'r')
    fc = f['frame_count'][0]
    ts = np.array(f['ts'][:fc])
    pressure = np.array(f['pressure'][:fc]).astype(np.float32)

    return pressure, fc, ts

def viz_data(pressure, pressure_min, pressure_max, use_log):
    pressure = (pressure.astype(np.float32) - pressure_min) / (pressure_max - pressure_min)
    pressure = np.clip(pressure, 0, 1)
    if use_log:
        pressure = np.log(pressure + 1) / np.log(2.0)

    im = cv2.applyColorMap((np.clip(pressure, 0, 1) * 255).astype('uint8'), cv2.COLORMAP_JET)
    im = cv2.resize(im, (500, 500))
    return im

def main():
    path = 'recordings_1_1719253910.791302.hdf5'
    pressure, fc, ts = tactile_reading(path)
    print(pressure.shape)

    pressure_min = 0
    pressure_max = 3400
    use_log = True
    viz = True

    if viz:
        for i in range(pressure.shape[0]):
            im = viz_data(pressure[i], pressure_min, pressure_max, use_log)
            cv2.imshow('VizualizerTouch', im)
            if cv2.waitKey(1) & 0xff == 27:
                break

if __name__ == "__main__":
    main()