import h5py
import numpy as np
def tactile_reading(path):
    f = h5py.File(path, 'r')
    fc = f['frame_count'][0]
    ts = np.array(f['ts'][:fc])
    pressure = np.array(f['pressure'][:fc]).astype(np.float32)

    return pressure, fc, ts

def find_closest_index(array, value):
    index = (np.abs(array - value)).argmin()
    return index, array[index]