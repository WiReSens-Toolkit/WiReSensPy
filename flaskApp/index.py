from flask import Flask
import json
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import time
import threading
import sys
import os
from .soundModule import midiPlayer
from .stepCount import stepCountTracker
import numpy as np
import h5py
app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}}) 
socketio = SocketIO(app, cors_allowed_origins="*")
# thisMidiPlayer = midiPlayer()
thisStepCount = stepCountTracker()

def tactile_reading(path):
    f = h5py.File(path, 'r')
    fc = f['frame_count'][0]
    ts = np.array(f['ts'][:fc])
    pressure = np.array(f['pressure'][:fc]).astype(np.float32)

    return pressure, fc, ts

def replay_sensors( pressureDict, frameRate, numFrames):
    with app.app_context():
        for i in range(numFrames):
            sensors={}
            for j in pressureDict:
                pressure=pressureDict[j][i]
                # thisStepCount.checkStep(socketio,pressure)
                # thisMidiPlayer.play_sound(pressure, allSensors[i].id==2)
                sensors[j]= (pressure).tolist()
            jsonSensors = json.dumps(sensors)
            socketio.emit('sensor_data', jsonSensors)
            time.sleep(1/frameRate)  # 50 FPS



        





def update_sensors(allSensors):
    with app.app_context():
        sensors={}
        while True:
            for i in range(len(allSensors)):
                pressure=allSensors[i].pressure.reshape(allSensors[i].selWires,allSensors[i].readWires)
                thisStepCount.checkStep(socketio,pressure)
                # thisMidiPlayer.play_sound(pressure, allSensors[i].id==2)
                sensors[allSensors[i].id]= (pressure).tolist()
                
            jsonSensors = json.dumps(sensors)
            socketio.emit('sensor_data', jsonSensors)
            time.sleep(1/50)  # 50 FPS

def start_server():
    socketio.run(app, port=5328, debug=True, use_reloader=False)


@app.route('/api/python')
def index():
    return "WebSocket server is running..."

if __name__ == '__main__':
    # Get the parent directory and add it to sys.path
    parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
    sys.path.insert(0, parent_dir)
    from TouchSensorWireless import MultiProtocolReceiver
    myReceiver = MultiProtocolReceiver(os.path.join(parent_dir, "WiSensConfigClean.json"))
    threads=[]
    captureThread = threading.Thread(target=myReceiver.collectData)
    captureThread.start()
    threads.append(captureThread)
    vizthread = threading.Thread(target=update_sensors, args=(myReceiver.allSensors, ))
    vizthread.start()
    threads.append(vizthread)
    socketio.run(app, port=5328, debug=True, use_reloader=False)