from flask import Flask
import json
from flask_socketio import SocketIO, emit
from flask_cors import CORS
import time
import threading
import sys
import os

app = Flask(__name__)
CORS(app, resources={r"/*": {"origins": "*"}}) 
socketio = SocketIO(app, cors_allowed_origins="*")



def update_sensors(allSensors):
    with app.app_context():
        sensors={}
        while True:
            for i in range(len(allSensors)):
                sensors[allSensors[i].id]= (allSensors[i].pressure.reshape(allSensors[i].selWires,allSensors[i].readWires)).tolist()
            jsonSensors = json.dumps(sensors)
            socketio.emit('sensor_data', jsonSensors)
            time.sleep(1/30)  # 30 FPS

@app.route('/api/python')
def index():
    return "WebSocket server is running..."

if __name__ == '__main__':
    print("Hello main")
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