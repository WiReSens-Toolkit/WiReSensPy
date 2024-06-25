import serial
import time
import keyboard
port = 'COM7' #update serial port
baudrate = 250000
ser = serial.Serial(port, baudrate=baudrate, timeout=1.0)
buffer=b''
start=time.time()
while True:
    waiting = ser.in_waiting  # find num of bytes currently waiting in hardware
    buffer += ser.read(waiting)
    if keyboard.is_pressed("enter"):
        break

end = time.time()
print(len(buffer)/(68*32)*1/(end-start))