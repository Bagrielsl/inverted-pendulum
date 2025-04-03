# import serial

# def readserial(comport, baudrate):
#     t = 115200.0
#     q = 1/t
#     ser = serial.Serial(comport, baudrate, timeout=0.01)

#     while True:
#         data = ser.readline().decode().strip()
#         if data:
#             print(data)

# readserial('ttyUSB0', 115200)
import os
from time import sleep 
t = 10000#115200.0
q = 1/t
while True:
    os.system('cat /dev/ttyUSB0')
    #print('--')
    sleep(q)