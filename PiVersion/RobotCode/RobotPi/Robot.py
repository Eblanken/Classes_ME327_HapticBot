#
# File: Robot.py
# ------------------------------
# Authors: Erick Blankenberg, Beck Goodloe, Josiah Clark
# Date: 5/23/2018
#
# Description:
#   Recieves directional information and transmits lidar data
#   and bumper states to teleoperator client. Uses UDP.
#

import struct
import rplidar
import msgpack
import msgpack_numpy as m
m.patch()
import socket
import serial
import signal
from serial.tools import list_ports
import numpy as np
import time

def signal_term_handler(signal, frame):
    lidarUnit.stop()
    lidarUnit.stop_motor()
    exit()

currentLoopCounter = 0
# Connects to microcontroller and lidar
portList     = list_ports.comports()
MCUUnit      = None
lidarUnit    = None
print("Seeking Peripherals")
print(portList)
#for currentPortIndex in range(0, len(portList)):
#currentPort = portList[currentPortIndex]
# May be Lidar
try:
    lidarUnit = rplidar.RPLidar('/dev/ttyUSB0')
    lidarUnit.stop()
    lidarUnit.stop_motor()
    lidarUnit.clean_input() # :(
    lidarUnit.start_motor()
    time.sleep(5) # Lidar likes to freak out if not already spinning
    signal.signal(signal.SIGTERM, signal_term_handler)
except rplidar.RPLidarException as e:
    lidarUnit = None
    print(e)

# May be MCU
try:
    serialPort = serial.Serial('/dev/ttyACM0', timeout = 0.1)
    serialPort.close()
    serialPort.open()
    serialPort.write(b'P')
    response = serialPort.readline().decode().strip()
    if(response == 'Here!'):
        MCUUnit = serialPort
        print("Connected to MCU!")
    else:
        print("Not the MCU, response: %s" % (response))
        serialPort.close()
except Exception as e:
    print("Unable to Open Port: %s" % (e))

if not MCUUnit:
    print("Connection to MCU Failed")
else:
    print("Connected to MCU!")
if not lidarUnit:
    print("Connection to Lidar Failed")

# Connects to operator
OPERATOR_IP = '25.11.192.247'
PORT        = 65432
print("Seeking Host")
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 256)
s.settimeout(0.0)

print("Operational!")
lastData = None
mostRecentAddr = None
while True:
    currentLoopCounter = currentLoopCounter + 1
    # Gets most recent microcontroller state
    latestMCUState = 0
    if MCUUnit:
        # Reads current state (passed back to main)
        MCUUnit.write(bytes("B", 'utf-8'))
        try:
            latestMCUState = int(MCUUnit.read().decode())
        except Exception as MCUDataException:
            print("MCU State Capture Failed: %s" % (MCUDataException))

    # Broadcasts lidar data and state data
    if lidarUnit:
        try:
            latestScan = next(lidarUnit.iter_scans())
            fullBuffer = np.asarray(latestScan) # format is quality, angle, distance
            dataBuffer = np.zeros(((fullBuffer.shape[0] + 1), 2)) # First row is [mcuState, 0]
            dataBuffer[1:,:] = fullBuffer[:, 1:3] # We do not care about quality
            dataBuffer[0, 0] = latestMCUState
            s.sendto(msgpack.packb(dataBuffer), (OPERATOR_IP, PORT))
        except Exception as e:
            print("Error : %s" % (e))

    # Gets most recent command
    try:
        data, client = s.recvfrom(1024)
        if data:
            inData = msgpack.unpackb(data)
            if (currentLoopCounter % 10 == 0):
                print("%d %d" % (inData[0], inData[1]))
            OPERATOR_IP, OPERATOR_PORT = client
            lastData = inData
        elif lastData:
            inData = lastData
        if MCUUnit:
            try:
                # Updates motor values
                MCUUnit.write(bytes("U%d %d" % (int(inData[0]), int(inData[1])), 'utf-8'))
            except Exception as mcue:
                print("Unable to write to MCU: %s" % (mcue))
    except Exception as e:
        pass
