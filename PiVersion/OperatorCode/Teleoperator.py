#
# File: Teleoperator.py
# ------------------------------
# Authors: Erick Blankenberg, Beck Goodloe, Josiah Clark
# Date: 5/23/2018
#
# Description:
#   Transmits control data and recieves lidar packet
#
#   Based off of the UDP communication tutorial at: https://www.studytonight.com/network-programming-in-python/working-with-udp-sockets
#
#   Notes:
#    - Make sure for the lidar data format from the pi on the bot that the first row
#      consists of two unsigned 16 bit integers (uint16_t) as these are interpreted
#      as binary data.
#

import serial
from serial.tools import list_ports

import sys
import numpy as np
import msgpack
import msgpack_numpy as m
m.patch()
import socket
import pygame
import math
import time

PI = 3.1415926535
OPERATOR_IP = "25.11.192.247"
PORT        = 65432

# Sets up visuals
pygame.init()
fpsCounter = pygame.time.Clock()
screen = pygame.display.set_mode((640, 480))
pygame.display.set_caption('Teleoperator Client')
screen.fill((0, 0, 0))
pygame.mouse.set_visible(False)
pygame.display.update()
screen.fill((0, 125, 125))
pygame.display.update()

# Connects to hapkit
print("Attempting to connect to haptic system...")
lastKeyboardKeys = None
try:
    fiveBarLinkage = serial.Serial('/dev/cu.usbserial-A603XVSM', baudrate = 115200, timeout = 0.1)
except Exception as e:
    fiveBarLinkage = None
    print('Failed to connect to haptic IO: %s' % (e))
if fiveBarLinkage:
    print("Connected to graphkit")
else:
    print("No graphkit found, defaulting to keyboard.")

# Connects to robot
try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    #s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024)
except socket.error, msg :
    print 'Failed to create socket. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()
try:
    s.bind(('0.0.0.0', PORT))
except socket.error , msg:
    print 'Bind failed. Error: ' + str(msg[0]) + ': ' + msg[1]
    sys.exit()
s.settimeout(0.0)

# Loop value
lastClient            = None
newAngle              = 0
newMagnitude          = 0
lastAngle             = 0.0
lastDistanceMagnitude = 0.0
lastData              = None

lidarDataIntervals    = np.arange(0, 360, 2)
LIDAR_MAX             = 1000.0 # mm

#Variables for low pass filtering of LIDAR data
bin_size = 7
lidar_movmean = np.zeros((180,bin_size))
currentFilterIndex = 0

# Colors
COLOR_CLIENTAVATAR          = pygame.Color("#FF420E")
COLOR_CLIENTAVATARPOINTER   = pygame.Color("#F98866")
COLOR_ENVIRONMENT           = pygame.Color("#89DA59")
COLOR_AVERAGEPOINTER        = pygame.Color("#80BD9E")

DRAW_LINETHRESHOLD          = 500
DRAW_MODE_LINE              = 1 #None
DRAW_AVATAR_CENTERRADIUS    = 10
DRAW_AVATAR_BUMPERRADIUS    = 12
DRAW_AVATAR_BUMPERTHICKNESS = 4
DRAW_AVATAR_ENVCURSORWIDTH  = 10
DRAW_AVATAR_HAPCURSORWIDTH  = 20

outData = np.array([0.0, 0.0]) # Format is (xVal out of 1024, yVal out of 1024)
def processLidarData(rawData, lidar_movmean, currentFilterIndex, currentOrientation):
    try:
        # Updates monitor
        screen.fill((0, 0, 0))
        #data = rawData[1:, :]
        #data[(data[:,1] > LIDAR_MAX), 1] = LIDAR_MAX

        sampleData = rawData[1:, :] # First row used for binary packed values, interpret each one as a byte

        # Fills in missing lidar data
        angleIncrements        = sampleData[1:, 0] - sampleData[:-1, 0] # Gets differences
        smallestAngleIncrement = np.min(abs(angleIncrements)) # Gets bottom of permissible range
        largestAngleIncrement  = smallestAngleIncrement * 1.5 # Gets top of range
        # Fills in missing data
        currentRow = 0
        lastAngle  = 0
        data = np.zeros((max(lidarDataIntervals.shape), 2))
        data[:, 0] = lidarDataIntervals
        data[:, 1] = LIDAR_MAX
        for currentIndex in range(0, max(data.shape)):
            currentAngle = data[currentIndex, 0]
            closestRawIndex = (np.abs(sampleData[:, 0] - currentAngle)).argmin()
            if (abs(sampleData[closestRawIndex, 0] - currentAngle) < largestAngleIncrement):
                data[currentIndex, 1] = rawData[closestRawIndex, 1]
            if(data[currentIndex, 1] > LIDAR_MAX) or (data[currentIndex, 1] < 25):
                data[currentIndex, 1] = LIDAR_MAX

        # Filters
        lidar_movmean[:, currentFilterIndex] = data[:, 1]
        currentFilterIndex = np.remainder(currentFilterIndex + 1, bin_size)

        data[:,1] = np.average(lidar_movmean, 1)
        #print(np.mi(data[:,1]))
        auxillaryData = rawData[0, :]
        longestSize = max(data.shape)
        radians = np.deg2rad(data[:, 0])

        # Gets average of lidar points in direction of pointer and hapkit pointer
        targetIndex = (np.abs(data[:, 0] - newAngle)).argmin()
        lastDistanceMagnitude = np.average(np.take(data[:, 1], np.array([targetIndex - 1, targetIndex, targetIndex + 1])))
        averagePoint = (320 - int(((lastDistanceMagnitude * np.cos(np.deg2rad(newAngle))) / LIDAR_MAX) * 119.0), 240 + int((((-lastDistanceMagnitude * np.sin(np.deg2rad(newAngle))) / LIDAR_MAX) * 199.0)))
        hapkitMagnitude = float(newMagnitude) * (1024.0 / 500)
        hapkitPoint  = (320 - int(((hapkitMagnitude * np.cos(np.deg2rad(newAngle))) / 2000.0) * 119.0), 240 + int((((-hapkitMagnitude * np.sin(np.deg2rad(newAngle))) / 2000.0) * 199.0)))
        # Draws circle at average of points in environment in desired movement direction
        pygame.draw.circle(screen, COLOR_AVERAGEPOINTER, averagePoint, DRAW_AVATAR_ENVCURSORWIDTH)
        # Draws hapkit magnitude
        #pygame.draw.line(screen, COLOR_CLIENTAVATARPOINTER, (320, 240), hapkitPoint, DRAW_AVATAR_HAPCURSORWIDTH)
        # Draws cursor
        pygame.draw.circle(screen, COLOR_CLIENTAVATAR, (320, 240), DRAW_AVATAR_CENTERRADIUS)


        # Draws current bumper state
        bumperVals = int(auxillaryData[0])
        maskVal    = int(1)
        # > Top Bumper
        if currentOrientation == 1:
            pygame.draw.line(screen, COLOR_CLIENTAVATAR, (320 - DRAW_AVATAR_BUMPERRADIUS, 240 - (DRAW_AVATAR_BUMPERRADIUS + 5)), (320 + DRAW_AVATAR_BUMPERRADIUS, 240 - (DRAW_AVATAR_BUMPERRADIUS + 5)), DRAW_AVATAR_BUMPERTHICKNESS)
        # > Bottom Bumper
        if currentOrientation == 2:
            pygame.draw.line(screen, COLOR_CLIENTAVATAR, (320 - DRAW_AVATAR_BUMPERRADIUS, 240 + (DRAW_AVATAR_BUMPERRADIUS + 5)), (320 + DRAW_AVATAR_BUMPERRADIUS, 240 + (DRAW_AVATAR_BUMPERRADIUS + 5)), DRAW_AVATAR_BUMPERTHICKNESS)
        # > Left Bumper
        if currentOrientation == 3:
            pygame.draw.line(screen, COLOR_CLIENTAVATAR, (320 - (DRAW_AVATAR_BUMPERRADIUS + 5), 240 + DRAW_AVATAR_BUMPERRADIUS), (320 - (DRAW_AVATAR_BUMPERRADIUS + 5), 240 - DRAW_AVATAR_BUMPERRADIUS), DRAW_AVATAR_BUMPERTHICKNESS)
        # > Right Bumper
        if currentOrientation == 4:
            pygame.draw.line(screen, COLOR_CLIENTAVATAR, (320 + (DRAW_AVATAR_BUMPERRADIUS + 5), 240 + DRAW_AVATAR_BUMPERRADIUS), (320 + (DRAW_AVATAR_BUMPERRADIUS + 5), 240 - DRAW_AVATAR_BUMPERRADIUS), DRAW_AVATAR_BUMPERTHICKNESS)


        # Updates hapkit if possible
        if fiveBarLinkage is not None:
            #print(lastDistanceMagnitude)
            stiffnessCoefficient = np.interp(np.asarray((lastDistanceMagnitude)), np.asarray((0, LIDAR_MAX)), np.asarray((0, 1024)))
            fiveBarLinkage.write(b"%d" % (stiffnessCoefficient))
            #print("Wrote Stiffnes %d" % (stiffnessCoefficient))

        # Plots lidar data
        x = -data[:, 1] * np.cos(radians)
        y = -data[:, 1] * np.sin(radians)
        oldPoint = None
        firstPoint = None
        for currentIndex in range(0, longestSize):
            point = (320 + int((x[currentIndex] / LIDAR_MAX) * 119.0), 240 + int(((y[currentIndex] / LIDAR_MAX) * 199.0)))
            if DRAW_MODE_LINE is None:
                screen.set_at(point, pygame.Color(255, 255, 0))
            else:
                if firstPoint is None:
                    firstPoint = point
                elif (np.linalg.norm(np.asarray(point) - np.asarray(oldPoint)) < DRAW_LINETHRESHOLD): # Dont want to draw lines too large
                    pygame.draw.line(screen, COLOR_ENVIRONMENT, oldPoint, point)
            oldPoint = point
        if DRAW_MODE_LINE is not None and (np.linalg.norm(np.asarray(firstPoint) - np.asarray(oldPoint)) < DRAW_LINETHRESHOLD):
            pygame.draw.line(screen, COLOR_ENVIRONMENT, oldPoint, firstPoint)
        # Updates
        pygame.display.update()
        pygame.event.get()
        fpsCounter.tick()
        #print(fpsCounter.get_fps())
    except Exception as e:
        print("Draw error: %s" % (e))
    return lidar_movmean, currentFilterIndex

# Main loop
lastLidarError   = None
lastHapkitError = None
endGame = False
while not endGame:
    # Receives lidar and other data
    try:
        inData, lastClient = s.recvfrom(4096)
        if inData is not None:
            data = msgpack.unpackb(inData).astype(float) # about 1000 bytes for 10
            #print("Current %d, %d" % (data.shape[0], data.shape[1]))
            lidar_movmean, currentFilterIndex = processLidarData(data, lidar_movmean, currentFilterIndex, currentOrientation) # Remember that first row is packed binary data
        else:
            print("No Data")
    except Exception as e:
        if lastLidarError is None and not (lastLidarError == e):
            lastLidarError = e
            print("Lidar failure: ", e)

    # Gets direction from hapkit
    """
    if fiveBarLinkage:
        try:
            # Gets IO from dual hapkit
            newData = fiveBarLinkage.readline().split()
            # Gets hapkit direction
            outData[0] = float(newData[1]) # X
            outData[1] = -float(newData[0]) # Y
            #np.clip(outData, -1024.0, 1024.0, outData)
            # Gets plotting angle and magnitude
            newMagnitude = np.linalg.norm(outData)
            if outData[0] == 0 and outData[1] == 0:
                newAngle = 0
            else:
                newAngle = -np.degrees(np.arctan2(outData[0], outData[1])) + 180
        except Exception as e:
            if lastHapkitError is None and not (lastHapkitError == e):
                lastHapkitError = e
                print("Bad graphkit data: %s" % (e))
    else:
    """
    # Gets latest keyboard input
    pygame.event.pump()
    currentKeys = pygame.key.get_pressed()
    outData[0] = 0
    outData[1] = 0
    if(currentKeys[pygame.K_SPACE]):
        endGame = True
    if(currentKeys[pygame.K_UP]):
        #if not lastKeyboardKeys or not lastKeyboardKeys[pygame.K_UP]:
        #    print("Key Up")
        currentOrientation = 1
        outData[1] = 64
        newMagnitude = 1024
    if(currentKeys[pygame.K_DOWN]):
        #if not lastKeyboardKeys or not lastKeyboardKeys[pygame.K_DOWN]:
        #    print("Key Down")
        currentOrientation = 2
        outData[1] = -64
        newMagnitude = 1024
    if(currentKeys[pygame.K_LEFT]):
        #if not lastKeyboardKeys or not lastKeyboardKeys[pygame.K_LEFT]:
        #    print("Key Left")
        currentOrientation = 3
        outData[0] = -64
        newMagnitude = 1024
    if(currentKeys[pygame.K_RIGHT]):
        #if not lastKeyboardKeys or not lastKeyboardKeys[pygame.K_RIGHT]:
        #    print("Key Right")
        currentOrientation = 4
        outData[0] = 64
        newMagnitude = 1024
    if(currentKeys[pygame.K_d]):
        newAngle = newAngle + 0.0025
    if(currentKeys[pygame.K_a]):
        newAngle = newAngle - 0.0025
    newAngle = newAngle % 355
    lastKeyboardKeys = currentKeys

    # Updates robot movement
    outData = outData.astype(int)
    if lastClient is not None and (lastData is None or np.any(np.not_equal(outData, lastData))):
        try:
            #outData[2] = 0 # Set to one if you want to reset the time client side
            lastData = outData
            s.sendto(msgpack.packb(outData), lastClient)
            #print("Data: %d, %d" % (outData[0], outData[1]))
        except Exception as e:
            print("Not Sent: %s" % (e))

# Connection closed
conn.close()
