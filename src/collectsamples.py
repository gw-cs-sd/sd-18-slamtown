from __future__ import print_function
from kinect.PyKinectFrameGrabber import KinectGrabber
from client.thermal_client import ThermalClient
from panorama_stitching.panorama import Stitcher
import socket
import serial
import msvcrt
import sys
import time
import os
from datetime import datetime
# Python 2/3 compatibility

import numpy as np
import cv2 as cv
import sys
from glob import glob
import itertools as it

#TODO have either timer (or possible ack?) to know when servos are finished moving and cameras should take next picture
#one of image needs to be flipped (kinect is mirrored, should be kinect)

ser = serial.Serial(        #pyserial windows com4 configuration
    port = "COM4",
    baudrate = 115200,
    bytesize = serial.EIGHTBITS, 
    parity = serial.PARITY_NONE,
    stopbits = serial.STOPBITS_ONE, 
    timeout = 1,
    xonxoff = False,
    rtscts = True,
    dsrdtr = True,
    writeTimeout = 201
)

raspiIP = socket.gethostbyname('raspberrypi')
myClient = ThermalClient(raspiIP,22222)
myKinect = KinectGrabber()
ser.write(bytes(0))                         #encode and send as byte                              
while True:                                 #read char, if not i (current position) read another until at right spot
    c = ser.read()
    #print c
    if len(c) > 0:
        if c == str(0):
            break

#print "starting main loop\n"
time.sleep(1)
colorimages = []
thermalimages = []
while True:
    for i in range (0,4):
        ser.write(bytes(i))                         #encode and send as byte                              
        while True:                                 #read char, if not i (current position) read another until at right spot
            c = ser.read()
            #print c
            if len(c) > 0:
                if c == str(i):
                    break
        colorfile = myKinect.ColorRequest(display = False)
        colorimg = cv.imread(colorfile)
        colorimg = colorimg[100:980, 273:1647]
        colorimg = cv.flip( colorimg, 1 )
        cv.imwrite(colorfile, colorimg)
        thermalfile = myClient.ImageRequest(display = False)
        thermalimg = cv.imread(thermalfile)
        height, width = colorimg.shape[:2]
        thermalimg = cv.resize(thermalimg,(width, height), interpolation = cv.INTER_LINEAR)
        cv.imwrite(thermalfile, thermalimg)
    time.sleep(3)
