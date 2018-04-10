from __future__ import print_function
from kinect.PyKinectFrameGrabber import KinectGrabber
from client.thermal_client import ThermalClient
import socket
import serial
import msvcrt
import sys
import time
import os
# Python 2/3 compatibility

import numpy as np
import cv2 as cv
import sys
from glob import glob
import itertools as it


def inside(r, q):
    rx, ry, rw, rh = r
    qx, qy, qw, qh = q
    return rx > qx and ry > qy and rx + rw < qx + qw and ry + rh < qy + qh


def draw_detections(img, rects, thickness = 1):
    for x, y, w, h in rects:
        # the HOG detector returns slightly larger rectangles than the real objects.
        # so we slightly shrink the rectangles to get a nicer output.
        pad_w, pad_h = int(0.15*w), int(0.05*h)
        cv.rectangle(img, (x+pad_w, y+pad_h), (x+w-pad_w, y+h-pad_h), (0, 255, 0), thickness)

def hog(tryimage):
    hog = cv.HOGDescriptor()
    hog.setSVMDetector( cv.HOGDescriptor_getDefaultPeopleDetector() )
    


    #default = ['../data/basketball2.png '] if tryimage == 0 else []

    try:
        img = cv.imread(tryimage)
        if img is None:
            print('Failed to load image file:', tryimage)
            return
    except:
        print('loading error')
        return

    found, w = hog.detectMultiScale(img, winStride=(16,16), padding=(32,32), scale=1.05)
    found_filtered = []
    for ri, r in enumerate(found):
        for qi, q in enumerate(found):
            if ri != qi and inside(r, q):
                break
        else:
            found_filtered.append(r)
    draw_detections(img, found)
    draw_detections(img, found_filtered, 3)
    print('%d (%d) found' % (len(found_filtered), len(found)))
    cv.imshow('img', img)
    ch = cv.waitKey()
    if ch == 27:
        cv.destroyAllWindows()

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
    #cv.imshow("color", colorimg)
    #cv.waitKey(0)
    #hog(colorfile)
    thermalfile = myClient.ImageRequest(display = False)
    thermalimg = cv.imread(thermalfile)
    height, width = colorimg.shape[:2]
    thermalimg = cv.resize(thermalimg,(width, height), interpolation = cv.INTER_LINEAR)
    cv.imwrite(thermalfile, thermalimg)
    #cv.imshow("thermal", thermalimg)
    #cv.waitKey(0)
ser.close()
myClient.Close()
myKinect.Close()
