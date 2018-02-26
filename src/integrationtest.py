from kinect.PyKinectFrameGrabber import KinectGrabber
from client.thermal_client import ThermalClient
import socket
import serial
import msvcrt
import sys

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
for i in range (0,4):
    c =  msvcrt.getch()                         #enter a char (0 through input 3) and send to arduino serial port
    ser.write(bytes(c))                         #encode and send as byte
    myKinect.ColorRequest(display = True, save = True)
    myClient.ImageRequest(display = True, save = True)
ser.close()
myClient.Close()
myKinect.Close()
