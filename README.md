This directory contains source files for controlling the arduino motors.

In this directory is the motorControl arduino sketch folder. This folder contains one source file which
receives input the windows machine and moves to the orientation requested. It requires the standard arduino servo library.

The other source file is arduinoCommands.py which contains a very simple implementation of message sending using the pySerial library.
It sends a user input char over the COM4 port which in turn moves the pan and tilt bracket holding the kinect using the motorControl sketch.