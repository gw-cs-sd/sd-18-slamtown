Currently this branch has functioning code for the server (windows) to receive thermal images hosted by the raspberry pi server.

for the server, the pylepton library should be installed and in the /server directory. 

The raspberry ip address will change unless the user has configured a static ip address, so both the server and client files must
be updated with the ip address of the raspi in order to connect the socket.

Usage:

first, start the server (python 2 or 3):

python thermal_server.py

Then, start the client (python 2, uses opencv python build)

python thermal_client.py

The client and server should write to std err to display if they are working correctly. The size of the thermal image being sent to the client
will be written in bytes as well as how much has been received thus far. The image should be written into the client/Images directory and then displayed in a window on the client. Press any key to continue the sample program. This test will be written to a seperate file in the future.
=======
This directory contains source files for controlling the arduino motors.

In this directory is the motorControl arduino sketch folder. This folder contains one source file which
receives input the windows machine and moves to the orientation requested. It requires the standard arduino servo library.

The other source file is arduinoCommands.py which contains a very simple implementation of message sending using the pySerial library.
It sends a user input char over the COM4 port which in turn moves the pan and tilt bracket holding the kinect using the motorControl sketch.
