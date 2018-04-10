This is the main branch for the Distributed Climate Control project

By Dan Coen and Conor Sheridan

contact: dancoen2@gmail.com / conorksheridan@gmail.com

Usage Instructions for running our code

Our project has only been used with Windows 10. If using Linux, see config differences in arduinoCommands.py. there may be more changes necessary to make the code compatible. We are also using python 2.7.13


#################   Raspberrypi server setup  #################

Dependencies: 

Python 2.7

Pylepton
use Lepton3 branch (or other if using a different sensor)
https://github.com/groupgets/pylepton

cv2 (opencv 3)
https://opencv.org/releases.html
or for pip install (may be updated version)
https://pypi.python.org/pypi/opencv-python

numpy
sudo apt-get install python-numpy



(if you are using the same pi as us (raspi 3) you shouldn't need to change config settings)

Change the raspi configuration settings to enable SPI, VNC, and SSH. Depending on what other sensors you are using
you may need to enable I2C. 

Connect an ethernet cable from the Windows machine to the pi. NO CHANGES should need to be made to network preferences. 

use "PING raspberrypi" on windows machine to ensure the hostname is resolvable on your pc.  


Install VNC viewer on your windows machine. 
Connect to the pi on VNC viewer using hostname: raspberry   password: pi
You can also use SSH instead of VNC viewer but VNC is useful if you need to make changes. 

on the pi, navigate to where the project has been cloned. install the necessary dependencies and run 

python /src/server/thermal_server.py

This should automatically configure a the local ip address of the pis eth0 interface and create the thermal image server on it. 
This should be all you need to do on the pi. We found this to be the easiest way to get the networking up and running 
(no internet sharing/bridge connections, etc.) if you need to commit changes you will have to connect the pi to the internet 
seperately or mess around with the other methods listed above.

To test if the machine is correctly receiving images, install dependencies and run 

python client/thermal_client.py



#################   Windows: Kinect V2  #################

Dependencies:

Pykinect
https://github.com/Microsoft/PTVS/wiki/PyKinect

KinectSDK v2
https://www.microsoft.com/en-us/download/details.aspx?id=44561

Note: you don't need to install pygame to run our code, we use opencv to display images instead of pygame/nui. 

In order to make use of the pyKinect library, we are running our code using 32 bit Anaconda (still python 2.7). You should install / make dependies available and run code in the anaconda prompt or a conda environment. You will encounter errors when not using 32bit anaconda.

The Kinect must be plugged into a USB 3.0 port in order to work.

To test to make sure opencv and pykinect are installed, uncomment the bottom of src/kinect/PyKinectFrameGrabber.py and run.


#################   Windows: Serial for Arduino #################

Dependencies:

Pyserial
https://pypi.python.org/pypi/pyserial/2.7

the com port in src/arduino/arduinoCommands.py must be configured differently for linux.


#################  Arduino Setup #################

Dependencies: internal Servo lib (Servo.h)

attach tilt servo to shield servo pin 1 and pan servo to shield pins 2. When doing so, the Black wire shoul be on the side nearest
to the power/usb input.

You can probably get away with not using the shield as well as the external power. the servos typically aren't drawing as much power as they are capible of, so the USB could actually be enough. Additionally, the external power as is may be unsafe for the arduino. 






