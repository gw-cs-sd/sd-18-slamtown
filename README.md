This is the main branch for the Distributed Climate Control project

By Dan Coen and Conor Sheridan

Contact: dancoen2@gmail.com / conorksheridan@gmail.com

Usage Instructions for running our code

Our project has only been tested with Windows 8.1 and 10. If using Linux, see config differences in arduinoCommands.py. Additionally, PyKinect2 requires the Windows Kinect SDK v2. There may be more changes necessary to make the code compatible. We are also using python 2.7.13


# Raspberry Pi Server Setup
## Dependencies: 

### Python 2.7
Should already be installed if on Raspbian

### Pylepton
Use Lepton3 branch (or other if using a different sensor)

[Github page](https://github.com/groupgets/pylepton)

### cv2 (opencv 3)
https://opencv.org/releases.html

or for pip install (may be updated version)

https://pypi.python.org/pypi/opencv-python

### numpy
`sudo apt-get install python-numpy`



(if you are using the same pi as us (raspi 3) you shouldn't need to change config settings)

* Change the raspi configuration settings to enable SPI, VNC, and SSH. Depending on what other sensors you are using
you may need to enable I2C. 

* Connect an ethernet cable from the Windows machine to the pi. NO CHANGES should need to be made to network preferences. 

* Use `PING raspberrypi` on windows machine to ensure the hostname is resolvable on your pc.  

* Install VNC viewer on your windows machine. 

* Connect to the pi on VNC viewer using hostname: raspberrypi   password: None

You can also use SSH instead of VNC viewer but VNC is useful if you need to make changes. 

* On the pi, navigate to where the project has been cloned. install the necessary dependencies and run `python ~/projects/sd-18-slamtown/src/server/thermal_server.py`

This should automatically configure a the local ip address of the pis eth0 interface and create the thermal image server on it. 

This should be all you need to do on the pi. We found this to be the easiest way to get the networking up and running 
(no internet sharing/bridge connections, etc.) if you need to commit changes you will have to connect the pi to the internet 
seperately or mess around with the other methods listed above.

To test if the machine is correctly receiving images, install dependencies and run 

`python client/thermal_client.py`



# Windows Machine Dependencies

If you are using the same UP board as us, then everything here should already be present

## Python 2.7 verion of 32-bit Anaconda
[Download the 32-bit installer here](https://www.anaconda.com/download/)

## pip installs through Anaconda Prompt

`pip install pykinect2`
For PyKinect2

`pip install comtypes`
For PyKinect2

`pip install numpy`
For various things

`pip install imutils`
Used for image reshaping

`pip install opencv-python`
`pip install opencv-contrib-python`
This is required to allow usage of keypoint detectors SIFT and SURF which are utilized in panorama stitching, and were removed in OpenCV 3

## Kinect for Windows SDK 2.0
[Download the SDK here](https://www.microsoft.com/en-us/download/details.aspx?id=44561) 

The Kinect must be plugged into a USB 3.0 port in order to work.

To test to make sure opencv and pykinect are installed, uncomment the bottom of src/kinect/PyKinectFrameGrabber.py and run.


## Serial for Arduino

[Pyserial Download](https://pypi.python.org/pypi/pyserial/2.7)
The com port in src/arduino/arduinoCommands.py must be configured differently for linux.


# Arduino Setup

## Dependencies: internal Servo lib (Servo.h)

Attach tilt servo to shield servo pin 1 and pan servo to shield pins 2. When doing so, the black wire should be on the side nearest
to the power/usb input.

If you're using a laptop, you can probably get away with not using the shield as well as the external power. The servos typically aren't drawing as much power as they are capible of, so the USB could actually be enough. However, if you are using the UP board, the external power is very necessary, as it does not provide sufficient power.



Once everything is set up and connected, you should be able to run `python integrationtest.py` and have everything work. Feel free to send us an email if you have any questions!






