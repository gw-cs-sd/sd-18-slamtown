from pykinect2 import PyKinectV2
from pykinect2.PyKinectV2 import *
from pykinect2 import PyKinectRuntime
from datetime import datetime
import numpy
import cv2

class KinectGrabber:
	def __init__(self):
		self.kinect = PyKinectRuntime.PyKinectRuntime(PyKinectV2.FrameSourceTypes_Depth | PyKinectV2.FrameSourceTypes_Color)

	def ColorRequest(self, display):
		frame = None
		filename = ''
		while(True):
			if (self.kinect.has_new_color_frame()):
				frame = self.kinect.get_last_color_frame()
				frame = frame.reshape((1080, 1920 ,4))		#height (rows), width (columns), channels (4) (
				#flip
				break

		if(display == True):	
			cv2.namedWindow('KINECT Image Stream', cv2.WINDOW_NORMAL)
			cv2.resizeWindow('KINECT Image Stream', 450,300)
			cv2.imshow('KINECT Image Stream', frame)

		filename = str(datetime.now().strftime('Color_Images/%H.%M.%S.%f')[:-3]) + '.png'
		cv2.imwrite(filename, frame)
		

		while True:
			cv2.waitKey()
			break
		return filename

	def Close(self):
		self.kinect.close()
		cv2.destroyAllWindows()

#mykinect = KinectGrabber()
#mykinect.ColorRequest(display = True, save = True)
#mykinect.Close()

