# Sensor Fusion: RGBD-T #

A complete library for creating RGBD-T models using data from multiple sensors. 

### Hardware ###

* Raspberry PI B+ (or similar)
* Flir Lepton V2 or V3
* MLX90621
* Microsoft Kinect V2

### Software ###

* Library modules:
	* Thermal Sensor Interface
		* Lepton grabber
		* Lepton temperature calibration
	* 3D Sensor Interface
		* Kinect grabber
	* Sensor Fusion Interface
		* Sensors sync
		* Sensors registration (calibration)
	* Image Processing library
		* Image Utils: tools for image creation and conversion (interface between our own structures and 
OpenCV structures)
	* Thermal comfort measurement
		* KeyPatch: module used to detect the key-patches on the human body
	* MessagePassing
		* PublisherUtils: tools to send specific messages
		* SubscriberUtils: tools to receive specific messages

* Applications:
	* ThermalViewer2D: simple app to visualize the raw thermal(T) stream
	* Viewer3D: simple app to visualize the raw RGB and D stream
	* Viewer5D: simple app to visualize the raw RGBD and T stream
	* ViewerFusion: simple app to visualize the fused RGBD-T stream
	* ViewerSkeleton: simple app to visualize the body skeleton detection
	* ViewerKeyPatch: simple app to visualize the key-patches detection on the human body
	* ThermalCalibration: Lepton thermal calibration module
	* DatasetRecorder: collecting new datasets using the following 3 apps:
		* DatasetPublisher: streams raw data from sensors
		* DatasetSubscriber: grabs raw data stream and save it to disk
		* ThermalModelPublisher: streams thermal models from live stream or offline dataset 
	* DatasetPlayer: allows you to play offline a previously recorded dataset
		
### Dependencies ###

* Windows Machine (for TCP/IP communication with the Raspberry Pi, and Kinect SDK)
* Microsoft Kinect SDK v2.X (Kinect SDK V2.0 tested)
* OpenCV 3.X.X (OpenCV 3.1.0 tested)
* Boost 1.6X.X (Boost 1.60.0 tested)
* ZMQ 4.X.X (ZMQ 4.1.0 tested)
* For Raspberry Pi server: bcm2835 library, and Lepton SDK

### Useful links ###

* https://github.com/groupgets/pylepton
* https://github.com/groupgets/LeptonModule
* https://github.com/maxritter/DIY-Thermocam
* https://groups.google.com/forum/#!forum/flir-lepton