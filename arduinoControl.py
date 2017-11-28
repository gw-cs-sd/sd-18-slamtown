from firmata import *

a = Arduino('com4')
a.pin_mode(2,OUTPUT)

while TRUE:
	a.parse()
	print a.analog_read(2)