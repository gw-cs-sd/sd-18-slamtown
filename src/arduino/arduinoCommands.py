import serial
import msvcrt
import sys

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

while True:
    c =  msvcrt.getch()                         #enter a char (0 through input 3) and send to arduino serial port
    #ser.write(bytes(str(c),'utf-8'))           #encode and send as byte
    ser.write(bytes(c))
    print(ser.readline())
ser.close()