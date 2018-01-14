import socket
import sys
import numpy as np
import cv2
from pylepton.Lepton3 import Lepton3


#from pylepton library
def capture(flip_v = False, device = "/dev/spidev0.0"):
  with Lepton3(device) as l:
    a,_ = l.capture()
  if flip_v:
    cv2.flip(a,0,a)
  cv2.normalize(a, a, 0, 65535, cv2.NORM_MINMAX)
  np.right_shift(a, 8, a)
  return np.uint8(a)

serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sys.stderr.write("starting on port 22222\n")
serversock.bind(('192.168.0.122',22222))
serversock.listen(1)
while True:
    while True:
        #wait for client connections
        sys.stderr.write("waiting for clients to connect\n")
        connection, client_address = serversock.accept()
        try:
            sys.stderr.write("connection established from {0} \n".format(client_address))
            chunk = connection.recv(16)

            if chunk.decode('utf-8') != "HELO":
                sys.stderr.write("No HELO from client.  Ending connection.\n")
                connection.sendall('ERR'.encode('utf-8'))
                break

            connection.sendall('OK'.encode('utf-8'))
            while True:
                chunk = connection.recv(16)

                if chunk.decode('utf-8') != 'IMGREQ':
                    if chunk.decode('utf-8') == 'QUIT':
                        sys.stderr.write("QUIT requested from client. Ending connection.\n")
                    else: 
                        sys.stderr.write("Invalid request from client. Ending connection.\n")
                        connection.sendall('ERR'.encode('utf-8'))
                    break

                connection.sendall('OK'.encode('utf-8'))
                image= capture()
                cv2.imwrite('temp_img.png', image)
                
                
                #send image
                img_data = b''
                sys.stderr.write("capturing Image...\n")
                with open('temp_img.png', 'rb') as fp:
                    img_data = fp.read()


                #SEND SIZE OF IMAGE TO CLIENT
                size = int(sys.getsizeof(img_data)) -21 #bug TODO
                print(int(size))

                chunk = connection.recv(16)
                if chunk.decode('utf-8') != 'SIZE':
                    sys.stderr.write("no size request from client. Ending connection.\n")
                    connection.sendall('ERR'.encode('utf-8'))
                    break

                connection.sendall(str(size).encode('utf-8'))

                chunk = connection.recv(16)
                if chunk.decode('utf-8') != 'OK':
                    sys.stderr.write("Client no OK to size of img. closing connection...\n")
                    connection.sendall('ERR'.encode('utf-8'))
                    break 

                connection.sendall(img_data)
                sys.stderr.write("finished sending img. waiting for next img_req.\n")
        finally:
            connection.close()
