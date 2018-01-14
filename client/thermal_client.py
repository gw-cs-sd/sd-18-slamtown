import socket
import sys
import cv2
import numpy
from datetime import datetime

class ThermalClient:
    def __init__(self,server_ip, port):
        self.clientsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_socket_data = (server_ip, port) #connect to where server is listening
        self.clientsock.connect(server_socket_data)
        sys.stderr.write("connected to server at %s, port %d\n" % (server_ip, port))
        self.clientsock.sendall('HELO'.encode('utf-8'))
        chunk = self.clientsock.recv(16)
        if chunk.decode('utf-8') != "OK":
            sys.stderr.write("No OK to HELO\n")
            self.clientsock.close

    def ImageRequest(self):
        sys.stderr.write("Sending Image Request\n")
        self.clientsock.sendall('IMGREQ'.encode('utf-8'))
        chunk = self.clientsock.recv(16)
        if chunk.decode('utf-8') != "OK":
            sys.stderr.write("No OK to IMG REQ\n")
            self.clientsock.close

        self.clientsock.sendall('SIZE'.encode('utf-8'))
        img_size = self.clientsock.recv(32).decode('utf-8')
        self.clientsock.sendall('OK'.encode('utf-8')) 
        print(img_size)

        chunks = []
        bytes_received = 0
        while bytes_received < int(img_size):
            chunk = self.clientsock.recv(min(int(img_size) - bytes_received, 8192))
            if chunk == b'':
                raise RuntimeError("socket connection broken")
                break
            bytes_received = bytes_received + len(chunk)
            chunks.append(chunk)
            print("bytes_received")
            print(bytes_received)
            print(int(img_size) - bytes_received)
        img_data = b''.join(chunks) 

        #img_data = clientsock.recv(int(img_size))

        sys.stderr.write("received all the img data. converting to file...\n")
        filename = str(datetime.now().strftime('Images/%m%d%H%M%S')) + '.png'
        myfile = open(filename, 'wb')
        myfile.write(img_data)
        myfile.close()
        cv2.namedWindow('image', cv2.WINDOW_NORMAL)
        img = cv2.imread(filename,0)
        cv2.imshow('image',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    def Close(self):
        self.clientsock.sendall('QUIT'.encode('utf-8'))
        sys.stderr.write("Session Closed\n")
        chunk = self.clientsock.recv(16)

myClient = ThermalClient('192.168.0.122', 22222)
myClient.ImageRequest()
myClient.Close()

