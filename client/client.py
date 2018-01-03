import socket
import sys
from datetime import datetime
#import numpy

clientsock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_address = ('localhost', 22222) #connect to where server is listening
sys.stderr.write("connecting to server at localhost port 22222\n")
clientsock.connect(server_address)

clientsock.sendall('HELO'.encode('utf-8'))
chunk = clientsock.recv(16)
if chunk.decode('utf-8') != "OK":
    sys.stderr.write("No OK to HELO\n")
    clientsock.close

sys.stderr.write("Sending Image Request\n")
clientsock.sendall('IMGREQ'.encode('utf-8'))
chunk = clientsock.recv(16)
if chunk.decode('utf-8') != "OK":
    sys.stderr.write("No OK to IMG REQ\n")
    clientsock.close

clientsock.sendall('SIZE'.encode('utf-8'))
size = clientsock.recv(32).decode('utf-8')
clientsock.sendall('OK'.encode('utf-8')) 
print(size)
img_data = clientsock.recv(int(size))
#print (img_data)

#recv image (loop)
# img_data = ''
# while True:
#     chunk = clientsock.recv(1024)
#     img_data += chunk.decode('utf-8')
#     if 'eof\n' in img_data:
#         img_data.strip('eof\n')
#         img_data = img_data.encode('utf-8')
#         break



sys.stderr.write("received all the img data. converting to file...\n")
myfile = open(str(datetime.now().strftime('%m%d%H%M%S')) + '.png', 'wb')
#myfile = open('received_file.png', 'wb')
myfile.write(img_data)
myfile.close()

clientsock.sendall('QUIT'.encode('utf-8'))
sys.stderr.write("Session Closed\n")
chunk = clientsock.recv(16)