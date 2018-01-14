import socket
import sys

serversock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sys.stderr.write("starting on port 22222 of localhost\n")
serversock.bind(('localhost',22222))
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
            chunk = connection.recv(16)

            if chunk.decode('utf-8') != 'IMGREQ':
                sys.stderr.write("Invalid request from client. Ending connection.\n")
                connection.sendall('ERR'.encode('utf-8'))
                break

            connection.sendall('OK'.encode('utf-8'))

            #send image
            img_data = b''
            sys.stderr.write("capturing Image...\n")
            with open('beef.png', 'rb') as fp:
                img_data = fp.read()
            
            #size = ''
            #connection.sendall()
            #TODO SEND SIZE OF IMAGE TO CLIENT
            size = sys.getsizeof(img_data)
            print(size)

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
            sys.stderr.write("finished sending img. waiting for ack.\n")
            chunk = connection.recv(16)

            if chunk.decode('utf-8') == "QUIT":
                sys.stderr.write("QUIT has been requested. Ending Connection...\n")
                #connection.sendall('OK'.encode('utf-8'))
                break
        finally:
            connection.close()