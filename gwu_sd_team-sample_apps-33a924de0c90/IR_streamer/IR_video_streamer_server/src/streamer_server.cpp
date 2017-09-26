//============================================================================
// Name        : streamer_server.cpp
// Author      : CosmaC
// Version     : V1.0
// Copyright   : GWU Research
// Description : Simple app for streaming video over the local network (TCP)
//============================================================================

#include <iostream>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using namespace std;

// Local Headers
#include "streamer.h"

// Main application
int main()
{
    // Communication Config
    const int portNumber = 5995;
    //string ip_address = "161.253.66.39";
    string ip_address = "161.253.66.95";


    // Create socket
    int socketHandle;
    if((socketHandle = socket(AF_INET, SOCK_STREAM, 0)) < 0)
    {
        cerr << "Server fail to create the socket." << endl;
        cerr << "Error: " << strerror(errno) << endl;
        close(socketHandle);
        exit(EXIT_FAILURE);
   }


    // Load system information into socket data structures
    struct sockaddr_in socketInfo;
    bzero(&socketInfo, sizeof(sockaddr_in));  // Clear structure memory
    socketInfo.sin_family = AF_INET;
    socketInfo.sin_addr.s_addr = inet_addr(ip_address.c_str());
    socketInfo.sin_port = htons((u_short)portNumber);


    // Bind the socket to a local socket address
    if( bind(socketHandle, (struct sockaddr *) &socketInfo, sizeof(socketInfo)) < 0)
    {
        cerr << "Server fail to bind the socket." << endl;
        cerr << "Error: " << strerror(errno) << endl;
        close(socketHandle);
        exit(EXIT_FAILURE);
    }


    // Wait for a client to connect
    if (listen(socketHandle, 1) < 0)
    {
        cerr << "Server fail to listen for a connection." << endl;
        cerr << "Error: " << strerror(errno) << endl;
        close(socketHandle);
        exit(EXIT_FAILURE);
    }


    // Accept incoming connection
    int socketConnection;
    if( (socketConnection = accept(socketHandle, NULL, NULL)) < 0)
    {
        cerr << "Server fail to accept client connection." << endl;
        cerr << "Error: " << strerror(errno) << endl;
        close(socketHandle);
        exit(EXIT_FAILURE);
    }
    close(socketHandle);


   // Send data
   char img[2 + 80 * 60 * 2];	// Header(MSG+CMD=2) + IR img(80*60*2)
   char msg[2];					// Header(MSG+Response=2)
   int rc = 0;
   memset(msg, 1, sizeof(msg));

   int count = 0;
   while (1)
   {
       ////////////////////////////////////////////////////////////////////////
       ///  Receive Request
       ////////////////////////////////////////////////////////////////////////
       rc = recv(socketConnection, msg, sizeof(msg), 0);
       cout << "[" << count << "]SERVER -- RECV -- Number of bytes read: " << rc << endl;

       // Check if connection is still open
       if (rc == -1)
       {
           cerr << "[" << count << "]SERVER -- CONNECTION -- Lost." << endl;
           cerr << "Error: " << strerror(errno) << endl;
           exit(EXIT_FAILURE);
       }

       // Frame request msg
       if (msg[0] == FRAME_REQUEST)
       {
    	   cout << "[" << count << "]SERVER -- RECV -- Message: FRAME_REQUEST" << endl;
    	   img[0] = FRAME_REQUEST;
    	   img[1] = FRAME_READY;
    	   memset((img+2), count, sizeof(img)-2);
       }
       else if (msg[0] == I2C_CMD)
       { // I2C command
    	   cout << "[" << count << "]SERVER -- RECV -- Message: I2C_CMD" << endl;
    	   img[0] = I2C_CMD;
    	   img[1] = I2C_SUCCEED;
       }
       else
       { // UNKNOWN Request
    	   cout << "[" << count << "]SERVER -- RECV -- Message: UNKNOWN_MSG" << endl;
    	   img[0] = UNKNOWN_MSG;
    	   img[1] = VOID;
       }

       ////////////////////////////////////////////////////////////////////////
       /// Send response
       ////////////////////////////////////////////////////////////////////////
       cout << "[" << count << "]SERVER -- SEND -- Sending message response... ";
       int sd = send(socketConnection, img, sizeof(img), 0);
       if (sd == -1)
       {
           cerr << "[" << count << "]SERVER -- CONNECTION -- Lost." << endl;
           cerr << "Error: " << strerror(errno) << endl;
           exit(EXIT_FAILURE);
       }
       cout << " Message sent! " << endl;

       count++;
   }

   // Close connection
   cout << "[" << count << "]SERVER -- Closing Connection...";
   close(socketConnection);
   cout << " Closed !" << endl;

   return 0;

}
