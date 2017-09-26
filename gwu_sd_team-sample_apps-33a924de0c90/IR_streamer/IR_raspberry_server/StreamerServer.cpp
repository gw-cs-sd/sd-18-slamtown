//============================================================================
// Name        : StreamerServer.cpp
// Author      : CosmaC
// Version     : V2.0
// Copyright   : GWU Research
// Description : Simple app for streaming video over the local network (TCP)
//============================================================================

// C/C++ Tools
#include <stdio.h>
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
#include "MLX90621.h"
#include "streamer.h"
#include "LeptonAPI.h"

// User defines
#define DPRINTF //printf


//============================================================================
// Main application
//============================================================================
int main()
{
    // Communication Config
    const int portNumber = 5995;
    //string ip_address = "161.253.66.39"; // Ubuntu machine IP
    //string ip_address = "161.253.66.95"; // Raspberry PI IP
    //string ip_address = "169.254.20.203"; // Old static ip address
    string ip_address = "169.254.244.43";  // New ip address


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

    // Init pointining IR sensor
    MLX90621 mlx_sensor;
    if(!mlx_sensor.Init()) {
        cerr << " [ERROR:main] MLX90621 init failed! \n";
        return 0;
    }
    leptonOpenConnection();

   // Send data
   const int width = FRAME_WIDTH;
   const int height = FRAME_HEIGHT;
   const int bpp = 2;
   const int header_size = 2 + 4 + 4; // Header(MSG(1) + CMD(1) + To(4) + SensorTemp(4)
   char img[header_size + width * height * bpp]; // Header + IR img
   char msg[2];			// Header(MSG+Response=2)
   int rc = 0;
   memset(msg, 1, sizeof(msg));

   int count = 0;
   while (1)
   {
       ////////////////////////////////////////////////////////////////////////
       ///  Receive Request
       ////////////////////////////////////////////////////////////////////////
       rc = recv(socketConnection, msg, sizeof(msg), 0);
       DPRINTF("[%d]SERVER -- RECV -- Number of bytes read: %d \n", count, rc);

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
    	   DPRINTF("[%d]SERVER -- RECV -- Message: FRAME_REQUEST \n", count);
    	   img[0] = FRAME_REQUEST;
           ((float *)(img+2))[0] = mlx_sensor.GetTo();
           if (msg[1] == U8)
               leptonGetFrame(img+6, 8);
	   else
           leptonGetFrame(img+6, 16);
           img[1] = FRAME_READY;
       }
       else if (msg[0] == I2C_CMD)
       { // I2C command
			DPRINTF("[%d]SERVER -- RECV -- Message: I2C_CMD \n", count);
			img[0] = I2C_CMD;
			leptonCommand(img, msg);
       }
       else
       { // UNKNOWN Request
    	   DPRINTF("[%d]SERVER -- RECV -- Message: UNKNOWN_MSG \n", count);
    	   img[0] = UNKNOWN_MSG;
    	   img[1] = VOID;
       }

       ////////////////////////////////////////////////////////////////////////
       /// Send response
       ////////////////////////////////////////////////////////////////////////
       DPRINTF("[%d]SERVER -- SEND -- Sending message response... ", count);
       int sd = send(socketConnection, img, sizeof(img), 0);
       if (sd == -1)
       {
           cerr << "[" << count << "]SERVER -- CONNECTION -- Lost." << endl;
           cerr << "Error: " << strerror(errno) << endl;
           exit(EXIT_FAILURE);
       }
       DPRINTF(" Message sent! \n");

       count++;
   }

    // Release sensors
    mlx_sensor.Deinit();
    leptonCloseConnection();

   // Close connection
   DPRINTF("[%d]SERVER -- Closing Connection...", count);
   close(socketConnection);
   DPRINTF(" Closed ! \n");

   return 0;

}
