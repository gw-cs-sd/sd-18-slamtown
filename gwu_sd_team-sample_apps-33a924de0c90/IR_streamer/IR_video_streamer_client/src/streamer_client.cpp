//============================================================================
// Name        : streamer_client.cpp
// Author      : CosmaC
// Version     : V1.0
// Copyright   : GWU Research
// Description : Simple app for streaming video over the local network (TCP)
//============================================================================

#include <iostream>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

using namespace std;
using namespace cv;

// Local Headers
#include "streamer.h"

// User Defines
#define DPRINTF //printf


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
        cerr << "Client fail to create the socket." << endl;
        cerr << "Error: " << strerror(errno) << endl;
        close(socketHandle);
        exit(EXIT_FAILURE);
   }


    // Load system information into socket data structures
    struct sockaddr_in remoteSocketInfo;
    bzero(&remoteSocketInfo, sizeof(sockaddr_in));  // Clear structure memory
    remoteSocketInfo.sin_family = AF_INET;
    remoteSocketInfo.sin_addr.s_addr = inet_addr(ip_address.c_str());
    remoteSocketInfo.sin_port = htons((u_short)portNumber);


    // Establish the connection with the server
    if(connect(socketHandle, (struct sockaddr *)&remoteSocketInfo, sizeof(sockaddr_in)) < 0)
    {
        cerr << "Client fail to connect to the server." << endl;
        cerr << "Error: " << strerror(errno) << endl;
        close(socketHandle);
        exit(EXIT_FAILURE);
    }

    // Send data
    int pckg_size = 2 + 80 * 60 * 2;// Header(MSG+CMD=2) + IR img(80*60*2)
    char img[pckg_size];
    char msg[2];					// Header(MSG+Response=2)
    int rc = 0;
    Mat ir_img (60, 80, CV_8UC1);

    int count = 0;
    while (1)
    {

    	////////////////////////////////////////////////////////////////////////
    	///  Create Request
    	////////////////////////////////////////////////////////////////////////
        msg[0] = FRAME_REQUEST;
        msg[1] = VOID;


    	////////////////////////////////////////////////////////////////////////
    	///  Send Request
    	////////////////////////////////////////////////////////////////////////
        DPRINTF("[%d]CLIENT -- SEND -- Sending message... ", count);
        int sd = send(socketHandle, msg, sizeof(msg), 0);
        if (sd == -1)
        {
            cerr << "[" << count << "]CLIENT -- CONNECTION -- Lost." << endl;
            cerr << "Error: " << strerror(errno) << endl;
            exit(EXIT_FAILURE);
        }
        DPRINTF(" Message sent! \n");


        ////////////////////////////////////////////////////////////////////////
        /// Receive response
        ////////////////////////////////////////////////////////////////////////
        //memset(img, 0, sizeof(img));
        int data_size = 0;
        do
        { // wait for data to be available
        	ioctl(socketHandle, FIONREAD, &data_size);
        } while (data_size < pckg_size);
        rc = recv(socketHandle, img, sizeof(img), 0);
        DPRINTF("[%d]CLIENT -- RECV -- Number of bytes read: %d \n", count, rc);

        // Check if connection is still open
        if (rc == -1)
        {
            cerr << "[" << count << "]CLIENT -- CONNECTION -- Lost." << endl;
            cerr << "Error: " << strerror(errno) << endl;
            exit(EXIT_FAILURE);
        }

        // Check response header
        if (img[0] == FRAME_REQUEST)
        {
        	DPRINTF("[%d]CLIENT -- RECV -- FRAME_REQUEST response. \n", count);
        	for (int i = 0; i < ir_img.rows; i++)
        		for (int j = 0; j < ir_img.cols; j++)
        			ir_img.data[i * ir_img.cols + j] = img[i * ir_img.cols + j + 2];
        }
        else if (img[0] == I2C_CMD)
        {
        	DPRINTF("[%d]CLIENT -- RECV -- I2C_CMD response. \n", count);
        }
        else if (img[0] == UNKNOWN_MSG)
        {
            cerr << "[" << count << "]CLIENT -- Server did not recognize your request." << endl;
            exit(EXIT_FAILURE);
        }
        else
        {
            cerr << "[" << count << "]CLIENT -- Unable to decode server message." << endl;
            exit(EXIT_FAILURE);
        }

        // Show image
        imshow("IR Img", ir_img);
        cvWaitKey(10);

        count++;
    }

    return 0;

}
