//============================================================================
// Name        : FLIRGrabber.cpp
// Author      : CosmaC
// Version     : V1.0
// Copyright   : GWU Research
// Description : FLIR Lepton grabber client implementation
//============================================================================

#define WIN32_LEAN_AND_MEAN

// Module interface
#include "ThermalSensor/FLIRGrabber.h"

// C/C++
#include <winsock2.h>
#include <ws2tcpip.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <errno.h>
using namespace std;
#pragma comment(lib, "Ws2_32.lib")

// Close Connection
int LeptonClient::closeConnection(SOCKET socket_hdl)
{
  // Shut down the transmision
  int res = shutdown(socket_hdl, SD_SEND);
  if (res == SOCKET_ERROR) {
      printf("[Error] shutdown() failed with error: %d\n", WSAGetLastError());
      WSACleanup();
      return 0;
  }  

  // Close the socket
  res = closesocket(socket_hdl);
  if (res == SOCKET_ERROR) {
      printf("[Error] closesocket() failed with error = %d\n", WSAGetLastError() );
      WSACleanup();
      return 0;
  }   

  // if everything fine, return OKAY
  return 1;
}

// Open Connection
SOCKET LeptonClient::openConnection(int port_number, string ip_address)
{
    // Initialize Winsock
    WSADATA wsaData;
    int iResult = WSAStartup(MAKEWORD(2,2), &wsaData);
    if (iResult != 0) {
        printf("[Error][IR_Connect] WSAStartup failed with error: %d\n", iResult);
        exit(EXIT_FAILURE);
    }

    // Create socket
    SOCKET socket_hdl = INVALID_SOCKET;
    socket_hdl = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if(socket_hdl == INVALID_SOCKET) {
        printf("[Error][IR_Connect] Error at socket(): %ld\n", WSAGetLastError());
        WSACleanup();
        exit(EXIT_FAILURE);
   }


    // Load system information into socket data structures
    struct sockaddr_in remoteSocketInfo;
    ZeroMemory( &remoteSocketInfo, sizeof(remoteSocketInfo) );
    remoteSocketInfo.sin_family = AF_INET;
    remoteSocketInfo.sin_port = htons((u_short)port_number);
    remoteSocketInfo.sin_addr.S_un.S_addr = inet_addr(ip_address.c_str());

    // Establish the connection with the server
    iResult = connect(socket_hdl, (SOCKADDR*) &remoteSocketInfo, (int)sizeof(remoteSocketInfo));
    if (iResult == SOCKET_ERROR) {
        printf("[Error][IR_Connect] Error at connect(): %ld\n", WSAGetLastError());
        closesocket(socket_hdl);
        exit(EXIT_FAILURE);
    }

    // Connection created successful
    return socket_hdl;
}

// Send a new request
void LeptonClient::sendMessage(SOCKET socket_hdl, const char msg[2], int msg_size)
{
    DPRINT("[%d]CLIENT -- SEND -- Sending message... ", count);
    int sd = send(socket_hdl, msg, msg_size, 0);
    if (sd == -1) {
        cerr << "[" << count << "]CLIENT -- CONNECTION -- Lost." << endl;
        cerr << "Error: " << strerror(errno) << endl;
        exit(EXIT_FAILURE);
    }
    DPRINT(" Message sent! \n");
}

// Receive next message
int LeptonClient::receiveMessage(SOCKET socket_hdl, char *buffer, const int pckg_size)
{
    u_long data_size = 0;
    do
    { // wait for data to be available
        ioctlsocket(socket_hdl, FIONREAD, &data_size);
        //cerr << "Error: " << data_size << endl;
    } while (data_size < pckg_size);
    int rc = recv(socket_hdl, buffer, pckg_size, 0);
    DPRINT("[%d]CLIENT -- RECV -- Number of bytes read: %d \n", count, rc);

    return rc;
}

// Receives a frame from LeptonServer
int LeptonClient::receiveFrame(SOCKET socket_hdl, char pixel_depth,
                               unsigned char *ir_img, unsigned short *ir_u16,
                               int& sensor_temp, float& object_temp) {

    // Frame buffer
    char* img = frame_pckg.data();
    
    // Receive data from IR server
    int rc = receiveMessage(socket_hdl, img, frame_pckg_size);

    // Check if connection is still open
    if (rc == -1) {
        cerr << "[" << count << "]CLIENT -- CONNECTION -- Lost." << endl;
        cerr << "Error: " << strerror(errno) << endl;
        return -1;
    }

    // Check response header
    if (img[0] == FRAME_REQUEST) {

        DPRINT("[%d]CLIENT -- RECV -- FRAME_REQUEST response. \n", count);
        if (pixel_depth == U8) {
            memcpy(ir_img, (img + 10), frame_size);
            object_temp = ((float*)(img + 2))[0];
            sensor_temp = ((int*)(img + 6))[0];
        }
        else { // U16
            memcpy(ir_u16, (img + 10), 2 * frame_size);
            object_temp = ((float*)(img + 2))[0];
            sensor_temp = ((int*)(img + 6))[0];
            
            // normalize(temp, ir_img, 0, 255, NORM_MINMAX, CV_8U);
            unsigned short minValue = 30000;
            unsigned short maxValue = 31300;
            unsigned short* u16_data = (unsigned short*)ir_u16;
            unsigned char* u8_data = ir_img;

            static const double scale = 255.0 / (maxValue - minValue);
            for (int idx = 0; idx < frame_size; ++idx) {
                //printf("%d ", u16_data[idx]);
                if (u16_data[idx] <= minValue) {
                    u8_data[idx] = 0;
                }
                else if (u16_data[idx] >= maxValue) {
                    u8_data[idx] = 255;
                }
                else {
                    u8_data[idx] = static_cast<unsigned char>((u16_data[idx] - minValue) * scale);
                }
            }
        }

        return 1;
    }
    else if (img[0] == I2C_CMD) {

        // set out frames to 0
        memset(ir_img, 0, frame_size);
        memset(ir_u16, 0, 2 * frame_size);

        // Check if I2C cmd succeed
        if (img[1] == I2C_FAILED) {
            return 0;
        }

        // Check I2C cmd type
        if (1) {
          int *buf1 = (int *)(img+2);
          int *buf2 = (int *)(ir_img);
          // Save temp
          buf2[0] = buf1[0];
        }

        DPRINT("[%d]CLIENT -- RECV -- I2C_CMD response.", count);
        return 1;
    }
    else if (img[0] == UNKNOWN_MSG) {
        DPRINT("[%d]CLIENT -- Server did not recognize your request.", count);
        return 0;
    }
    else {
        DPRINT("[%d]CLIENT -- Unable to decode server message.", count);
        return 0;
    }
}
