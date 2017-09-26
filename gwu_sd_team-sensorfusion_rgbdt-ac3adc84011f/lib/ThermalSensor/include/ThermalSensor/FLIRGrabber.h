//============================================================================
// Name        : FLIRGrabber.h
// Author      : CosmaC
// Version     : V1.0
// Copyright   : GWU Research
// Description : FLIR Lepton grabber client API
//============================================================================

#pragma once
#define _WINSOCKAPI_

#include "ThermalSensor/FLIRGrabberCommon.h"

#include <winsock2.h>

#include <vector>
#include <string>

// Debug option
#define DPRINT //printf

class LeptonClient {

public:

    /**
     * @brief Constructor/destructor
     */
    LeptonClient() :
        count(0),
        frame_width(80),
        frame_height(60){
        frame_size = frame_width * frame_height;
        frame_pckg_size = 2 + 4 + 4 + frame_size * 2;// Header(MSG+CMD=2) + SensorTemp(4) + IR img(80*60*2)
        frame_pckg.resize(frame_pckg_size);
    }

    LeptonClient(int width, int height) :
        count(0),
        frame_width(width),
        frame_height(height) {
        frame_size = frame_width * frame_height;
        frame_pckg_size = 2 + 4 + 4 + frame_size * 2;// Header(MSG+CMD=2) + SensorTemp(4) + IR img(80*60*2)
        frame_pckg.resize(frame_pckg_size);
    }
    
    ~LeptonClient() {};

    /**
     * @brief Setters/Getters
     */
    inline int getFrameWidth() { return frame_width; }
    inline int getFrameHeight() { return frame_height; }

    /**
     * @brief Closes connection with LeptonServer.
     */
    int closeConnection(SOCKET socket_hdl);

    /**
     * @brief Opens a new connection with LeptonServer.
     */
    SOCKET openConnection(int port_number, std::string ip_address);

    /**
     * @brief Sends the message to LeptonServer.
     */
    void sendMessage(SOCKET socket_hdl, const char msg[2], int msg_size);

    /**
     * @brief Receives a message from LeptonServer.
     */
    int receiveMessage(SOCKET socket_hdl, char *buffer, const int pckg_size);

    /**
     * @brief Receives a frame from LeptonServer. 
     *        The method waits for the first message from the server to be available.
     *        The message is interpreted as a frame message.
     *        Make sure you ask the server for a new frame before calling this method.
     *
     * @returns 1 - frame received
     *          0 - unknown response
     *         -1 - connection lost
     */
    int receiveFrame(SOCKET socket_hdl, char pixel_depth,
                     unsigned char *ir_img, unsigned short *ir_u16,
                     int& sensor_temp, float& object_temp);
private:
    int count;
    int frame_width;
    int frame_height;
    int frame_size;
    int frame_pckg_size;
    std::vector<char> frame_pckg;
};
