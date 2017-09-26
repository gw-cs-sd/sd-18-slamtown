//============================================================================
// Name        : ThermalSensor.h
// Author      : CosmaC
// Version     : V1.0
// Copyright   : GWU Research
// Description : Thermal sensor grabber API
//============================================================================

#pragma once
#define _WINSOCKAPI_

#include "ThermalSensor/FLIRGrabber.h"
#include "ThermalSensor/FLIRGrabberCommon.h"

// C/C++
#include <winsock2.h>
#include <string.h>

// Boost
#include <boost/format.hpp>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>

// Sensor type
enum ThermalSensorType { LEPTON_V2, LEPTON_V3 };
constexpr int LEPTON_V2_WIDTH = 80;
constexpr int LEPTON_V2_HEIGHT = 60;
constexpr int LEPTON_V3_WIDTH = 160;
constexpr int LEPTON_V3_HEIGHT = 120;

class ThermalSensor
{
public:
    
    // Constructor/destructor
    ThermalSensor(int width, int height, ThermalSensorType type) :
        frame_id_(0),
        frame_available_(false),
        img_width_(width),
        img_height_(height),
        do_grab_(false),
        port_number_(5995),
        //ip_address_("169.254.20.203"), // static old ip
        ip_address_("169.254.244.43"), // new dynamic ip 
        sensor_temp_(-1),
        object_temp_(-1.0),
        type_(type) {

        // Checks if requested resolution is compatible with the sensor
        if (LEPTON_V3 == type_) {
            if (LEPTON_V3_WIDTH != img_width_ || LEPTON_V3_HEIGHT != img_height_) {
                img_width_ = LEPTON_V3_WIDTH;
                img_height_ = LEPTON_V3_HEIGHT;
                std::cerr << "[Error][ThermalSensor::ThermalSensor] Resolution (" << width <<
                    "," << height << ") is incompatible with LEPTON_V3."<< std::endl;
            }
        }
        else if (LEPTON_V2 == type_) {
            if (LEPTON_V2_WIDTH != img_width_ || LEPTON_V2_HEIGHT != img_height_) {
                img_width_ = LEPTON_V2_WIDTH;
                img_height_ = LEPTON_V2_HEIGHT;
                std::cerr << "[Error][ThermalSensor::ThermalSensor] Resolution (" << width <<
                    "," << height << ") is incompatible with LEPTON_V2." << std::endl;
            }
        }
        else {
            std::cerr << "[Error][ThermalSensor::ThermalSensor] Unknown sensor type."
                << std::endl;
        }

        lepton_dev_ = LeptonClient(width, height);
        img_size_ = img_width_ * img_height_;
        char msg[] = { FRAME_REQUEST, U8 };
        std::vector<char> new_msg(msg, msg + 1);
        msg_pipe_.push_back(new_msg);
        img_buffer_u8_ = new unsigned char[img_size_];
        img_buffer_u16_ = new unsigned short[img_size_];
        new_buffer_u16_ = new unsigned short[img_size_];
    }
    ~ThermalSensor() { 
        stop();
        delete[] img_buffer_u8_;
        delete[] img_buffer_u16_;
        delete[] new_buffer_u16_;
    }

    // Start thermal frame grabber
    bool start();
    // Stop thermal frame grabber
    bool stop();
    // Get next thermal frame
    unsigned long long nextFrame(unsigned char* frame_u8,
                                 int& sensor_temperature, float& object_temperature,
                                 bool force = false);
    unsigned long long nextFrame(unsigned char* frame_u8, unsigned short* frame_u16,
                                 int& sensor_temperature, float& object_temperature,
                                 bool force = false);
    unsigned long long nextFrame(unsigned short* frame_u16,
                                 int& sensor_temperature, float& object_temperature,
                                 bool force = false);
    // Send message to thermal sensor
    void sendMessage(char msg[2]);
    // Getters/Setter
    int getFrameWidth() { return img_width_; }
    int getFrameHeight() { return img_height_; }
    ThermalSensorType getType() { return type_; };
    bool hasNewFrame() { return frame_available_; }
    
private:
    // Grabbing thread
    void run();
    // Get new frame from sensor
    int GrabFrame(const char* msg, int msg_size);

    // Grabber internal flags and vars
    unsigned long long frame_id_;       // current frame id(number)
    bool               frame_available_;// new frame available to read
    int                img_width_;      // thermal frame width
    int                img_height_;     // thermal frame height
    unsigned char*     img_buffer_u8_;  // thermal frame buffer (u8)
    unsigned short*    img_buffer_u16_; // thermal frame buffer (u16)
    unsigned short*    new_buffer_u16_; // thermal frame buffer (u16)
    int                img_size_;       // thermal frame total number of pixels
    bool               do_grab_;        // thermal grabber state
    int                port_number_;    // thermal sensor server port
    std::string        ip_address_;     // thermal sensor server ip address (raspberryPI)
    int                sensor_temp_;    // thermal camera sensor temperature
    float              object_temp_;    // object temperature from pointing IR
    ThermalSensorType  type_;           // thermal sensor type, to personalize some functionality

    boost::thread     img_thread_;      // grabbing thread
    SOCKET            socket_handle_;   // thermal sensor server ethernet connection handle
    LeptonClient      lepton_dev_;      // thermal sensor client
    std::vector<std::vector<char>> msg_pipe_; // message queue
    boost::mutex      mtx_;             // lock for shared data
};
