//============================================================================
// Name        : ThermalSensor.cpp
// Author      : CosmaC
// Version     : V1.0
// Copyright   : GWU Research
// Description : Thermal sensor grabber implementation
//============================================================================


// Module interface
#include "ThermalSensor/ThermalSensor.h"
#include "ThermalSensor/FLIRGrabber.h"

// C/C++
#include <winsock2.h>
#include <string.h>
#include <vector>


// Start IR img grabber
bool ThermalSensor::start() {
    
    // Check if not running
    if (!do_grab_) {

        // Open connection
        socket_handle_ = lepton_dev_.openConnection(port_number_, ip_address_);

        // Check if valid handle
        if (socket_handle_ == INVALID_SOCKET) {
            printf("[Error][ThermalSensor::start] Unable to open ethernet connection with the IR sensor.\n");
            return false;
        }

        // Create grabbing thread
        do_grab_ = true;
        img_thread_ = boost::thread(&ThermalSensor::run, this);
    }

    // if everything is fine, return OKAY
    return true;
}

// Stop IR img grabber
bool ThermalSensor::stop(){

    // Check if running
    if (do_grab_) {

        // Flag stop grabbing
        do_grab_ = false;

        // Wait for thread to end
        img_thread_.join();

        // Close IR connection
        if (!lepton_dev_.closeConnection(socket_handle_)) {
            printf("[Error][ThermalSensor::stop] Unable to close ethernet connection with the IR sensor.\n");
            return false;
        }
    }

    // if everything fine, return OKAY
    return true;
}

// IR img grabber run
void ThermalSensor::run() {

  // Message to send
  static char msg[2];  // Header(MSG Type + Details=2)

  // Grab frames
  while(do_grab_) {

    // set message
    msg[0] = msg_pipe_[0][0];
    msg[1] = msg_pipe_[0][1];
    if (msg_pipe_.size() > 1) {
        mtx_.lock();
        msg_pipe_.erase(msg_pipe_.begin());
        mtx_.unlock();
    }

    // Grab
    if (!GrabFrame(msg, 2)) {
        continue;
    }

    // New frame received
    mtx_.lock();
    unsigned short* aux = img_buffer_u16_;
    img_buffer_u16_ = new_buffer_u16_;
    new_buffer_u16_ = aux;
    frame_available_ = true;
    frame_id_++;
    mtx_.unlock();
  }
}

// Get new frame from sensor
int ThermalSensor::GrabFrame(const char *msg, int msg_size) {

    // Ask for a frame
    lepton_dev_.sendMessage(socket_handle_, msg, 2);

    // Wait for the frame
    int res = lepton_dev_.receiveFrame(socket_handle_, msg[1], img_buffer_u8_, 
        img_buffer_u16_, sensor_temp_, object_temp_);
    if (res == -1) { // connection closed

        // Open connection
        printf("[Error][ThermalGrabber::run::GrabFrame] Try to reopen connection.\n");
        socket_handle_ = lepton_dev_.openConnection(port_number_, ip_address_);

        // Check if valid handle
        if (socket_handle_ == INVALID_SOCKET) {
            printf("[Error][ThermalGrabber::run::GrabFrame] Unable to re-open ethernet connection with the IR sensor.\n");
            return 0;
        }
    }

    // Check if new frame is different from the previous one
    int diff_sum = 0;
    for (size_t idx = 0; idx < img_size_; ++idx) {
        diff_sum += std::abs(static_cast<int>(img_buffer_u16_[idx] - new_buffer_u16_[idx]));
        if (diff_sum > 0) {
            return 1;
        }
    }
    
    // frames are identical
    return 0;
}

// Set message to be sent
void ThermalSensor::sendMessage(char msg[2]) {

    std::vector<char> new_msg(&msg[0], &msg[0]+2);
    mtx_.lock();
    msg_pipe_.push_back(new_msg);
    mtx_.unlock();
}

// Get frame u8
unsigned long long ThermalSensor::nextFrame(unsigned char* frame_u8,
                                            int& sensor_temperature,
                                            float& object_temperature,
                                            bool force) {

    mtx_.lock();
    if (frame_available_ || force) {
        memcpy(frame_u8, img_buffer_u8_, img_size_);
        sensor_temperature = sensor_temp_;
        object_temperature = object_temp_;
        frame_available_ = false;
    }
    mtx_.unlock();

    return frame_id_;
}

// Get frame u8 and u16
unsigned long long ThermalSensor::nextFrame(unsigned char* frame_u8, 
                                            unsigned short* frame_u16,
                                            int& sensor_temperature,
                                            float& object_temperature,
                                            bool force) {

    mtx_.lock();
    if (frame_available_ || force) {
        memcpy(frame_u8, img_buffer_u8_, img_size_);
        memcpy(frame_u16, new_buffer_u16_, img_size_ * 2);
        sensor_temperature = sensor_temp_;
        object_temperature = object_temp_;
        frame_available_ = false;
    }
    mtx_.unlock();

    return frame_id_;
}

// Get frame u16
unsigned long long ThermalSensor::nextFrame(unsigned short* frame_u16,
                                            int& sensor_temperature,
                                            float& object_temperature,
                                            bool force) {

    mtx_.lock();
    if (frame_available_ || force) {
        memcpy(frame_u16, new_buffer_u16_, img_size_ * 2);
        sensor_temperature = sensor_temp_;
        object_temperature = object_temp_;
        frame_available_ = false;
    }
    mtx_.unlock();

    return frame_id_;
}