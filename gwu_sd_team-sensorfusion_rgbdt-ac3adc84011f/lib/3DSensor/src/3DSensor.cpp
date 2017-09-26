//============================================================================
// Name        : 3DSensor.cpp
// Author      : CosmaC
// Date        : October, 2016
// Copyright   : GWU Research
// Description : 3D sensor grabber implementation
//============================================================================


// Module interface
#include "3DSensor/3DSensor.h"
#include "3DSensor/KinectGrabber.h"

// C/C++
#include <winsock2.h>
#include <string.h>
#include <vector>


// Start depth frame grabber
bool Sensor3D::start() {
    
    // Check if not running
    if (!do_grab_) {

        // Check if valid object
        if (!p_depth_dev_) {
            printf("[Error][3DSensor::start] Invalid depth sensor object.\n");
            return false;
        }

        // Open connection
        bool dev_open = p_depth_dev_->openConnection();

        // Check if valid handle
        if (!dev_open) {
            printf("[Error][3DSensor::start] Unable to open connection with the depth sensor.\n");
            return false;
        }

        // Create grabbing thread
        do_grab_ = true;
        img_thread_ = boost::thread(&Sensor3D::run, this);
    }

    // if everything is fine, return OKAY
    return true;
}

// Stop depth frame grabber
bool Sensor3D::stop(){

    // Check if running
    if (do_grab_) {

        // Flag stop grabbing
        do_grab_ = false;

        // Wait for thread to end
        img_thread_.join();

        // Check if valid object
        if (!p_depth_dev_) {
            printf("[Error][3DSensor::stop] Invalid depth sensor object.\n");
            return false;
        }

        // Close connection
        if (!p_depth_dev_->closeConnection()) {
            printf("[Error][Sensor3D::stop] Unable to close connection with the depth sensor.\n");
            return false;
        }
    }

    // if everything fine, return OKAY
    return true;
}

// Depth frame grabber run
void Sensor3D::run() {

  // Grab frames
  while(do_grab_) {

    // Grab
    if (!GrabFrame()) {
        continue;
    }

    // New frame received
    mtx_.lock();
    unsigned short* aux = depth_buffer_;
    depth_buffer_ = temp_depth_buffer_;
    temp_depth_buffer_ = aux;
    temp_skeletons_.swap(skeletons_);
    
    // Convert color buffer
    unsigned char* src = raw_color_buffer_;
    unsigned char* dst = color_buffer_;
    for (size_t i = 0; i < color_size_; ++i) {
        dst[0] = src[0];
        dst[1] = src[1];
        dst[2] = src[2];
        src += raw_color_bpp_;
        dst += color_bpp_;
    }
    frame_available_ = true;
    frame_id_++;
    mtx_.unlock();
  }
}

// Get new frame from sensor
int Sensor3D::GrabFrame() {

    // Check if valid object
    if (!p_depth_dev_) {
        printf("[Error][3DSensor::GetFrame] Invalid depth sensor object.\n");
        return false;
    }

    // Wait for the frame
    int res = p_depth_dev_->receiveFrame(temp_depth_buffer_, 
        &min_reliable_distance, raw_color_buffer_, temp_skeletons_);
    
    if (res == -1) { // connection closed

        // Open connection
        bool dev_open = p_depth_dev_->openConnection();

        // Check if valid handle
        if (!dev_open) {
            printf("[Error][3DSensor::run::GrabFrame] Unable to re-open connection "
                "with the depth sensor.\n");
            return 0;
        }
        INFO_PRINT("[INFO][3DSensor::run::GrabFrame] Re-open connection with the depth sensor.\n");
    }
    
    return res;
}

// Get depth frame
unsigned long long Sensor3D::nextFrame(unsigned short* frame_depth,
                                       unsigned char* frame_color,
                                       bool force) {

    mtx_.lock();
    if (frame_available_ || force) {
        memcpy(frame_depth, depth_buffer_, depth_size_ * sizeof(unsigned short));
        memcpy(frame_color, color_buffer_, color_size_ * color_bpp_);
        frame_available_ = false;
    }
    mtx_.unlock();

    return frame_id_;
}

// Get depth frame and bodies in the frame
unsigned long long Sensor3D::nextFrame(unsigned short* frame_depth,
                                       unsigned char* frame_color,
                                       BodySkeleton* frame_skeletons,
                                       bool force) {

    mtx_.lock();
    if (frame_available_ || force) {
        memcpy(frame_depth, depth_buffer_, depth_size_ * sizeof(unsigned short));
        memcpy(frame_color, color_buffer_, color_size_ * color_bpp_);
        memcpy(frame_skeletons, skeletons_.data(), skeletons_.size() * sizeof(BodySkeleton));
        frame_available_ = false;
    }
    mtx_.unlock();

    return frame_id_;
}