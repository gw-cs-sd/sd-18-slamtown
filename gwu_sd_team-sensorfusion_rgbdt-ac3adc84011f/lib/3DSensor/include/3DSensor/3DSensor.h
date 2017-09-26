//============================================================================
// Name        : 3DSensor.h
// Author      : CosmaC
// Date        : October, 2016
// Copyright   : GWU Research
// Description : 3D sensor grabber API
//============================================================================

#pragma once
#define _WINSOCKAPI_

#include "3DSensor/KinectGrabber.h"

// C/C++
#include <string.h>

// Boost
#include <boost/format.hpp>
#include <boost/signals2.hpp>
#include <boost/thread.hpp>


class Sensor3D
{
public:
    
    // Constructor/destructor
    Sensor3D(int width_d, int height_d, int width_c, int height_c, int bpp_c) :
        frame_id_(0),
        frame_available_(false),
        depth_width_(width_d),
        depth_height_(height_d),
        color_width_(width_c),
        color_height_(height_c),
        color_bpp_(bpp_c),
        raw_color_bpp_(4),
        min_reliable_distance(0),
        do_grab_(false) {

        if (color_bpp_ != 3) {
            std::cout << "[ERROR][Sensor3D] The requested depth for the color image is not supported.\n";
            throw std::runtime_error("Only color image wth 3 bpp is accepted by current drivers.");
        }
        p_depth_dev_ = new KinectClient(depth_width_, depth_height_,
            color_width_, color_height_);
        depth_size_ = depth_width_ * depth_height_;
        color_size_ = color_width_ * color_height_;
        depth_buffer_ = new unsigned short[depth_size_];
        temp_depth_buffer_ = new unsigned short[depth_size_];
        color_buffer_ = new unsigned char[color_size_ * color_bpp_];
        raw_color_buffer_ = new unsigned char[color_size_ * raw_color_bpp_];

        skeletons_.resize(MAX_BODIES);
        temp_skeletons_.resize(MAX_BODIES);
    }
    ~Sensor3D() {
        stop();
        delete p_depth_dev_;
        delete[] depth_buffer_;
        delete[] temp_depth_buffer_;
        delete[] color_buffer_;
        delete[] raw_color_buffer_;
    }

    // Start 3D frame grabber
    bool start();
    // Stop 3D frame grabber
    bool stop();
    // Get next 3D frame
    unsigned long long nextFrame(unsigned short* frame_depth,
                                 unsigned char* frame_color,
                                 bool force = false);
    // Get next 3D frame and bodies in the frame
    unsigned long long nextFrame(unsigned short* frame_depth,
                                 unsigned char* frame_color,
                                 BodySkeleton* frame_skeletons,
                                 bool force = false);

    // Getters/Setters
    int getDepthFrameWidth() { return depth_width_; }
    int getDepthFrameHeight() { return depth_height_; }
    int getColorFrameWidth() { return color_width_; }
    int getColorFrameHeight() { return color_height_; }
    int getColorFrameBPP() { return color_bpp_; };
    bool hasNewFrame() { return frame_available_; }

    /**
     * @brief Returns a pointer to the coordinate mapper pointer
     */
    ICoordinateMapper** const getCoordinateMapper() {
        if (p_depth_dev_) {
            return p_depth_dev_->getCoordinateMapper();
        }
        else {
            return nullptr;
        }
    };
    
    /**
     * @brief Returns color camera vertical FOV
     */
    float getColorVerticalFOV() { 
        if (p_depth_dev_) {
            return p_depth_dev_->getColorVerticalFOV();
        }
        else {
            return 0.f;
        }
    };

private:
    // Grabbing thread
    void run();
    // Get new frame from sensor
    int GrabFrame();

    // Grabber internal flags and vars
    unsigned long long frame_id_;         // current frame id(number)
    bool               frame_available_;  // new frame available to read
    bool               do_grab_;          // depth grabber state

    int                depth_width_;      // depth frame width
    int                depth_height_;     // depth frame height
    unsigned short*    depth_buffer_;     // depth frame buffer (u16)
    unsigned short*    temp_depth_buffer_;// depth frame buffer (u16)
    int                depth_size_;       // depth frame total number of pixels
    unsigned short     min_reliable_distance; // minimum depth for reliable distance measure
    
    int                color_width_;      // color frame width
    int                color_height_;     // color frame height
    int                color_size_;       // color frame size in pixels
    int                color_bpp_;        // bytes per pixel for the color frame
    int                raw_color_bpp_;    // bytes per pixel for the raw color frame
    unsigned char*     color_buffer_;     // color frame buffer (u8)
    unsigned char*     raw_color_buffer_; // color raw frame buffer (u8)
    
    std::vector<BodySkeleton> skeletons_; // skeletons for the current frame
    std::vector<BodySkeleton> temp_skeletons_; // skeletons for the current frame

    boost::thread     img_thread_;       // grabbing thread
    KinectClient*     p_depth_dev_;      // 3D sensor(kinect) client
    boost::mutex      mtx_;              // lock for shared data
};
