//============================================================================
// Name        : KinectGrabber.cpp
// Author      : CosmaC
// Date        : October, 2016
// Copyright   : GWU Research
// Description : Kinect grabber client implementation
//============================================================================

#define WIN32_LEAN_AND_MEAN

// Module interface
#include "3DSensor/KinectGrabber.h"

// C/C++
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <string>
#include <vector>
#include <errno.h>

// Close Connection
bool KinectClient::closeConnection() {

    // Close comunication
    if (p_kinect_sensor) {
        HRESULT hr = p_kinect_sensor->Close();
        if (FAILED(hr)) {
            std::cerr << "[Error][KinectGrabber::closeConnection] Unable to disconnect "
                "the Kinect device." << std::endl;
            return false;
        }
        p_kinect_sensor->Release();
        p_kinect_reader->Release();
        p_kinect_mapper->Release();
        p_kinect_sensor = nullptr;
        p_kinect_reader = nullptr;
        p_kinect_mapper = nullptr;
    }

    // if everything fine, return OKAY
    return true;
}

// Open Connection
bool KinectClient::openConnection() {

    // Connect to the default kinect sensor
    HRESULT hr = GetDefaultKinectSensor(&p_kinect_sensor);
    if (FAILED(hr) || !p_kinect_sensor) {
        std::cerr << "[Error][KinectGrabber::openConnection] Unable to find "
            "connected Kinect device." << std::endl;
        return false;
    }
    // If Kinect open correctly
    else {

        // Open comunication
        hr = p_kinect_sensor->Open();
        if (FAILED(hr)) {
            std::cerr << "[Error][KinectGrabber::openConnection] Unable to connect "
                "to the Kinect device." << std::endl;
            return false;
        }
        else {

            hr = p_kinect_sensor->OpenMultiSourceFrameReader(
                FrameSourceTypes::FrameSourceTypes_Depth |
                FrameSourceTypes::FrameSourceTypes_Color |
                //FrameSourceTypes::FrameSourceTypes_Infrared,
                FrameSourceTypes::FrameSourceTypes_Body,
                &p_kinect_reader);
            if (FAILED(hr)) {
                std::cerr << "[Error][KinectGrabber::openConnection] Unable to create "
                    "the multi-frame reader." << std::endl;
                return false;
            }

            hr = p_kinect_sensor->get_CoordinateMapper(&p_kinect_mapper);
            if (FAILED(hr)) {
                std::cerr << "[Error][KinectGrabber::openConnection] Unable to create "
                    "the coordinate space mapper." << std::endl;
                return false;
            }

        }
    }

    // Connection created successful
    return true;
}

// Receives a frame from LeptonServer
int KinectClient::receiveFrame(unsigned short *depth_img, unsigned short* min_reliable_distance,
                               unsigned char *color_img, std::vector<BodySkeleton>& skeletons) {

    // Reset skeletons info
    for (auto& skeleton : skeletons) {
        skeleton.is_tracked = false;
    }

    // Checks if reader was created and open properly
    if (!p_kinect_reader) {
        std::cerr << "[Error][KinectClient::receiveFrame] Invalid frame reader."
            << std::endl;
        return -1;
    }

    // Get current frame
    IMultiSourceFrame* frame = nullptr;
    HRESULT hr = p_kinect_reader->AcquireLatestFrame(&frame);

    // Process multi source frame
    if (SUCCEEDED(hr)) {

        hr = getDepthData(frame, depth_img, min_reliable_distance);
        if (FAILED(hr)) {
            INFO_PRINT("[INFO][KinectClient::receiveFrame] Unable to acquire depth frame.\n");
            return 0;
        }

        hr = getColorData(frame, color_img);
        if (FAILED(hr)) {
            INFO_PRINT("[INFO][KinectClient::receiveFrame] Unable to acquire color frame.\n");
            return 0;
        }
        
        //hr = getInfraredData(frame);
        //if (FAILED(hr)) {
        //    std::cerr << "[Error][KinectClient::update] Unable to acquire infrared frame."
        //        << std::endl;
        //}

        hr = getBodyData(frame, skeletons);
        if (FAILED(hr)) {
            INFO_PRINT("[INFO][KinectClient::receiveFrame] Unable to acquire body frame.\n");
            return 0;
        }
    }
    else {
        INFO_PRINT("[INFO][KinectClient::receiveFrame] New frame not ready. \n");
        return 0;
    }

    // Release frame
    if (frame) {
        frame->Release();
        frame = nullptr;
    }

    // New frame received
    return 1;
}

// Grabs and stores the depth frame
HRESULT KinectClient::getDepthData(IMultiSourceFrame* frame, 
                                   unsigned short *depth_img, 
                                   unsigned short* min_reliable_distance) {

    // Get color frame reference
    IDepthFrameReference* depth_frame_ref = nullptr;
    HRESULT hr = frame->get_DepthFrameReference(&depth_frame_ref);
    if (FAILED(hr)) {

        std::cerr << "[Error][KinectClient::getDepthData] Unable to get depth frame reference."
            << std::endl;
        return E_FAIL;
    }

    // Get depth frame
    IDepthFrame* depth_frame = nullptr;
    hr = depth_frame_ref->AcquireFrame(&depth_frame);
    if (SUCCEEDED(hr)) {

        // Get frame descriptor
        IFrameDescription* pFrameDescription = nullptr;
        hr = depth_frame->get_FrameDescription(&pFrameDescription);

        // Frame properties
        int width_d = 0;
        int height_d = 0;
        if (SUCCEEDED(hr)) {
            hr = pFrameDescription->get_Width(&width_d);
            hr = pFrameDescription->get_Height(&height_d);

            if (width_d != depth_frame_width || height_d != depth_frame_height) {
                std::cerr << "[Error][KinectClient::getDepthData] Depth frame "
                    "resolution different from expected resolution." << std::endl;
                return E_FAIL;
            }
        }

        hr = depth_frame->get_DepthMinReliableDistance(min_reliable_distance);
        // Get depth data
        if (SUCCEEDED(hr)) {
            unsigned int nBufferSize = 0;
            hr = depth_frame->AccessUnderlyingBuffer(&nBufferSize, &raw_depth_u16);
            memcpy(depth_img, raw_depth_u16, nBufferSize*sizeof(unsigned short));
        }

        if (pFrameDescription) {
            pFrameDescription->Release();
            pFrameDescription = nullptr;
        }
    }

    // Release frame resources
    if (depth_frame) {
        depth_frame->Release();
        depth_frame = nullptr;
    }
    if (depth_frame_ref) {
        depth_frame_ref->Release();
        depth_frame_ref = nullptr;
    }

    return hr;
}

// Grabs and stores the color frame
HRESULT KinectClient::getColorData(IMultiSourceFrame* frame, unsigned char *color_img) {

    // Get color frame reference
    IColorFrameReference* color_frame_ref = nullptr;
    HRESULT hr = frame->get_ColorFrameReference(&color_frame_ref);
    if (FAILED(hr)) {
        std::cerr << "[Error][KinectClient::getColorData] Unable to get color frame reference."
            << std::endl;
        return E_FAIL;
    }

    // Get color frame
    IColorFrame* color_frame;
    hr = color_frame_ref->AcquireFrame(&color_frame);
    if (SUCCEEDED(hr)) {

        // Get frame descriptor
        IFrameDescription* pFrameDescription_c = nullptr;
        hr = color_frame->get_FrameDescription(&pFrameDescription_c);

        // Frame properties
        if (SUCCEEDED(hr)) {
            int width_c = 0;
            int height_c = 0;
            pFrameDescription_c->get_Width(&width_c);
            pFrameDescription_c->get_Height(&height_c);
            pFrameDescription_c->get_VerticalFieldOfView(&color_vertical_fov);

            if (width_c != color_frame_width || height_c != color_frame_height) {
                std::cerr << "[Error][KinectClient::getColorData] Color frame "
                    "resolution different from expected resolution." << std::endl;
                return E_FAIL;
            }
        }
        if (pFrameDescription_c) {
            pFrameDescription_c->Release();
            pFrameDescription_c = nullptr;
        }

        // Get color fromat
        ColorImageFormat imageFormat_c = ColorImageFormat_None;
        hr = color_frame->get_RawColorImageFormat(&imageFormat_c);

        // Get color data
        if (SUCCEEDED(hr)) {

            static UINT nBufferSize = color_frame_size * sizeof(RGBQUAD);
            if (imageFormat_c == ColorImageFormat_Bgra) {
                hr = color_frame->CopyRawFrameDataToArray(nBufferSize, color_img);
            }
            else {
                hr = color_frame->CopyConvertedFrameDataToArray(nBufferSize, reinterpret_cast<BYTE*>(color_img), ColorImageFormat_Bgra);
            }
        }
    }
    else {
        INFO_PRINT("[INFO][KinectClient::getColorData] Unable to get color frame. \n");
    }

    // Release frame resources
    if (color_frame) {
        color_frame->Release();
        color_frame = nullptr;
    }
    if (color_frame_ref) {
        color_frame_ref->Release();
        color_frame_ref = nullptr;
    }

    return hr;
} /* Grabber::getColorData() */

// Grabs and stores the body frame
HRESULT KinectClient::getBodyData(IMultiSourceFrame* frame, 
                                  std::vector<BodySkeleton>& skeletons) {

    // Get bofy frame reference
    IBodyFrameReference* body_frame_ref = nullptr;
    HRESULT hr = frame->get_BodyFrameReference(&body_frame_ref);
    if (FAILED(hr)) {
        std::cerr << "[Error][KinectClient::getBodyData] Unable to get body frame reference."
            << std::endl;
        return E_FAIL;
    }

    // Get body frame
    IBodyFrame* body_frame;
    hr = body_frame_ref->AcquireFrame(&body_frame);
    if (SUCCEEDED(hr)) {

        // Get bodies
        IBody* bodies[MAX_BODIES] = { 0 };
        hr = body_frame->GetAndRefreshBodyData(MAX_BODIES, bodies);

        // Process collected bodies
        if (SUCCEEDED(hr)) {

            ProcessBodies(MAX_BODIES, bodies, skeletons);
            
            //After body processing is done, we're done with our bodies so release them.
            for (unsigned int bodyIndex = 0; bodyIndex < _countof(bodies); bodyIndex++) {
                if (bodies[bodyIndex]) {
                    bodies[bodyIndex]->Release();
                    bodies[bodyIndex] = nullptr;
                }
            }
        }
        else {
            INFO_PRINT("[INFO][KinectClient::getBodyData] No data to refresh. \n");
        }
    }
    else {
        INFO_PRINT("[INFO][KinectClient::getBodyData] Unable to get body frame. \n");
    }

    // Release frame resources
    if (body_frame) {
        body_frame->Release();
        body_frame = nullptr;
    }
    if (body_frame_ref) {
        body_frame_ref->Release();
        body_frame_ref = nullptr;
    }

    return hr;
}

// Process bodiws
void KinectClient::ProcessBodies(unsigned int body_count, IBody **bodies,
                                 std::vector<BodySkeleton>& skeletons) {

    size_t count = 0;
    for (unsigned int body_index = 0; body_index < body_count; body_index++) {
        
        IBody *body = bodies[body_index];

        // Get the tracking status for the body, if it's not tracked we'll skip it
        BOOLEAN is_tracked = false;
        HRESULT hr = body->get_IsTracked(&is_tracked);
        if (FAILED(hr) || is_tracked == false) {
            skeletons[body_index].is_tracked = false;
            continue;
        }
        
        // If we're here the body is tracked
        skeletons[body_index].is_tracked = true;
        body->get_TrackingId(&(skeletons[body_index].id));
        count++;
        
        // Get the joint properties for this skeleton
        Joint joints[JointType_Count];
        hr = body->GetJoints(_countof(joints), joints);
        if (SUCCEEDED(hr)) {
            
            // Process joints
            for (size_t j = 0; j < JointType_Count; ++j) {

                if (joints[j].TrackingState != TrackingState_Tracked) {
                    skeletons[body_index].joints[j].is_valid = false;
                    skeletons[body_index].joints[j].type = (JointType)j;
                }
                else {
                    DepthSpacePoint depth_point;
                    p_kinect_mapper->MapCameraPointToDepthSpace(joints[j].Position, &depth_point);
                    skeletons[body_index].joints[j].x = static_cast<int>(depth_point.X);
                    skeletons[body_index].joints[j].y = static_cast<int>(depth_point.Y);
                    skeletons[body_index].joints[j].type = (JointType)j;
                    skeletons[body_index].joints[j].is_valid = true;
                }
            }
        }
        else {
            INFO_PRINT("[INFO][KinectClient::ProcessBodies] Unable to get body joints. \n");
            skeletons[body_index].is_tracked = false;
        }
    }
}
