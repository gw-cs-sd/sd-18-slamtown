//============================================================================
// Name        : KinectGrabber.h
// Author      : CosmaC
// Date        : October, 2016
// Copyright   : GWU Research
// Description : Kinect grabber client API
//============================================================================

#pragma once

// Windows
#include <Kinect.h>

// C/C++
#include <vector>
#include <string>

// Debug option
#define INFO_PRINT //printf
#define PRINT_WARNING //printf

// Define maximum number of tracked bodies
constexpr size_t MAX_BODIES = BODY_COUNT;

// Structure for the skeleton joint definitation
struct SkeletonJoint {
    bool is_valid;
    JointType type; // from kinect.h
    int x;
    int y;
};

// Structure for the skeleton of a human body
// The order of the joints is pre-setted and is equivalent with the joint type
struct BodySkeleton {
    bool is_tracked;
    unsigned long long id;
    SkeletonJoint joints[JointType_Count];
};

class KinectClient {

public:

    /**
     * @brief Constructor/destructor
     */
    KinectClient(int width_d, int height_d, int width_c, int height_c) :
        count(0),
        depth_frame_width(width_d),
        depth_frame_height(height_d),
        color_frame_width(width_c),
        color_frame_height(height_c),
        color_vertical_fov(0.f) {

        depth_frame_size = depth_frame_width * depth_frame_height;
        color_frame_size = color_frame_width * color_frame_height;
    }

    ~KinectClient() {
        p_kinect_sensor = nullptr;
        p_kinect_reader = nullptr;
        p_kinect_mapper = nullptr;
    };

    /**
     * @brief Setters/Getters
     */
    inline int getDepthFrameWidth() { return depth_frame_width; }
    inline int getDepthFrameHeight() { return depth_frame_height; }

    /**
     * @brief Closes connection with the Kinect device
     */
    bool closeConnection();

    /**
     * @brief Opens a new connection with the Kinect device
     */
    bool openConnection();

    /**
     * @brief Receives a frame from Kinect.
     *
     * @param[out] depth_img              Raw depth frame (2 bytes per pixel)
     * @param[out] min_reliable_distance  Minimum depth for reliable distance measure
     * @param[out] color_img              Raw color image (RGBA - 4bytes per pixel)
     * @param[out] skeletons              Detected bodies skeletons
     *
     * @returns 1 - frame received
     *          0 - no frame from device
     *         -1 - connection lost
     */
    int receiveFrame(unsigned short *depth_img, unsigned short* min_reliable_distance,
        unsigned char *color_img, std::vector<BodySkeleton>& skeletons);

    /**
     * @brief Returns a pointer to the coordinate mapper pointer
     */
    ICoordinateMapper** const getCoordinateMapper() { return &p_kinect_mapper; };

    /**
     * @brief Returns color camera vertical FOV
     */
    float getColorVerticalFOV() { return color_vertical_fov; };

private:
    // Frame counter
    int count;
    
    // Depth frame
    int depth_frame_width;
    int depth_frame_height;
    int depth_frame_size;
    unsigned short* raw_depth_u16;      // Raw depth(z) data buffer

    // Color frame
    int color_frame_width;
    int color_frame_height;
    int color_frame_size;
    float color_vertical_fov;
    RGBQUAD* raw_color_RGBX;     // Raw RGB data buffer

    // Kinect driver and frame reader
    IKinectSensor*           p_kinect_sensor = nullptr;   // Sensor driver
    IMultiSourceFrameReader* p_kinect_reader = nullptr;   // Kinect frame grabber
    ICoordinateMapper*       p_kinect_mapper = nullptr;   // Converts between depth, color, and 3d coordinates

    /**
     * @brief  Grabs and stores the depth frame
     *
     * @param[in]  frame                  Multi-source pointer to the current frame
     * @param[out] depth_img              Raw depth frame
     * @param[out] min_reliable_distance  Minimum depth for reliable distance measure
     *
     * @returns  Indicates success or failure
     */
    HRESULT getDepthData(IMultiSourceFrame* frame,
                         unsigned short *depth_img,
                         unsigned short* min_reliable_distance);

    /**
     * @brief  Grabs and stores the color frame
     *
     * @param[in]  frame     Multi-source pointer to the current frame
     * @param[out] color_img Raw color image (RGBA - 4bytes per pixel)
     *
     * @returns  Indicates success or failure
     */
    HRESULT getColorData(IMultiSourceFrame* frame, unsigned char *color_img);

    /**
     * @brief  Grabs and stores the detected bodies
     *
     * @param[in]  frame     Multi-source pointer to the current frame
     * @param[out] skeletons Detected bodies skeletons
     *
     * @returns  Indicates success or failure
     */
    HRESULT getBodyData(IMultiSourceFrame* frame, std::vector<BodySkeleton>& skeletons);

    /**
     * @brief  Extracts the body skeletons given the tracked bodies
     *
     * @param[in]  body_count  Max number of bodies tracked
     * @param[in]  bodies      Tracked bodies
     * @param[out] skeletons   Detected bodies skeletons
     */
    void KinectClient::ProcessBodies(unsigned int body_count, IBody **bodies, 
                                     std::vector<BodySkeleton>& skeletons);
};
