//============================================================================
// Name        : ImageUtils.h
// Author      : CosmaC
// Date        : October, 2016
// Copyright   : GWU Research
// Description : Image processing utils
//============================================================================

#pragma once

#include <SensorFusion/SensorFusionSync.h>
#include <KeyPatch/KeyPatch.h>

// OpenCV
#include <opencv/cv.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// Configuration structure to control the frame visualization
struct VisualizationConfig {
    bool view_depth = true;         // Display depth image
    bool view_color = true;         // Display color image
    bool view_temperature = true;   // Display thermal image
    bool view_skeleton = true;      // Display detected bodies
    bool view_keypatches = true;    // Display detected keypatches
    int thermal_sensor_temperature = 0;
    int status = 0;                 // System status (WARM_UP, PAUSE, COLLEC)
    float center_temperature = 0.f; // Temeprature of the pixek in the middle of the image

    VisualizationConfig(bool w_d, bool w_c, bool w_t, bool w_s, bool w_k, int t, int s)
        : view_depth(w_d),
        view_color(w_c),
        view_temperature(w_t),
        view_skeleton(w_s),
        view_keypatches(w_k),
        thermal_sensor_temperature(t),
        status(s) {};
};

/**
 * @brief Creates an U8 image given pixels data and image resolution,
 *        by min max normalization
 */
template< typename T >
void CreateImageU8ByMinMaxNormalization(std::vector<T>& img_data,
                                        size_t width, size_t height,
                                        cv::Mat& img,
                                        T min = 0, T max = 0);

/**
 * @brief Creates an U8 image given pixels data and image resolution,
 *        by range normalization
 */
void CreateImageU8ByRangeNormalization(std::vector<unsigned short>& img_data,
                                       size_t width, size_t height, 
                                       int segment_range,
                                       cv::Mat& img);

/**
 * @brief Creates a 3 channels U8 color image given a frame
 */
void FrameToColorImage(const Frame& frame,
                       cv::Mat& img);

/**
* @brief Creates a 1 channel U8 depth image given a frame
*/
void FrameToDepthImage(const Frame& frame,
                       cv::Mat& img,
                       int range);

/**
 * @brief Creates a 1 channels U8 (or 3 channels) thermal image given a frame
 * 
 * Note: min and max threshold are automatically computed by default
 */
void FrameToThermalImage(const Frame& frame,
                         cv::Mat& img,
                         float min_th = 0.f, float max_th = 0.f,
                         bool apply_coloring = false);

/**
 * @brief Given the skeletons array and a u8 3 channels image, the function
 *        draws the skeletons on top of the image
 *
 * Note: image shoulde be u8 3 channels
 */
void DrawSkeletons(const std::vector<BodySkeleton>& skeletons,
                   cv::Mat& image);

/**
 * @brief Displays desired images given the frame
 */
char ViewFrame(const Frame& reg_frame, const RawFrame& raw_frame,
               const VisualizationConfig& config);

char ViewFrame(const Frame& reg_frame, const RawFrame& raw_frame,
               const std::vector<BodyThermalModel>& bodies,
               const VisualizationConfig& config);
