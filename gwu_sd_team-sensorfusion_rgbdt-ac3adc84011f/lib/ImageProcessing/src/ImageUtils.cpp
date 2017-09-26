//============================================================================
// Name        : ImageUtils.cpp
// Author      : CosmaC
// Date        : October, 2016
// Copyright   : GWU Research
// Description : Image processing utils
//============================================================================

#include <ImageProcessing/ImageUtils.h>
#include <KeyPatch/KeyPatch.h>

// C/C++
#include <iostream>

// OpenCV
#include <opencv/cv.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


/**
 * @brief Creates an U8 image given pixels data and image resolution,
 *        by min max normalization
 */
template< typename T >
void CreateImageU8ByMinMaxNormalization(std::vector<T>& img_data,
                                        size_t width, size_t height,
                                        cv::Mat& img, 
                                        T min, T max) {

    // check data correctness
    if (img_data.size() != width * height) {
        std::cout << "[WARNING] Buffer size do not match the image resolution."
            " Skip image creation." << std::endl;
        return;
    }

    // Create image 
    if (img.rows != height || img.cols != width ||
        img.type() != CV_8UC1) { // recreate image

        img = cv::Mat(height, width, CV_8UC1);
    }
    unsigned char* dst = img.data;

    // Compute min and max
    T minValue = min;
    T maxValue = max;
    if (std::abs(minValue - maxValue) < 1e-12) { // Automatically compute min and max
        minValue = img_data[0];
        maxValue = img_data[0];
        for (size_t idx = 1; idx < img_data.size(); ++idx) {
            if (img_data[idx] > maxValue) {
                maxValue = img_data[idx];
            }
            if (img_data[idx] < minValue) {
                minValue = img_data[idx];
            }
        }
    }
    
    // Populate with values
    const double scale = 255.0 / (maxValue - minValue);
    for (size_t idx = 0; idx < img_data.size(); ++idx) {
        if (img_data[idx] <= minValue) {
            dst[idx] = 0;
        }
        else if (img_data[idx] >= maxValue) {
            dst[idx] = 255;
        }
        else {
            dst[idx] = static_cast<unsigned char>((img_data[idx] - minValue) * scale);
        }
    }
}

/**
 * @brief Creates an U8 image given pixels data and image resolution,
 *        by range normalization
 */
void CreateImageU8ByRangeNormalization(std::vector<unsigned short>& img_data,
                                       size_t width, size_t height, 
                                       int segment_range,
                                       cv::Mat& img) {

    // check data correctness
    if (img_data.size() != width * height) {
        std::cout << "[WARNING] Buffer size do not match the image resolution."
            " Skip image creation." << std::endl;
        return;
    }

    // Create image 
    if (img.rows != height || img.cols != width ||
        img.type() != CV_8UC1) { // recreate image

        img = cv::Mat(height, width, CV_8UC1);
    }
    unsigned char* dst = img.data;

    // Populate with values
    double scale = (255.0 / segment_range);
    if (0 == scale) {
        scale = 1;
    }
    for (size_t idx = 0; idx < img_data.size(); ++idx) {
        dst[idx] = static_cast<unsigned char>((img_data[idx] % segment_range) * scale);
    }
}

/**
 * @brief Creates a 3 chanels U8 color image given a frame
 */
void FrameToColorImage(const Frame& frame,
                       cv::Mat& img) {

    if (img.rows != frame.height || img.cols != frame.width ||
        img.type() != CV_8UC3) { // recreate image

        img = cv::Mat(frame.height, frame.width, CV_8UC3);
    }

    unsigned char* img_data = img.data;
    for (size_t idx = 0; idx < frame.data.size(); ++idx) {
        img_data[3 * idx] = frame.data[idx].r;
        img_data[3 * idx + 1] = frame.data[idx].g;
        img_data[3 * idx + 2] = frame.data[idx].b;
    }
}

/**
 * @brief Creates a 1 chanel U8 depth image given a frame
 */
void FrameToDepthImage(const Frame& frame,
                       cv::Mat& img,
                       int range) {

    if (img.rows != frame.height || img.cols != frame.width ||
        img.type() != CV_8UC1) { // recreate image

        img = cv::Mat(frame.height, frame.width, CV_8UC1);
    }

    std::vector<unsigned short> depth_data(frame.data.size());
    for (size_t idx = 0; idx < frame.data.size(); ++idx) {
        depth_data[idx] = frame.data[idx].depth;
    }

    CreateImageU8ByRangeNormalization(depth_data,
        frame.width, frame.height, range, img);
}

/**
 * @brief Creates a 1 channels U8 (or 3 channels) thermal image given a frame
 */
void FrameToThermalImage(const Frame& frame,
                         cv::Mat& img,
                         float min_th, float max_th,
                         bool apply_coloring) {

    if (img.rows != frame.height || img.cols != frame.width ||
        (img.type() != CV_8UC1 && !apply_coloring) || 
        (img.type() != CV_8UC3 && apply_coloring)) { // recreate image

        if (apply_coloring) {
            img = cv::Mat(frame.height, frame.width, CV_8UC3);
        }
        else {
            img = cv::Mat(frame.height, frame.width, CV_8UC1);
        }
    }

    std::vector<float> temp_data(frame.data.size());
    for (size_t idx = 0; idx < frame.data.size(); ++idx) {
        temp_data[idx] = frame.data[idx].temperature;
    }
    
    if (apply_coloring) {
        cv::Mat temp_img;
        CreateImageU8ByMinMaxNormalization(temp_data, frame.width, frame.height,
            temp_img, min_th, max_th);
        cv::applyColorMap(temp_img, img, cv::COLORMAP_JET);
    }
    else {
        CreateImageU8ByMinMaxNormalization(temp_data, frame.width, frame.height,
            img, min_th, max_th);
    }
}

/**
 * @brief Given the skeletons array and a u8 3 channels image, the function
 *        draws the skeletons on top of the image
 */
void DrawSkeletons(const std::vector<BodySkeleton>& skeletons,
                   cv::Mat& image) {

    // checks input data
    if (skeletons.empty() || image.empty() ||
        image.type() != CV_8UC3) {
        
        PRINT_WARNING("[WARNING] Skeletons %d - Image type %d. \n", skeletons.size(), image.type());
        return;
    }

    // Draw skeleton points
    for (const auto& skeleton : skeletons) {
        if (!skeleton.is_tracked) {
            continue;
        }
        for (const auto& joint : skeleton.joints) {
            if (joint.is_valid) {
                cv::circle(image, cv::Point(joint.x, joint.y), 2, cv::Scalar(0, 255, 0, 0));
            }
        }
    }

    // Draw skeleton links
    const cv::Scalar link_color(0.0, 0.0, 255.0);
    const int link_thickness = 1;
    for (const auto& skeleton : skeletons) {
        if (!skeleton.is_tracked) {
            continue;
        }
        // Right foot
        if (skeleton.joints[JointType_FootRight].is_valid &&
            skeleton.joints[JointType_AnkleRight].is_valid) {
            cv::line(image, 
                cv::Point(skeleton.joints[JointType_FootRight].x, skeleton.joints[JointType_FootRight].y),
                cv::Point(skeleton.joints[JointType_AnkleRight].x, skeleton.joints[JointType_AnkleRight].y),
                link_color, link_thickness);
        }
        // Right leg
        if (skeleton.joints[JointType_KneeRight].is_valid &&
            skeleton.joints[JointType_AnkleRight].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_KneeRight].x, skeleton.joints[JointType_KneeRight].y),
                cv::Point(skeleton.joints[JointType_AnkleRight].x, skeleton.joints[JointType_AnkleRight].y),
                link_color, link_thickness);
        }
        if (skeleton.joints[JointType_KneeRight].is_valid &&
            skeleton.joints[JointType_HipRight].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_KneeRight].x, skeleton.joints[JointType_KneeRight].y),
                cv::Point(skeleton.joints[JointType_HipRight].x, skeleton.joints[JointType_HipRight].y),
                link_color, link_thickness);
        }
        
        // Left foot
        if (skeleton.joints[JointType_FootLeft].is_valid &&
            skeleton.joints[JointType_AnkleLeft].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_FootLeft].x, skeleton.joints[JointType_FootLeft].y),
                cv::Point(skeleton.joints[JointType_AnkleLeft].x, skeleton.joints[JointType_AnkleLeft].y),
                link_color, link_thickness);
        }
        // Left leg
        if (skeleton.joints[JointType_KneeLeft].is_valid &&
            skeleton.joints[JointType_AnkleLeft].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_KneeLeft].x, skeleton.joints[JointType_KneeLeft].y),
                cv::Point(skeleton.joints[JointType_AnkleLeft].x, skeleton.joints[JointType_AnkleLeft].y),
                link_color, link_thickness);
        }
        if (skeleton.joints[JointType_KneeLeft].is_valid &&
            skeleton.joints[JointType_HipLeft].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_KneeLeft].x, skeleton.joints[JointType_KneeLeft].y),
                cv::Point(skeleton.joints[JointType_HipLeft].x, skeleton.joints[JointType_HipLeft].y),
                link_color, link_thickness);
        }

        // Torso
        if (skeleton.joints[JointType_SpineBase].is_valid &&
            skeleton.joints[JointType_HipLeft].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_SpineBase].x, skeleton.joints[JointType_SpineBase].y),
                cv::Point(skeleton.joints[JointType_HipLeft].x, skeleton.joints[JointType_HipLeft].y),
                link_color, link_thickness);
        }
        if (skeleton.joints[JointType_SpineBase].is_valid &&
            skeleton.joints[JointType_HipRight].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_SpineBase].x, skeleton.joints[JointType_SpineBase].y),
                cv::Point(skeleton.joints[JointType_HipRight].x, skeleton.joints[JointType_HipRight].y),
                link_color, link_thickness);
        }
        if (skeleton.joints[JointType_SpineBase].is_valid &&
            skeleton.joints[JointType_SpineMid].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_SpineBase].x, skeleton.joints[JointType_SpineBase].y),
                cv::Point(skeleton.joints[JointType_SpineMid].x, skeleton.joints[JointType_SpineMid].y),
                link_color, link_thickness);
        }
        
        // Shoulders
        if (skeleton.joints[JointType_SpineShoulder].is_valid &&
            skeleton.joints[JointType_SpineMid].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_SpineShoulder].x, skeleton.joints[JointType_SpineShoulder].y),
                cv::Point(skeleton.joints[JointType_SpineMid].x, skeleton.joints[JointType_SpineMid].y),
                link_color, link_thickness);
        }
        if (skeleton.joints[JointType_SpineShoulder].is_valid &&
            skeleton.joints[JointType_ShoulderRight].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_SpineShoulder].x, skeleton.joints[JointType_SpineShoulder].y),
                cv::Point(skeleton.joints[JointType_ShoulderRight].x, skeleton.joints[JointType_ShoulderRight].y),
                link_color, link_thickness);
        }
        if (skeleton.joints[JointType_SpineShoulder].is_valid &&
            skeleton.joints[JointType_ShoulderLeft].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_SpineShoulder].x, skeleton.joints[JointType_SpineShoulder].y),
                cv::Point(skeleton.joints[JointType_ShoulderLeft].x, skeleton.joints[JointType_ShoulderLeft].y),
                link_color, link_thickness);
        }

        // Right arm        
        if (skeleton.joints[JointType_ElbowRight].is_valid &&
            skeleton.joints[JointType_ShoulderRight].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_ElbowRight].x, skeleton.joints[JointType_ElbowRight].y),
                cv::Point(skeleton.joints[JointType_ShoulderRight].x, skeleton.joints[JointType_ShoulderRight].y),
                link_color, link_thickness);
        }
        if (skeleton.joints[JointType_ElbowRight].is_valid &&
            skeleton.joints[JointType_WristRight].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_ElbowRight].x, skeleton.joints[JointType_ElbowRight].y),
                cv::Point(skeleton.joints[JointType_WristRight].x, skeleton.joints[JointType_WristRight].y),
                link_color, link_thickness);
        }
        // Right hand
        if (skeleton.joints[JointType_HandRight].is_valid &&
            skeleton.joints[JointType_WristRight].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_HandRight].x, skeleton.joints[JointType_HandRight].y),
                cv::Point(skeleton.joints[JointType_WristRight].x, skeleton.joints[JointType_WristRight].y),
                link_color, link_thickness);
        }

        // Left arm        
        if (skeleton.joints[JointType_ElbowLeft].is_valid &&
            skeleton.joints[JointType_ShoulderLeft].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_ElbowLeft].x, skeleton.joints[JointType_ElbowLeft].y),
                cv::Point(skeleton.joints[JointType_ShoulderLeft].x, skeleton.joints[JointType_ShoulderLeft].y),
                link_color, link_thickness);
        }
        if (skeleton.joints[JointType_ElbowLeft].is_valid &&
            skeleton.joints[JointType_WristLeft].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_ElbowLeft].x, skeleton.joints[JointType_ElbowLeft].y),
                cv::Point(skeleton.joints[JointType_WristLeft].x, skeleton.joints[JointType_WristLeft].y),
                link_color, link_thickness);
        }
        // Left hand
        if (skeleton.joints[JointType_HandLeft].is_valid &&
            skeleton.joints[JointType_WristLeft].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_HandLeft].x, skeleton.joints[JointType_HandLeft].y),
                cv::Point(skeleton.joints[JointType_WristLeft].x, skeleton.joints[JointType_WristLeft].y),
                link_color, link_thickness);
        }

        // Head and neck
        if (skeleton.joints[JointType_SpineShoulder].is_valid &&
            skeleton.joints[JointType_Neck].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_SpineShoulder].x, skeleton.joints[JointType_SpineShoulder].y),
                cv::Point(skeleton.joints[JointType_Neck].x, skeleton.joints[JointType_Neck].y),
                link_color, link_thickness);
        }
        if (skeleton.joints[JointType_Head].is_valid &&
            skeleton.joints[JointType_Neck].is_valid) {
            cv::line(image,
                cv::Point(skeleton.joints[JointType_Head].x, skeleton.joints[JointType_Head].y),
                cv::Point(skeleton.joints[JointType_Neck].x, skeleton.joints[JointType_Neck].y),
                link_color, link_thickness);
        }
    }
}

/**
 * @brief Draws the keypatches on top of the detected bodies
 */
void DrawKeypatches(const std::vector<BodyThermalModel>& bodies,
                    cv::Mat& image) {

    // Prepare buffers
    unsigned char* data = image.data;

    // Parse detected bodies
    for (const auto& body : bodies) {

        // Parse detected patches of the body
        for (size_t p = 0; p < body.patches.size(); ++p) {

            const auto& patch = body.patches[p];

            // Skip invalid patches
            if (!patch.is_valid) {
                continue;
            }

            // Draw patch area
            for (size_t idx = 0; idx < patch.pt_x.size(); ++idx) {
                data[patch.pt_x[idx] * 3 + patch.pt_y[idx] * image.cols * 3] = keypatch_color_table[3 * p];
                data[patch.pt_x[idx] * 3 + 1 + patch.pt_y[idx] * image.cols * 3] = keypatch_color_table[3 * p + 1];
                data[patch.pt_x[idx] * 3 + 2 + patch.pt_y[idx] * image.cols * 3] = keypatch_color_table[3 * p + 2];
            }

            // Draw patch avg temp
            int font_face = cv::FONT_HERSHEY_PLAIN;
            double font_scale = 0.8;
            char label[50];
            sprintf(label, "%.1f(%.1f)", patch.temperature_avg, patch.temperature_variance);
            cv::putText(image, label, cv::Point(patch.pt_x[0], patch.pt_y[0]), 
                font_face, font_scale, cv::Scalar(0.0,0.0,255.0));
        }
    }
}

/**
 * @brief Displays desired images given the frame
 */
const std::string status_str[3] = {"WARM UP", "PAUSE", "COLLECT"};
char ViewFrame(const Frame& reg_frame, const RawFrame& raw_frame,
               const VisualizationConfig& config) {

    // Show depth frame
    if (config.view_depth) {
        int range = 500; // 5m
        cv::Mat image_depth;
        FrameToDepthImage(reg_frame, image_depth, range);
        cv::imshow("Depth image", image_depth);
    }
    // Show color
    if (config.view_color) {
        cv::Mat image_color;
        FrameToColorImage(reg_frame, image_color);
        
        char status[255];
        sprintf(status, "Status: %s", status_str[config.status].c_str());
        cv::putText(image_color, status, cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN,
            1, cv::Scalar(255, 255, 255));
        
        char temp[255];
        sprintf(temp, "Sensor T = %d", config.thermal_sensor_temperature);
        cv::putText(image_color, temp, cv::Point(10, 35), cv::FONT_HERSHEY_PLAIN,
            1, cv::Scalar(255, 255, 255));

        cv::imshow("Color image", image_color);
    }
    // Show temperature frame
    if (config.view_temperature) {
        const float min_th = 10.f;
        const float max_th = 40.f;
        cv::Mat temperature_image;
        FrameToThermalImage(reg_frame, temperature_image, min_th, max_th, true);

        if (config.center_temperature > 0.f) {
            cv::Point center(temperature_image.cols / 2, temperature_image.rows / 2);
            cv::circle(temperature_image, center, 2, cv::Scalar(0, 0, 0));
            char label[20];
            sprintf(label, "%.3f", config.center_temperature);
            cv::putText(temperature_image, label, center, cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 0));
        }
        cv::imshow("Temperature image", temperature_image);
    }
    // Show skeletons
    if (config.view_skeleton) {
        int range = 500;
        cv::Mat image_depth;
        FrameToDepthImage(reg_frame, image_depth, range);
        cv::Mat image_skeleton(reg_frame.height, reg_frame.width, CV_8UC3);
        cv::cvtColor(image_depth, image_skeleton, CV_GRAY2BGR);
        DrawSkeletons(raw_frame.skeletons, image_skeleton);
        cv::imshow("Skeleton image", image_skeleton);
    }

    return cv::waitKey(10);
}

/**
 * @brief Displays desired images given the frame
 */
char ViewFrame(const Frame& reg_frame, const RawFrame& raw_frame,
               const std::vector<BodyThermalModel>& bodies,
               const VisualizationConfig& config) {

    // Show key patches 
    if (config.view_keypatches) {

        cv::Mat image_keypath;
        FrameToColorImage(reg_frame, image_keypath);
        DrawKeypatches(bodies, image_keypath);
        cv::imshow("Keypatches image", image_keypath);
    }

    // Call overloaded function for rest of the images
    return ViewFrame(reg_frame, raw_frame, config);
}

///////////////////////////////////////////////////////////////////////////////
// Templates
///////////////////////////////////////////////////////////////////////////////
template void CreateImageU8ByMinMaxNormalization(
    std::vector<double>& img_data,
    size_t width, size_t height,
    cv::Mat& img,
    double min, double max);

template void CreateImageU8ByMinMaxNormalization(
    std::vector<float>& img_data,
    size_t width, size_t height,
    cv::Mat& img,
    float min, float max);

template void CreateImageU8ByMinMaxNormalization(
    std::vector<unsigned short>& img_data,
    size_t width, size_t height,
    cv::Mat& img,
    unsigned short min, unsigned short max);
