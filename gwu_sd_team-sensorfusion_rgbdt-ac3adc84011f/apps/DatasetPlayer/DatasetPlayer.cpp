//============================================================================
// Name        : DatasetPlayer.cpp
// Author      : CosmaC
// Date        : November, 2016
// Copyright   : GWU Research
// Description : Dataset player helps users visualize offline data, such as
//               recorded datasets
//============================================================================

#include "SensorFusion/SensorFusionSync.h"
#include <ImageProcessing/ImageUtils.h>

// C/C++
#include <string>

// OpenCV
#include <opencv/cv.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// App configuration
bool view_depth = true;
bool view_color = true;
bool view_temperature = true;
bool view_skeleton = true;

void main(int argc, char* argv[]) {

    if (argc == 1) {
        std::cout << "\n[Error] Please specify the dataset path.\n"
            "DatasetPlayer.exe path_to_dataset" << std::endl;
        exit(0);
    }

    // Create sensors interfaces
    SensorFusionSync sensor_fusion(argv[1], 1);

    // config depth sensor
    bool hr_i = sensor_fusion.start(false, true);
    if (!hr_i) {
        std::cerr << "[Error][SensorFusion] Unable to connect to the 3D sensing device."
            << std::endl;
        exit(0);
    }

    // Prepare bufers
    RawFrame frame;
    frame.width_d = sensor_fusion.getDepthWidth();
    frame.height_d = sensor_fusion.getDepthHeight();
    frame.width_c = sensor_fusion.getColorWidth();
    frame.height_c = sensor_fusion.getColorHeight();
    frame.width_t = sensor_fusion.getTemperatureWidth();
    frame.height_t = sensor_fusion.getTemperatureHeight();
    frame.frame_depth.resize(sensor_fusion.getDepthSize());
    frame.frame_color.resize(sensor_fusion.getColorSize());
    frame.frame_temperature.resize(sensor_fusion.getTemperatureSize());
    frame.skeletons.resize(MAX_BODIES);

    Frame reg_frame;
    reg_frame.data.resize(frame.width_d * frame.height_d);
    reg_frame.mask.resize(frame.width_d * frame.height_d);
    reg_frame.data_3D.resize(frame.width_d * frame.height_d);
    reg_frame.width = frame.width_d;
    reg_frame.height = frame.height_d;

    // Grab and show frames
    unsigned long long old_id = 9999999;
    unsigned long long new_id = 0;
    int status = 0;
    int nb_frames = 0;
    int nb_all_frames = 0;
    clock_t begin = clock();
    while (1) {

        // Grab
        if (sensor_fusion.HasNewFrame()) {
            new_id = sensor_fusion.NextFrame(frame, reg_frame, status);
            if (status == 0) {
                break;
            }
        }
        ++nb_all_frames;

        // Check for new frame
        if (old_id != new_id) {
            old_id = new_id;
            ++nb_frames;

            // Show depth
            if (view_depth) {
                int range = 500; // 5m
                cv::Mat image_depth;
                FrameToDepthImage(reg_frame, image_depth, range);
                //CreateImageU8ByRangeNormalization(frame.frame_depth,
                //    frame.width_d, frame.height_d, range, image_depth);
                cv::imshow("Depth image", image_depth);
            }
            // Show color
            if (view_color) {
                cv::Mat image_color;
                FrameToColorImage(reg_frame, image_color);
                cv::imshow("Color image", image_color);
            }
            // Show temperature
            if (view_temperature) {
                const float min_th = 10.f;
                const float max_th = 40.f;
                cv::Mat temperature_image;
                FrameToThermalImage(reg_frame, temperature_image, min_th, max_th, true);
                cv::imshow("Temperature image", temperature_image);
            }
            // Show skeletons
            if (view_skeleton) {
                int range = 500;
                cv::Mat image_depth;
                FrameToDepthImage(reg_frame, image_depth, range);
                cv::Mat image_skeleton(reg_frame.height, reg_frame.width, CV_8UC3);
                cv::cvtColor(image_depth, image_skeleton, CV_GRAY2BGR);
                DrawSkeletons(frame.skeletons, image_skeleton);
                cv::imshow("Skeleton image", image_skeleton);
            }
        }
        char key = cv::waitKey(10);
        if (key == 27) {
            cv::destroyAllWindows();
            break;
        }

        // Compute fps
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        if (elapsed_secs > 1.0) {
            
            double real_fps = nb_frames ? nb_frames / elapsed_secs : 0;
            double processing_fps = nb_all_frames ? nb_all_frames / elapsed_secs : 0;
            printf("\r Sensors %f - Processing %f", real_fps, processing_fps);
            
            nb_frames = 0;
            nb_all_frames = 0;
            begin = end;
        }
    }

    // Stop sensors
    hr_i = sensor_fusion.stop();
    if (!hr_i) {
        std::cerr << "[Error][ViewerFusion] Unable to close the thermal imager device."
            << std::endl;
        exit(0);
    }
}
