//============================================================================
// Name        : Viewer3D.cpp
// Author      : CosmaC
// Date        : September, 2016
// Copyright   : GWU Research
// Description : 3D Viewer App for Kinect like devices
//============================================================================

#include "3DSensor/3DSensor.h"

#include <opencv/cv.h>
#include <opencv2/highgui.hpp>

bool view_depth = true;
bool view_color = false;

void main() {

    constexpr size_t width_d = 512;
    constexpr size_t height_d = 424;
    constexpr size_t width_c = 1920;
    constexpr size_t height_c = 1080;
    constexpr size_t bpp_c = 3;
    Sensor3D sensor_3d(width_d, height_d, width_c, height_c, bpp_c);

    // config sensor
    bool hr_i = sensor_3d.start();
    if (!hr_i) {
        std::cerr << "[Error][Viewer3D] Unable to connect to the 3D device."
            << std::endl;
        exit(0);
    }

    // Prepare bufers
    std::vector<unsigned short> frame_depth(width_d*height_d);
    std::vector<unsigned char> frame_color(width_c*height_c*bpp_c);

    // Grab and show frames
    unsigned long long old_id = 9999999;
    unsigned long long new_id = 0;
    int nb_real_frames = 0;
    int nb_all_frames = 0;
    clock_t begin = clock();
    while (1) {

        // Grab
        if (sensor_3d.hasNewFrame()) {
            //new_id = sensor_3d.nextFrame(frame_rgb, frame_depth.data(), frame_ir);
            new_id = sensor_3d.nextFrame(frame_depth.data(), frame_color.data());
        }
        ++nb_all_frames;

        // Check for new frame
        if (old_id != new_id) {
            old_id = new_id;
            ++nb_real_frames;

            // Show new frame
            if (view_depth) {
                cv::Mat image_depth(height_d, width_d, CV_8UC1);
                unsigned short* src = frame_depth.data();
                unsigned char* dst = image_depth.data;
                for (size_t idx = 0; idx < frame_depth.size(); ++idx) {
                    dst[idx] = (src[idx] % 500) / 2;
                }
                cv::imshow("Depth image", image_depth);
            }
            if (view_color) {
                cv::Mat image_color(height_c, width_c, CV_8UC3, frame_color.data());
                cv::imshow("Color image", image_color);
            }
        }
        if (cv::waitKey(10) == 27) {
            break;
        }

        // Compute fps
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        if (elapsed_secs > 1.0) {
            
            double real_fps = nb_real_frames / elapsed_secs;
            double processing_fps = nb_all_frames / elapsed_secs;
            printf("\r Real Speed %f - Processing Speed %f", real_fps, processing_fps);
            
            nb_real_frames = 0;
            nb_all_frames = 0;
            begin = end;
        }
    }

    // Stop 3D sensor
    hr_i = sensor_3d.stop();
    if (!hr_i) {
        std::cerr << "[Error][Viewer3D] Unable to close the 3D device."
            << std::endl;
        exit(0);
    }
}