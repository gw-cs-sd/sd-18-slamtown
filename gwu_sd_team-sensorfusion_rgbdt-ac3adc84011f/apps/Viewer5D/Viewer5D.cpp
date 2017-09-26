//============================================================================
// Name        : Viewer5D.cpp
// Author      : CosmaC
// Date        : September, 2016
// Copyright   : GWU Research
// Description : 5D Viewer App: thermal, depth and color stream
//============================================================================

#include "3DSensor/3DSensor.h"
#include "ThermalSensor/ThermalSensor.h"

#include <opencv/cv.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

// App configuration
constexpr size_t width_d = 512;
constexpr size_t height_d = 424;
constexpr size_t width_c = 1920;
constexpr size_t height_c = 1080;
constexpr size_t bpp_c = 3;
constexpr ThermalSensorType type = LEPTON_V3;
constexpr size_t width_t = 160;
constexpr size_t height_t = 120;

constexpr bool do_FFC = true;
constexpr FRAME_TYPE frame_type = U16;

bool view_depth = true;
bool view_color = true;
bool view_temperature = true;

void main() {

    // Create sensors interfaces
    Sensor3D sensor_3d(width_d, height_d, width_c, height_c, bpp_c);
    ThermalSensor thermal_sensor(width_t, height_t, type);

    // config depth sensor
    bool hr_i = sensor_3d.start();
    if (!hr_i) {
        std::cerr << "[Error][Viewer5D] Unable to connect to the 3D sensing device."
            << std::endl;
        exit(0);
    }
    // config thermal sensor
    hr_i = thermal_sensor.start();
    if (!hr_i) {
        std::cerr << "[Error][Viewer5D] Unable to conect to the thermal imager device."
            << std::endl;
        exit(0);
    }
    char msg_u16[] = { FRAME_REQUEST, frame_type };
    thermal_sensor.sendMessage(msg_u16);

    // Run FFC
    if (do_FFC) {
        printf("[INFO] Run FFC... ");
        char msg_ffc[] = { I2C_CMD , FFC };
        thermal_sensor.sendMessage(msg_ffc);
        char msg_frame[] = { FRAME_REQUEST , frame_type };
        thermal_sensor.sendMessage(msg_frame);
        Sleep(1000);
        printf(" Done! \n");
    }

    // Prepare bufers
    std::vector<unsigned short> frame_depth(width_d * height_d);
    std::vector<unsigned char> frame_color(width_c * height_c * bpp_c);
    std::vector<unsigned char> frame_temperature(width_t * height_t);

    // Grab and show frames
    unsigned long long old_id_t = 9999999;
    unsigned long long old_id_cd = 9999999;
    unsigned long long new_id_t = 0;
    unsigned long long new_id_cd = 0;
    int nb_frames_t = 0;
    int nb_frames_cd = 0;
    int nb_all_frames = 0;
    int sensor_temp = -1;
    float object_temp = -1.0;
    clock_t begin = clock();
    while (1) {

        // Grab
        if (sensor_3d.hasNewFrame()) {
            //new_id = sensor_3d.nextFrame(frame_rgb, frame_depth.data(), frame_ir);
            new_id_cd = sensor_3d.nextFrame(frame_depth.data(), frame_color.data());
        }
        if (thermal_sensor.hasNewFrame()) {
            new_id_t = thermal_sensor.nextFrame(frame_temperature.data(), 
                sensor_temp, object_temp);
        }
        ++nb_all_frames;

        // Check for new frame
        if (old_id_cd != new_id_cd) {
            old_id_cd = new_id_cd;
            ++nb_frames_cd;

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
                cv::resize(image_color, image_color, cv::Size(image_color.cols/2, image_color.rows/2));
                cv::imshow("Color image", image_color);
            }
        }
        if (old_id_t != new_id_t) {
            old_id_t = new_id_t;
            ++nb_frames_t;

            // Show new frame
            cv::Mat temperature_image_u8(height_t, width_t, CV_8UC1, frame_temperature.data());
            cv::flip(temperature_image_u8, temperature_image_u8, 0);
            cv::resize(temperature_image_u8, temperature_image_u8, cv::Size(width_t * 4, height_t * 4));
            cv::applyColorMap(temperature_image_u8, temperature_image_u8, cv::COLORMAP_JET);
            cv::imshow("Temperature image", temperature_image_u8);
        }
        if (cv::waitKey(10) == 27) {
            break;
        }

        // Compute fps
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        if (elapsed_secs > 1.0) {
            
            double real_cd_fps = nb_frames_cd ? nb_frames_cd / elapsed_secs : 0;
            double real_t_fps = nb_frames_t ? nb_frames_t / elapsed_secs : 0;
            double processing_fps = nb_all_frames ? nb_all_frames / elapsed_secs : 0;
            printf("\r Kinect %f - Lepton %f - Processing %f", 
                real_cd_fps, real_t_fps, processing_fps);
            
            nb_frames_cd = 0;
            nb_frames_t = 0;
            nb_all_frames = 0;
            begin = end;
        }
    }

    // Stop 3D sensor
    hr_i = sensor_3d.stop();
    if (!hr_i) {
        std::cerr << "[Error][Viewer5D] Unable to close the 3D sensing device."
            << std::endl;
        exit(0);
    }
    hr_i = thermal_sensor.stop();
    if (!hr_i) {
        std::cerr << "[Error][Viewer5D] Unable to close the thermal imager device."
            << std::endl;
        exit(0);
    }
}