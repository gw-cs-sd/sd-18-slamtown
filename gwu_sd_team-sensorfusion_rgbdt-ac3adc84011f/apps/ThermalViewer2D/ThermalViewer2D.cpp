//============================================================================
// Name        : ThermalViewer2D.cpp
// Author      : CosmaC
// Date        : September, 2016
// Copyright   : GWU Research
// Description : 2D Thermal Viewer App
//============================================================================

#include "ThermalSensor/ThermalSensor.h"

#include <opencv/cv.h>
#include <opencv2/highgui.hpp>

// App configuration
constexpr ThermalSensorType type = LEPTON_V3;
constexpr size_t width = 160;
constexpr size_t height = 120;
constexpr bool do_FFC = true;
constexpr FRAME_TYPE frame_type = U16;

void main() {

    ThermalSensor thermal_sensor(width, height, type);
    int frame_width = thermal_sensor.getFrameWidth();
    int frame_height = thermal_sensor.getFrameHeight();
    int frame_szie = frame_height * frame_width;
    unsigned short* temperature_u16 = new unsigned short[frame_szie]; // Raw temperature buffer
    unsigned char* temperature_u8 = new unsigned char[frame_szie];      // Post-processed temperature buffer
    int sensor_temp = -1;
    float object_temp = -1.0;

    // config sensor
    bool hr_i = thermal_sensor.start();
    if (!hr_i) {
        std::cerr << "[Error][ThermalViewer] Unable to conect to the FLIR device."
            << std::endl;
        exit(0);
    }
    else {
        char msg_u16[] = { FRAME_REQUEST , frame_type };
        thermal_sensor.sendMessage(msg_u16);
    }

    // Run FFC
    if (do_FFC) {
        printf("[INFO] Run FFC... ");
        char msg_ffc[] = { I2C_CMD , FFC };
        thermal_sensor.sendMessage(msg_ffc);
        char msg_frame[] = { FRAME_REQUEST , frame_type };
        thermal_sensor.sendMessage(msg_frame);
        Sleep(2000);
        printf(" Done! \n");
    }

    // Grab and show frames
    unsigned long long old_id = 9999999;
    unsigned long long new_id = 0;
    int nb_real_frames = 0;
    int nb_all_frames = 0;
    clock_t begin = clock();
    while (1) {

        // Grab
        if (thermal_sensor.hasNewFrame()) {
            new_id = thermal_sensor.nextFrame(temperature_u8, sensor_temp, 
                object_temp, true);
        }
        ++nb_all_frames;

        // Check for new frame
        if (old_id != new_id) {
            old_id = new_id;
            ++nb_real_frames;

            // Show new frame
            cv::Mat temperature_image_u8(height, width, CV_8UC1, temperature_u8);
            cv::flip(temperature_image_u8, temperature_image_u8, 0);
            cv::imshow("Temperature image", temperature_image_u8);
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
            printf("\r Real Speed %f - Processing Speed %f - Temp %f", 
                real_fps, processing_fps, object_temp);
            
            nb_real_frames = 0;
            nb_all_frames = 0;
            begin = end;
        }
    }
    
    // Stop and clean
    thermal_sensor.stop();
    delete[] temperature_u8;
    delete[] temperature_u16;
}