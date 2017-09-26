//============================================================================
// Name        : ThermalCalibration.cpp
// Author      : CosmaC
// Date        : October, 2016
// Copyright   : GWU Research
// Description : Thermal camera calibration app
//============================================================================

#include "ThermalSensor/ThermalSensor.h"
#include "ThermalSensor/ThermalCalibration.h"

// OpenCV
#include <opencv/cv.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


// Mouse callback
int click_x = -1;
int click_y = -1;
static void onMouse(int event, int x, int y, int, void*) {

    if (event != cv::EVENT_LBUTTONDOWN) {
        return;
    }

    click_x = x;
    click_y = y;
}


// Test sensor calibration
void TestCalibration(ThermalSensor& thermal_sensor, ThermalCalibration& thermal_calib) {

    // create frame buffers
    const int frame_width = thermal_sensor.getFrameWidth();
    const int frame_height = thermal_sensor.getFrameHeight();
    const int frame_szie = frame_height * frame_width;
    unsigned short* temperature_u16 = new unsigned short[frame_szie]; // Raw temperature buffer
    unsigned char* temperature_u8 = new unsigned char[frame_szie];      // Post-processed temperature buffer
    
    // Grab frames
    int T_s = 0;
    float T_o = 0;
    float object_temp = 0;
    char label[255];
    cv::destroyAllWindows();
    cv::namedWindow("Temperature", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("Temperature", onMouse, 0);
    cv::Mat temperature_image_u8(frame_height, frame_width, CV_8UC1);
    while (true) {

        // get frame
        if (thermal_sensor.hasNewFrame()) {
        
            thermal_sensor.nextFrame(temperature_u8, temperature_u16, T_s, T_o, true);
        
            // draw temperature
            memcpy(temperature_image_u8.data, temperature_u8, frame_height*frame_width);
            cv::flip(temperature_image_u8, temperature_image_u8, 0);
            if (click_x >= 0 && click_y >= 0) {
                object_temp = thermal_calib.RawToTemperature(temperature_u16[click_y*frame_width + click_x]);
                cv::circle(temperature_image_u8, cv::Point(click_x, click_y), 1, cv::Scalar(0));
                sprintf(label, "%.2f", object_temp);
                cv::putText(temperature_image_u8, label, cv::Point(click_x,click_y), 1, 1.0, cv::Scalar(0));
            }
        }

        // show image
        cv::imshow("Temperature", temperature_image_u8);
        if (cv::waitKey(10) == 27) {
            break;
        }
    }

    // Delete buffers
    delete[] temperature_u8;
    delete[] temperature_u16;
}

// App configuration
constexpr ThermalSensorType type = LEPTON_V3;
constexpr size_t width = 160;
constexpr size_t height = 120;
constexpr FRAME_TYPE frame_type = U16;

void main() {

    // Create sensor interface
    ThermalSensor thermal_sensor(width, height, type);
    bool hr_i = thermal_sensor.start();
    if (!hr_i) {
        std::cerr << "[Error][Grabber::init] Unable to conect to the FLIR device."
            << std::endl;
        exit(0);
    }
    else {
        char msg_u16[] = { FRAME_REQUEST , frame_type };
        thermal_sensor.sendMessage(msg_u16);
    }

    // Calibrate sensor
    ThermalCalibration thermal_calib(type);
    thermal_calib.Calibrate(thermal_sensor);

    // Test calibration
    TestCalibration(thermal_sensor, thermal_calib);

    // Close sensor interface
    thermal_sensor.stop();

}