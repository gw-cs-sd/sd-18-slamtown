//============================================================================
// Name        : ThermalSensorAnalysis.cpp
// Author      : CosmaC
// Date        : March, 2017
// Copyright   : GWU Research
// Description : Thermal camera output analysis
//============================================================================

#include "ThermalSensor/ThermalSensor.h"
#include "ThermalSensor/ThermalCalibration.h"
#include "ImageProcessing/ImageUtils.h"

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
constexpr ThermalSensorType type = LEPTON_V2;
constexpr size_t width = 80;
constexpr size_t height = 60;
constexpr size_t kROI_x = 10;       // 10|20
constexpr size_t kROI_y = 25;       // 25|50
constexpr size_t kROI_width = 60;   // 60|120
constexpr size_t kROI_height = 10;  // 10|20
constexpr size_t kTr_std = 12;      // 12|15
constexpr FRAME_TYPE frame_type = U16;

// Define analysis board
constexpr float ts_min = 10;
constexpr float ts_max = 40;
constexpr float to_min = 10;
constexpr float to_max = 40;
constexpr unsigned short tr_min = 7700;
constexpr unsigned short tr_max = 8700;

constexpr size_t a_width = 1280;
constexpr size_t a_height = 640;
constexpr size_t step_grid = 10;
constexpr size_t num_steps = a_height / step_grid;
cv::Mat analyzer(a_height+2, a_width+2, CV_8UC3);
std::vector<float> to(a_width);
std::vector<float> ts(a_width);
std::vector<unsigned short> tr(a_width);
size_t current_idx = 0;

template <typename T>
void DrawData(const std::vector<T>& data, T min, T max, const cv::Vec3b& color, cv::Mat& image) {

    size_t flip_offset = image.rows - 1;
    float scale_factor = (1.f / (max - min)) * a_height;
    for (size_t x = 0; x < data.size(); ++x) {

        // Compute true index
        size_t x_offsetted = (x + current_idx) % a_width;
        if (data[x_offsetted] > max || data[x_offsetted] < min) {
            continue;
        }

        // Compute Y value
        size_t y = static_cast<size_t>((data[x_offsetted] - min) * scale_factor + 0.5);

        // Mark point
        image.at<cv::Vec3b>(flip_offset - y, x) = color;
        image.at<cv::Vec3b>(flip_offset - y, x+1) = color;
        image.at<cv::Vec3b>(flip_offset - (y+1), x) = color;
        image.at<cv::Vec3b>(flip_offset - (y+1), x+1) = color;
    }
}

void DrawGrid(cv::Mat& image, size_t step) {

    const size_t min_x = 0;
    const size_t max_x = image.cols - 1;
    for (int i = image.rows - step; i > 0; i -= step) {
        unsigned char scale = (i/step) % 2;
        unsigned char gray = (scale+1) * 30;
        cv::line(image, cv::Point(min_x, i), cv::Point(max_x, i), cv::Scalar(gray, gray, gray));
    }
}

void DisplayAnalysis() {

    // Reset image
    analyzer = 0;
    // Draw grid
    DrawGrid(analyzer, step_grid);
    // Add points
    DrawData(to, to_min, to_max, cv::Vec3b(0, 0, 255), analyzer);
    DrawData(ts, ts_min, ts_max, cv::Vec3b(255, 255, 255), analyzer);
    DrawData(tr, tr_min, tr_max, cv::Vec3b(0, 255, 0), analyzer);
    // Add current measurements
    char label_to[100];
    char label_ts[100];
    char label_tr[100];
    if (current_idx == 0) {
        sprintf(label_to, "To(%.3f,%.3f,%.3f) = %.3f",
            to_min, to_max, (to_max-to_min) / num_steps, to[to.size() - 1]);
        sprintf(label_ts, "Ts(%.3f,%.3f,%.3f) = %.3f", 
            ts_min, ts_max, (ts_max - ts_min) / num_steps, ts[ts.size() - 1]);
        sprintf(label_tr, "Tr(%d,%d,%lld) = %d", 
            tr_min, tr_max, (tr_max - tr_min) / num_steps, tr[tr.size() - 1]);
    }
    else {
        sprintf(label_to, "To(%.3f,%.3f,%.3f) = %.3f", 
            to_min, to_max, (to_max - to_min) / num_steps, to[current_idx - 1]);
        sprintf(label_ts, "Ts(%.3f,%.3f,%.3f) = %.3f", 
            ts_min, ts_max, (ts_max - ts_min) / num_steps, ts[current_idx - 1]);
        sprintf(label_tr, "Tr(%d,%d,%lld) = %d", 
            tr_min, tr_max, (tr_max - tr_min) / num_steps, tr[current_idx - 1]);
    }
    cv::putText(analyzer, label_to, cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 0, 255));
    cv::putText(analyzer, label_ts, cv::Point(10, 40), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(255, 255, 255));
    cv::putText(analyzer, label_tr, cv::Point(10, 60), cv::FONT_HERSHEY_PLAIN, 1.5, cv::Scalar(0, 255, 0));
}

void GetAvgTemperatureReading(const unsigned short* temperature_u16, size_t width, unsigned short& avg_T_r) {

    unsigned int sum = 0;
    for (size_t y = kROI_y; y < kROI_y + kROI_height; ++y) {
        size_t offset = y * width;
        for (size_t x = kROI_x; x < kROI_x + kROI_width; ++x) {
            sum += temperature_u16[offset + x];
        }
    }
    avg_T_r = sum / (kROI_width * kROI_height);

    float std_dev = 0;
    for (size_t y = kROI_y; y < kROI_y + kROI_height; ++y) {
        size_t offset = y * width;
        for (size_t x = kROI_x; x < kROI_x + kROI_width; ++x) {
            std_dev += (avg_T_r - temperature_u16[offset + x]) *
                (avg_T_r - temperature_u16[offset + x]);
        }
    }
    std_dev /= (kROI_width * kROI_height);
    //printf("AVG %f  STD %f \n", avg_T_r, std_dev);
    if (std_dev > kTr_std * kTr_std) {
        avg_T_r = 0;
    }

}

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

    // Loop
    ThermalCalibration thermal_calib(type);
    unsigned int old_id = 9999999;
    unsigned int new_id = 0;
    clock_t begin = clock();
    clock_t clock_0 = clock();
    int nb_frames = 0;
    int nb_all_frames = 0;
    int sensor_tempeature = 0;
    float obj_temperature = 0;
    std::vector<unsigned short> frame_u16(width*height);
    while (1) {

        // Grab
        if (thermal_sensor.hasNewFrame()) {
            new_id = thermal_sensor.nextFrame(frame_u16.data(), sensor_tempeature, obj_temperature, true);
            clock_t clock_now = clock();
        }
        ++nb_all_frames;

        // Check for new frame
        if (old_id != new_id) {

            // Count the new frame
            old_id = new_id;
            ++nb_frames;

            // Update analysis
            to[current_idx] = obj_temperature;
            ts[current_idx] = (sensor_tempeature / 100.f - 273.15);
            GetAvgTemperatureReading(frame_u16.data(), width, tr[current_idx]);
            ++current_idx;
            if (current_idx >= a_width) {
                current_idx = 0;
            }
            DisplayAnalysis();
        }

        // View frame & Control
        cv::Mat thermal_img;
        CreateImageU8ByMinMaxNormalization(frame_u16, width, height, thermal_img,
            static_cast<unsigned short>(8000), static_cast<unsigned short>(8500));
        cv::flip(thermal_img, thermal_img, 0);
        cv::resize(thermal_img, thermal_img, cv::Size(0,0), 2, 2);
        cv::imshow("Thermal Image", thermal_img);
        cv::imshow("Analyzer", analyzer);
        char key = cv::waitKey(10);
        if (key == 27) { // esc
            cv::destroyAllWindows();
            break;
        }
        // Do thermal calibration
        else if (key == 'c') {
            thermal_calib.Calibrate(thermal_sensor);
        }
        // Test calibration
        else if (key == 't') {
            TestCalibration(thermal_sensor, thermal_calib);
        }
        // Do FFC
        else if (key == 'f') {
            printf("[INFO] Run FFC... ");
            char msg_ffc[] = { I2C_CMD , FFC };
            thermal_sensor.sendMessage(msg_ffc);
            char msg_frame[] = { FRAME_REQUEST , U16 };
            thermal_sensor.sendMessage(msg_frame);
            Sleep(1000);
            printf(" Done! \n");
        }
        // Help: print options
        else if (key == 'h') {
            std::cout << "\n\n Options: " << std::endl;
            std::cout << "\t- c: do thermal calibration" << std::endl;
            std::cout << "\t- t: test calibration" << std::endl;
            std::cout << "\t- f: run FFC on demand" << std::endl;
            std::cout << "\t- h: help" << std::endl;
            std::cout << "\t- esc: exit dataset collection" << std::endl;
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

    // Close sensor interface
    thermal_sensor.stop();
}