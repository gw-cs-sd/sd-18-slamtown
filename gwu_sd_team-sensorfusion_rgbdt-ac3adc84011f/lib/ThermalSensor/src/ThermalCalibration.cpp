//============================================================================
// Name        : ThermalCalibration.cpp
// Author      : CosmaC
// Version     : V1.0
// Copyright   : GWU Research
// Description : Thermal calibration implementation
//============================================================================


// Module interface
#include "ThermalSensor/ThermalCalibration.h"

// OpenCV
#include <opencv/cv.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>


// Compute power of 2
inline static float sqr(float x) {
    return x*x;
}


// Least square fit for a linear equation f(x) = a*x + b
bool linreg(int n, const float* x, const float* y, float* m, float* b, float* r) {

    double   sumx = 0.0;
    double   sumx2 = 0.0;
    double   sumxy = 0.0;
    double   sumy = 0.0;
    double   sumy2 = 0.0;
    for (int i = 0; i < n; i++)
    {
        sumx += x[i];
        sumx2 += sqr(x[i]);
        sumxy += x[i] * y[i];
        sumy += y[i];
        sumy2 += sqr(y[i]);
    }
    double denom = (n * sumx2 - sqr(sumx));
    if (denom == 0) {
        //singular matrix. can't solve the problem.
        *m = 0;
        *b = 0;
        *r = 0;
        return false;
    }
    *m = (n * sumxy - sumx * sumy) / denom;
    *b = (sumy * sumx2 - sumx * sumxy) / denom;
    if (r != nullptr) {
        *r = (sumxy - sumx * sumy / n) /
            sqrt((sumx2 - sqr(sumx) / n) *
                (sumy2 - sqr(sumy) / n));
    }
    return true;
}


// Run FFC before calibration
void ThermalCalibration::ThermalFFC(ThermalSensor& thermal_sensor) {

    // Run FFC
    printf("[INFO] Run FFC... ");
    char msg_ffc[] = { I2C_CMD , FFC};
    thermal_sensor.sendMessage(msg_ffc);
    char msg_frame[] = { FRAME_REQUEST , U16 };
    thermal_sensor.sendMessage(msg_frame);

    // Wait for FFC to finish, 2 seconds should be enough
    Sleep(kFFC_time);
    printf(" Done! \n");
}


// Get avg_Tr using a ROI around center of the image
void ThermalCalibration::GetAvgTemperatureReading(const unsigned short* temperature_u16, size_t width, float& avg_T_r) {

    unsigned int sum = 0;
    for (size_t y = kROI_y; y < kROI_y + kROI_height; ++y) {
        size_t offset = y * width;
        for (size_t x = kROI_x; x < kROI_x + kROI_width; ++x) {
            sum += temperature_u16[offset + x];
        }
    }
    avg_T_r = static_cast<float>(sum) / (kROI_width * kROI_height);

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
        avg_T_r = kWrong_Tr;
    }

}


// Run thermal calibration
bool ThermalCalibration::Calibrate(ThermalSensor& thermal_sensor) {

    // Todo: warmup wait
    printf("[INFO] Warmup... ");
    Sleep(kWarmup_time);
    printf("Done! \n");

    // FFC
    ThermalFFC(thermal_sensor);

    // Get readings
    const size_t frame_width = thermal_sensor.getFrameWidth();
    const size_t frame_height = thermal_sensor.getFrameHeight();
    const size_t frame_szie = frame_height * frame_width;
    unsigned short* temperature_u16 = new unsigned short[frame_szie]; // Raw temperature buffer
    unsigned char* temperature_u8 = new unsigned char[frame_szie];      // Post-processed temperature buffer

    int nb_frames = 0;
    int T_s;
    float T_o, old_T_o = 0;
    float avg_T_r, old_avg_T_r = 0;

    while(true) {

        printf("[INFO] Get readings (Press S if you want to skip calibration)... \n");
        while (nb_frames < nb_calib_frames) {

            if (thermal_sensor.hasNewFrame()) {

                thermal_sensor.nextFrame(temperature_u8, temperature_u16, T_s, T_o, true);
                GetAvgTemperatureReading(temperature_u16, frame_width, avg_T_r);
                //printf("T_r %f  T_o %f \n", avg_T_r, T_o);
                if (std::abs(avg_T_r - old_avg_T_r) > Tr_diff_threshold && avg_T_r > kWrong_Tr &&
                    std::abs(T_o - old_T_o) < To_diff_threshold && T_o > kWrong_To) {

                    old_avg_T_r = avg_T_r;
                    sensor_read[nb_frames] = avg_T_r;
                    object_temp[nb_frames] = T_o;
                    sensor_temp[nb_frames] = T_s;
                    nb_frames++;
                    printf("\tNew calib frame %d out of %d. \n", nb_frames, nb_calib_frames);
                }
                old_T_o = T_o;

                cv::Mat temperature_image_u8(frame_height, frame_width, CV_8UC1, temperature_u8);
                cv::flip(temperature_image_u8, temperature_image_u8, 0);
                cv::rectangle(temperature_image_u8, cv::Rect(kROI_x, kROI_y, kROI_width, kROI_height), cv::Scalar(0));
                cv::imshow("Temperature image", temperature_image_u8);
                char key = cv::waitKey(10);
                if (key == 27) {
                    break;
                }
                else if (key == 's' || key == 'S') {
                    // skip calibration
                    printf("\tSkip! \n");
                    return false;
                }
            }
        }
        printf("\tDone! \n");

        // Fit function
        printf("[INFO] Fit function to collected data... ");
        linreg(nb_calib_frames, sensor_read.data(), object_temp.data(),
            &fnc_slope, &fnc_offset, &fnc_correlation);
        printf("Done! \n");

        if (fnc_correlation < kCorrelation_th) {
            std::cout << "\tWeak correlation " << fnc_correlation
                      << "... Restarting calibration! " << std::endl;
        }
        else {
            std::cout << "\tGood correlation " << fnc_correlation
                << "... Successful calibration! " << std::endl;
            break;
        }
    }
    printf("\tRaw values to temperature: %f * X + %f = T \n", fnc_slope, fnc_offset);

    // Stop and clean
    delete[] temperature_u8;
    delete[] temperature_u16;

    // save model
    printf("[INFO] Save calibration to file... ");
    FILE* f_m = fopen("thermal_model.csv", "a");
    if (f_m) {
        fprintf(f_m, "%f  %f  %f \n", fnc_correlation, fnc_slope, fnc_offset);
        fclose(f_m);
    }
    else {
        printf("\n\tError opening file thermal_model.csv to write model.\n");
    }

    FILE* f_l = fopen("thermal_data.csv", "w");
    if (f_l) {
        for (size_t i = 0; i < nb_calib_frames; ++i) {
            fprintf(f_l, "%d \t %f \t %f \n", sensor_temp[i], sensor_read[i], object_temp[i]);
        }
        fclose(f_l);
    }
    else {
        printf("\n\tError opening file calib.csv to write calibration points.\n");
    }
    printf("Done! \n");

    return true;
}


// Load calibration
bool ThermalCalibration::LoadCalibration(ThermalSensorType sensor_type) {
    
    // load calib model
    printf("[INFO] Load thermal calibration from file... ");

    std::string file_name;
    if (sensor_type == LEPTON_V2) {
        file_name = "l2_thermal_model.bin";
    }
    else { // Lepton V3
        file_name = "l3_thermal_model.bin";
    }
    FILE* f_m = fopen(file_name.c_str(), "rb");
    if (!f_m) {
        printf("\n\tError opening file thermal_model.csv to read calibration model.\n");
        return false;
    }

    fread(&fnc_slope, sizeof(fnc_slope), 1, f_m);
    fread(&fnc_offset, sizeof(fnc_offset), 1, f_m);
    fread(&fnc_correlation, sizeof(fnc_correlation), 1, f_m);
    fclose(f_m);
    printf("\n\tCorrelation: %f \n\tSlope: %f \n\tOffset: %f \n", fnc_correlation, fnc_slope, fnc_offset);

    return true;
}
