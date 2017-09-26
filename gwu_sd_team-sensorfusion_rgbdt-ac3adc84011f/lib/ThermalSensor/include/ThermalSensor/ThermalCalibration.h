//============================================================================
// Name        : ThermalCalibration.h
// Author      : CosmaC
// Version     : V1.0
// Copyright   : GWU Research
// Description : Thermal calibration API
//============================================================================

#pragma once

#include "ThermalSensor/ThermalSensor.h"

struct ThermalModel {
    float fnc_slope;
    float fnc_offset;
    float fnc_correlation;
};

class ThermalCalibration
{
public:
    
    /*
     * @brief Constructor/destructor
     */
    ThermalCalibration(ThermalSensorType type) :
        nb_calib_frames(20),
        Tr_diff_threshold(0),
        To_diff_threshold(10),
        type_(type) {

        // Allocate buffers
        sensor_temp.resize(nb_calib_frames);
        object_temp.resize(nb_calib_frames);
        sensor_read.resize(nb_calib_frames);

        // Set default calibration
        if (type_ == LEPTON_V3) {
            // Thermal calibration default model
            fnc_offset = -197.486481f;
            fnc_slope = 0.027617f;
            // Calibration params
            kROI_x = 20;        // ROI top left OX
            kROI_width = 120;   // ROI width
            kROI_y = 50;        // ROI top left OY
            kROI_height = 20;   // ROI height
            kTr_std = 15;       // Accepted STD of the values inside the ROI
            kCorrelation_th = 0.5;// Minimum correlation to accept a new calibration
        }
        else if (type_ == LEPTON_V2) {
            // Thermal calibration default model
            fnc_offset = -241.52554f;
            fnc_slope = 0.032577534532f;
            // Calibration params
            kROI_x = 10;        // ROI top left OX
            kROI_width = 60;    // ROI width
            kROI_y = 25;        // ROI top left OY
            kROI_height = 10;   // ROI height
            kTr_std = 12;       // Accepted STD of the values inside the ROI
            kCorrelation_th = 0.5;// Minimum correlation to accept a new calibration
        }
        else {
            std::cerr << "[Error][ThermalCalibration::ThermalCalibration] Unknown sensor type."
                << std::endl;
        }

        // Common settings
        kWarmup_time = 1000; // Wait time to warmup the sensor
        kFFC_time = 2000;    // Wait time for FFC
    }
    ~ThermalCalibration() {}

    /*
     * @brief Change the number of minimum frames necessary to run the calibration
     */
    inline void set_nb_calib_frames(int nb_calib_frames_) {
        nb_calib_frames = nb_calib_frames_;
        sensor_temp.resize(nb_calib_frames);
        object_temp.resize(nb_calib_frames);
        sensor_read.resize(nb_calib_frames);
    }

    /*
     * @brief Run calibration
     */
    bool Calibrate(ThermalSensor& thermal_sensor);
    
    /*
     * @brief Load calibration
     */
    bool LoadCalibration(ThermalSensorType sensor_type);

    /*
     * @brief Convert raw data to temperature
     */
    inline float RawToTemperature(unsigned short raw_value) { 
        return (fnc_slope * raw_value) + fnc_offset; }

    // Get/Set
    inline ThermalModel GetThermalCalibration() {
        ThermalModel model;
        model.fnc_correlation = fnc_correlation;
        model.fnc_offset = fnc_offset;
        model.fnc_slope = fnc_slope;
        return model;
    }
    inline void SetThermalCalibration(const ThermalModel& calib) {
        fnc_correlation = calib.fnc_correlation;
        fnc_slope = calib.fnc_slope;
        fnc_offset = calib.fnc_offset;
    }

private:
    /*
     * @brief  Get the average IR raw value for the pre-setted ROI.
     *         If the STD is to high, the average raw value is set to invalid.
     */
    void GetAvgTemperatureReading(const unsigned short* temperature_u16, size_t width, float& avg_T_r);
    
    /*
     * @brief  Run the FFC for the give sensor
     */
    void ThermalFFC(ThermalSensor& thermal_sensor);
    
    // Sensor type to be calibrated
    ThermalSensorType type_;

    // Calibration configuration (see class constructor)
    int nb_calib_frames;    // min number of measurements
    int Tr_diff_threshold;  // min diff between last stored Tr measure and next one
    int To_diff_threshold;  // min diff between last stored To measure and next one
    size_t kROI_x;          // ROI top left OX
    size_t kROI_width;      // ROI width
    size_t kROI_y;          // ROI top left OY
    size_t kROI_height;     // ROI height
    size_t kTr_std;         // Accepted STD of the values inside the ROI
    float  kCorrelation_th; // Minimum correlation to accept a new calibration
    size_t kWarmup_time;    // Wait time to warmup the sensor
    size_t kFFC_time;       // Wait time for FFC

    // Calibration data
    float fnc_slope;
    float fnc_offset;
    float fnc_correlation;
    std::vector<int> sensor_temp;
    std::vector<float> object_temp;
    std::vector<float> sensor_read;

    // Calibration constants
    const float  kWrong_Tr       = -101.0;  // invalid Tr value
    const float  kWrong_To       = -101.0;  // invalid To value

};
