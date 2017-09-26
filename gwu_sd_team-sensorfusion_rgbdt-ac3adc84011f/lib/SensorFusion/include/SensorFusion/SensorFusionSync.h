//============================================================================
// Name        : SensorFusionSync.h
// Author      : CosmaC
// Date        : October, 2016
// Copyright   : GWU Research
// Description : Sensor fusion stream syncronizer
//============================================================================

#pragma once

#include "SensorFusion/SensorFusionRegistration.h"
#include "3DSensor/3DSensor.h"
#include "ThermalSensor/ThermalSensor.h"
#include "ThermalSensor/ThermalCalibration.h"

// C/C++
#include <iostream>
#include <cstdlib>
#include <fstream>
#include <climits>

#define INVALID_FRAME_BUFFERS ULLONG_MAX
#define INVALID_FRAME_MAPPER (ULLONG_MAX-1)

constexpr int FLAG_RGB = 1;
constexpr int FLAG_D = 2;
constexpr int FLAG_T = 4;
constexpr int FLAG_S = 8;
constexpr int FLAG_RGBDT = 16;
constexpr int FLAG_MASK = 32;
constexpr int FLAG_XYZ = 64;
constexpr int STATE_FULL_FRAME = 127;
constexpr int STATE_FULL_RAW_FRAME = 15;
constexpr int STATE_FULL_REG_FRAME = 112;

enum FrameType {
    RAW_FRAME_RGB,          // raw color image from sensors
    RAW_FRAME_D,            // raw depth image from sensors
    RAW_FRAME_T,            // raw thermal image from sensors
    RAW_SKELETON,           // detected bodies in the depth frame
    RAW_FRAME_ALL,          // raw frame with all channels (color, depth, temperature and skeleton)
    REGISTERED_FRAME_RGBDT, // geometrically calibrated image
    REGISTERED_FRAME_MASK,  // geometrically calibrated image mask
    REGISTERED_FRAME_XYZ,   // geometrically calibrated image XYZ
    METADATA_FRAME,     // no pixel data, just frame description
    TIMESTAMP,          // frame timestamp
    G_CALIB_FRAME,      // geometric calibration metadata
    T_CALIB_FRAME,      // thermal calibration metadata
    TM_FRAME,           // thermal model frame
    END_FRAME           // signal end of processing
};

struct RawFrame {
    std::vector<unsigned short> frame_depth;        // depth image; values are in mm
    std::vector<unsigned char> frame_color;         // color image; with RGBA(4 bytes) per pixel
    std::vector<unsigned short> frame_temperature;  // temperatures image; raw readings from sensor
    std::vector<BodySkeleton> skeletons;   // bodies in the latest complete read frame
    // unsigned long long timestamp_depth;
    // unsigned long long timestamp_color;
    // unsigned long long timestamp_temperature;
    size_t width_d;
    size_t height_d;
    size_t width_c;
    size_t height_c;
    size_t width_t;
    size_t height_t;
};


struct Pixel_RGBDT {
    float temperature;  // temperature in Celsius degrees
    short depth;        // depth in mm
    unsigned char r;    // Red
    unsigned char g;    // Green
    unsigned char b;    // Blue
};

struct Pixel_XYZ {
    float x;
    float y;
    float z;
};


struct Frame {
    std::vector<Pixel_RGBDT> data;  // depth image; values are in mm
    std::vector<Pixel_XYZ> data_3D; // point in the world coordinate frame
    std::vector<bool> mask;         // mask of valid pixels
    size_t width;
    size_t height;
};

class SensorFusionSync {

public:
    // Class constructor
    SensorFusionSync(ThermalSensorType thermal_type) :
        work_offline_(false),
        frame_available_(false),
        do_grab_(false),
        pause_sync_grab_(false),
        frame_id_(0),
        type_(thermal_type){

        // Set thermal sensor resolution
        if (LEPTON_V3 == type_) {
            kWidth_T = LEPTON_V3_WIDTH;
            kHeight_T = LEPTON_V3_HEIGHT;
        }
        else if (LEPTON_V2 == type_) {
            kWidth_T = LEPTON_V2_WIDTH;
            kHeight_T = LEPTON_V2_HEIGHT;
        }
        else {
            std::cerr << "[Error][ThermalSensor::ThermalSensor] Unknown sensor type."
                << std::endl;
        }

        // Create thermal sensor drivers
        thermal_sensor_ = std::unique_ptr<ThermalSensor>(new ThermalSensor(kWidth_T, kHeight_T, thermal_type));
        thermal_calib_ = std::unique_ptr<ThermalCalibration>(new ThermalCalibration(thermal_type));

        // Create depth sensor driver
        depth_sensor_ = std::unique_ptr<Sensor3D>(new Sensor3D(kWidth_D, kHeight_D, kWidth_C, kHeight_C, kColor_bpp));
        geo_calib_ = std::unique_ptr<GeometryCalibration>(new GeometryCalibration);

        // Allocate frame buffers
        frame_depth_size_ = kWidth_D * kHeight_D;
        frame_color_size_ = kWidth_C * kHeight_C * kColor_bpp;
        frame_temperature_size_ = kWidth_T * kHeight_T;

        frame_to_read_ = new RawFrame;
        frame_to_read_->frame_color.resize(frame_color_size_);
        frame_to_read_->frame_depth.resize(frame_depth_size_);
        frame_to_read_->frame_temperature.resize(frame_temperature_size_);
        frame_to_read_->skeletons.resize(MAX_BODIES);
        
        frame_to_write_ = new RawFrame;
        frame_to_write_->frame_color.resize(frame_color_size_);
        frame_to_write_->frame_depth.resize(frame_depth_size_);
        frame_to_write_->frame_temperature.resize(frame_temperature_size_);
        frame_to_write_->skeletons.resize(MAX_BODIES);

        // Prepare depth image indexes for registration
        size_t idx = 0;
        depth_idx.resize(frame_depth_size_);
        for (size_t l = 0; l < kHeight_D; ++l) {
            for (size_t c = 0; c < kWidth_D; ++c) {
                depth_idx[idx].X = static_cast<float>(c);
                depth_idx[idx].Y = static_cast<float>(l);
                ++idx;
            }
        }
    }

    SensorFusionSync(const std::string& dataset_path, int start_id) :
        work_offline_(true),
        dataset_path_(dataset_path),
        frame_available_(false),
        do_grab_(false),
        pause_sync_grab_(false),
        frame_id_(start_id) {

        // Load Geometric calibration
        GeometryModel geo_calib;
        char geometry_calib_file[255];
        sprintf(geometry_calib_file, "%s/geometry_model.bin", dataset_path.c_str());
        std::ifstream input_file(geometry_calib_file, std::ios::binary);
        if (!input_file.is_open()) {
            std::cout << "[ERROR][SensorFusionSync::constructor] Unable to open"
                " dataset geometry calibration file." << std::endl;
            // throw error
            throw std::runtime_error("System failed to open calibration file.");
        }
        else {
            input_file.read((char*)&geo_calib, sizeof(geo_calib));
            input_file.close();

            printf("\n\tDelta disp: %d \n\tInverse coefficient %f \n\tHomography:\n"
                "\t %f  %f  %f\n\t %f  %f  %f\n\t %f  %f  %f\n", 
                geo_calib.deltaDisp, geo_calib.invCoeff,
                geo_calib.h[0], geo_calib.h[1], geo_calib.h[2], 
                geo_calib.h[3], geo_calib.h[4], geo_calib.h[5],
                geo_calib.h[6], geo_calib.h[7], geo_calib.h[8]);
            printf("\t Color image resolution: %d x %d\n", geo_calib.width_c, geo_calib.height_c);
            printf("\t Depth image resolution: %d x %d\n", geo_calib.width_d, geo_calib.height_d);
            printf("\t Thermal image resolution: %d x %d\n", geo_calib.width_t, geo_calib.height_t);
            printf("\t Thermal image scale: %d\n", geo_calib.scale_t);

            kWidth_C = geo_calib.width_c;
            kHeight_C = geo_calib.height_c;
            kWidth_D = geo_calib.width_d;
            kHeight_D = geo_calib.height_d;
            kWidth_T = geo_calib.width_t;
            kHeight_T = geo_calib.height_t;

            if (kWidth_T == 80 && kHeight_T == 60) {
                type_ = LEPTON_V2;
            }
            else if (kWidth_T == 160 && kHeight_T == 120) {
                type_ = LEPTON_V3;
            }
            else {
                std::cout << "[ERROR][SensorFusionSync::constructor] Unknown "
                    "thermal sensor resolution." << std::endl;
                // throw error
            }
        }

        // Load thermal calibration
        ThermalModel thermal_calib;
        char thermal_calib_file[255];
        sprintf(thermal_calib_file, "%s/thermal_model.bin", dataset_path.c_str());
        input_file = std::ifstream(thermal_calib_file, std::ios::binary);
        if (!input_file.is_open()) {
            std::cout << "[ERROR][SensorFusionSync::constructor] Unable to open"
                " dataset thermal calibration file." << std::endl;
            // throw error
        }
        else {
            input_file.read((char*)&thermal_calib, sizeof(thermal_calib));
            input_file.close();

            printf("\n\tCorrelation: %f \n\tSlope: %f \n\tOffset: %f \n", 
                thermal_calib.fnc_correlation, thermal_calib.fnc_slope, thermal_calib.fnc_offset);
        }

        // Create thermal sensor drivers
        thermal_sensor_ = std::unique_ptr<ThermalSensor>(new ThermalSensor(kWidth_T, kHeight_T, type_));
        thermal_calib_ = std::unique_ptr<ThermalCalibration>(new ThermalCalibration(type_));
        thermal_calib_->SetThermalCalibration(thermal_calib);

        // Create depth sensor driver
        depth_sensor_ = std::unique_ptr<Sensor3D>(new Sensor3D(kWidth_D, kHeight_D, kWidth_C, kHeight_C, kColor_bpp));
        geo_calib_ = std::unique_ptr<GeometryCalibration>(new GeometryCalibration);
        geo_calib_->setCalibration(geo_calib);

        // Allocate frame buffers
        frame_depth_size_ = kWidth_D * kHeight_D;
        frame_color_size_ = kWidth_C * kHeight_C * kColor_bpp;
        frame_temperature_size_ = kWidth_T * kHeight_T;

        frame_to_read_ = new RawFrame;
        frame_to_read_->frame_color.resize(frame_color_size_);
        frame_to_read_->frame_depth.resize(frame_depth_size_);
        frame_to_read_->frame_temperature.resize(frame_temperature_size_);
        frame_to_read_->skeletons.resize(MAX_BODIES);

        frame_to_write_ = new RawFrame;
        frame_to_write_->frame_color.resize(frame_color_size_);
        frame_to_write_->frame_depth.resize(frame_depth_size_);
        frame_to_write_->frame_temperature.resize(frame_temperature_size_);
        frame_to_write_->skeletons.resize(MAX_BODIES);

        // Prepare depth image indexes for registration
        size_t idx = 0;
        depth_idx.resize(frame_depth_size_);
        for (size_t l = 0; l < kHeight_D; ++l) {
            for (size_t c = 0; c < kWidth_D; ++c) {
                depth_idx[idx].X = static_cast<float>(c);
                depth_idx[idx].Y = static_cast<float>(l);
                ++idx;
            }
        }
    }
    
    // Class destructor
    ~SensorFusionSync() {

        // Delete buffers
        frame_to_write_->frame_color.clear();
        frame_to_write_->frame_depth.clear();
        frame_to_write_->frame_temperature.clear();
        frame_to_write_->skeletons.clear();
        delete frame_to_write_;
        frame_to_read_->frame_color.clear();
        frame_to_read_->frame_depth.clear();
        frame_to_read_->frame_temperature.clear();
        frame_to_read_->skeletons.clear();
        delete frame_to_read_;
    }

    // Starts frame grabber
    bool start(bool do_FFC = false, bool force_start_depth_sensor = false);

    // Stops frame grabber
    bool stop();

    // Run calibration: thermal calibration and/or sensors geometry calibration
    bool Calibrate(bool do_geometry, bool do_thermal);

    // Load calibration: thermal calibration and/or sensors geometry calibration
    bool LoadCalibration(bool do_geometry, bool do_thermal, ThermalSensorType sensor_type);

    // Gets next frame available
    unsigned long long NextFrame(RawFrame& frame, bool force = false);
    
    // Gets next registred frame available
    unsigned long long NextFrame(RawFrame& raw_frame, Frame& frame, 
                                 int& status, bool force = false);

    // Checks if new frame avaliable
    inline bool HasNewFrame() {     
        // Check if offline mode
        if (work_offline_) {
            return true;
        }
        else {
            return frame_available_;
        }
    };

    // Allocate and initialize raw frame container
    // Use this to avoid manually creation and initialization of frame buffer
    inline void AllocateAndInitializeRawFrame(RawFrame& frame) {
        frame.width_d = kWidth_D;
        frame.height_d = kHeight_D;
        frame.width_c = kWidth_C;
        frame.height_c = kHeight_C;
        frame.width_t = kWidth_T;
        frame.height_t = kHeight_T;
        frame.frame_depth.resize(frame_depth_size_);
        frame.frame_color.resize(frame_color_size_);
        frame.frame_temperature.resize(frame_temperature_size_);
        frame.skeletons.resize(MAX_BODIES);
    }

    // Allocate and initialize registered frame container
    // Use this to avoid manually creation and initialization of frame buffer
    inline void AllocateAndInitializeRegFrame(Frame& frame) {
        frame.data.resize(kWidth_D * kHeight_D);
        frame.mask.resize(kWidth_D * kHeight_D);
        frame.data_3D.resize(kWidth_D * kHeight_D);
        frame.width = kWidth_D;
        frame.height = kHeight_D;
    }

    // Getters
    size_t getDepthWidth() { return kWidth_D; };
    size_t getDepthHeight() { return kHeight_D; };
    size_t getDepthSize() { return frame_depth_size_; };

    size_t getColorWidth() { return kWidth_C; };
    size_t getColorHeight() { return kHeight_C; };
    size_t getColorSize() { return frame_color_size_; };
    size_t getColorBPP() { return kColor_bpp; };

    size_t getTemperatureWidth() { return kWidth_T; };
    size_t getTemperatureHeight() { return kHeight_T; };
    size_t getTemperatureSize() { return frame_temperature_size_; };

    int getSensorTemperature() { return sensor_temp_; };
    float getObjectTemperature() { return object_temp_; };

    ThermalModel getThermalCalibration() {
        return thermal_calib_->GetThermalCalibration();
    }

    GeometryModel getGeometryCalibration() {
        GeometryModel calib;
        geo_calib_->getCalibration(calib);
        return calib;
    }

    // Run FFC on demand
    inline void doFFC() {
        printf("[INFO] Run FFC... ");
        char msg_ffc[] = { I2C_CMD , FFC };
        thermal_sensor_->sendMessage(msg_ffc);
        char msg_frame[] = { FRAME_REQUEST , U16 };
        thermal_sensor_->sendMessage(msg_frame);
        Sleep(1000);
        printf(" Done! \n");
    }

private:
    // Grab and sync frames
    void run();
    
    // Gets next frame available
    unsigned long long NextOnlineFrame(RawFrame& frame, bool force = false);
    unsigned long long NextOfflineFrame(RawFrame& frame, bool force = false);
    // Gets next registred frame available
    unsigned long long NextOnlineFrame(RawFrame& raw_frame, Frame& frame, bool force = false);
    unsigned long long NextOfflineFrame(RawFrame& raw_frame, Frame& frame, bool force = false);
    // Compute registred frame from raw frame
    bool computeRegFrame(RawFrame& raw_frame, Frame& frame);

    // Thermal calibration
    bool DoThermalCalibration();
    bool DoGeometryCalibration();
    ThermalSensorType type_;

    // Device grabbers
    std::unique_ptr<ThermalSensor> thermal_sensor_;     // thermal imager device handle
    std::unique_ptr<Sensor3D> depth_sensor_;            // depth sensing device handle
    std::unique_ptr<ThermalCalibration> thermal_calib_; // Thermal calibration module
    std::unique_ptr<GeometryCalibration> geo_calib_;    // Geometry calibration module

    // Frame buffers
    RawFrame* frame_to_read_;   // latest complete read frame
    RawFrame* frame_to_write_;  // current processing frame from devices
    size_t frame_depth_size_;
    size_t frame_color_size_;
    size_t frame_temperature_size_;
    int sensor_temp_   = -1;
    float object_temp_ = -1.0;

    // Control
    bool work_offline_;             // offline mode trigger
    std::string dataset_path_;      // dataset path for offline mode
    bool frame_available_;          // signals availability of a new frame
    unsigned long long frame_id_;   // latest complete grabbed frame id
    bool do_grab_;                  // trigger grabbing thread
    bool pause_sync_grab_;          // temporary pause grabbing
    boost::thread img_thread_;      // grabbing thread handle
    boost::mutex mtx_;              // lock for shared data

    // Other util variables, lookup tables, or pre-computed indexes
    std::vector<DepthSpacePoint> depth_idx;
    std::vector<ColorSpacePoint> color_idx;
    std::vector<CameraSpacePoint> data_3D;

    // Sensors default resolution
    size_t kWidth_D = 512;
    size_t kHeight_D = 424;
    size_t kWidth_C = 1920;
    size_t kHeight_C = 1080;
    size_t kColor_bpp = 3;
    size_t kWidth_T;                // This will be set by the constructor based on the sensor type
    size_t kHeight_T;               // This will be set by the constructor based on the sensor type

    // Track internal state for offline mode
    int state_;
};