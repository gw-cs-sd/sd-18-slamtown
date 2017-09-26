//============================================================================
// Name        : SensorFusionSync.cpp
// Author      : CosmaC
// Date        : October, 2016
// Copyright   : GWU Research
// Description : Sensor fusion stream syncronizer
//============================================================================

#include "SensorFusion/SensorFusionSync.h"
#include "SensorFusion/SensorFusionRegistration.h"
#include "ThermalSensor/ThermalSensor.h"
#include "ThermalSensor/ThermalCalibration.h"

///////////////////////////////////////////////////////////////////////////////
//  Sensor Fusion Grabber: Syncronized grabbing
///////////////////////////////////////////////////////////////////////////////

// Starts frame grabber
bool SensorFusionSync::start(bool do_FFC, bool force_start_depth_sensor) {
    
    // Check if offline mode
    if (work_offline_) {
        if (force_start_depth_sensor) {
            bool hr = depth_sensor_->start();
            if (!hr) {
                std::cerr << "[Error][SensorFusionSync] Unable to connect to the 3D sensing device."
                    << std::endl;
                return false;
            }
        }
        return true;
    }

    // Check if not running
    if (!do_grab_) {

        // config depth sensor
        bool hr = depth_sensor_->start();
        if (!hr) {
            std::cerr << "[Error][SensorFusionSync] Unable to connect to the 3D sensing device."
                << std::endl;
            return false;
        }
        // config thermal sensor
        hr = thermal_sensor_->start();
        if (!hr) {
            std::cerr << "[Error][SensorFusionSync] Unable to conect to the thermal imager device."
                << std::endl;
            return false;
        }
        char msg_u16[] = { FRAME_REQUEST, U16 };
        thermal_sensor_->sendMessage(msg_u16);

        // Run FFC
        if (do_FFC) {
            printf("[INFO] Run FFC... ");
            char msg_ffc[] = { I2C_CMD , FFC };
            thermal_sensor_->sendMessage(msg_ffc);
            char msg_frame[] = { FRAME_REQUEST , U16 };
            thermal_sensor_->sendMessage(msg_frame);
            Sleep(1000);
            printf(" Done! \n");
        }

        // Create grabbing thread
        do_grab_ = true;
        img_thread_ = boost::thread(&SensorFusionSync::run, this);
    }

    // All sensors up and running
    return true;
}

// Stops frame grabber
bool SensorFusionSync::stop() {

    // Check if offline mode
    if (work_offline_) {
        return true;
    }

    // Check if running
    if (do_grab_) {

        // Flag stop grabbing
        do_grab_ = false;

        // Wait for thread to end
        img_thread_.join();

        // Stop depth sensor
        bool hr = depth_sensor_->stop();
        if (!hr) {
            std::cerr << "[Error][SensorFusionSync] Unable to close the 3D sensing device."
                << std::endl;
            return false;
        }
        // Stop thermal sensor
        hr = thermal_sensor_->stop();
        if (!hr) {
            std::cerr << "[Error][SensorFusionSync] Unable to close the thermal imager device."
                << std::endl;
            return false;
        }
    }

    // if everything fine, return OKAY
    return true;
}

// Grab and sync frames
void SensorFusionSync::run() {

    unsigned long long new_id_cd = 0;
    unsigned long long new_id_t = 0;

    // Grab frames
    while (do_grab_) {

        if (pause_sync_grab_) {
            Sleep(500);
            continue;
        }

        // Grab
        if (depth_sensor_->hasNewFrame() && thermal_sensor_->hasNewFrame()) {

            new_id_cd = depth_sensor_->nextFrame(frame_to_write_->frame_depth.data(),
                frame_to_write_->frame_color.data(), frame_to_write_->skeletons.data());
            new_id_t = thermal_sensor_->nextFrame(frame_to_write_->frame_temperature.data(),
                    sensor_temp_, object_temp_);
        }
        // No new depth frame
        else {
            continue;
        }

        // New frame received
        mtx_.lock();
        RawFrame* aux = frame_to_read_;
        frame_to_read_ = frame_to_write_;
        frame_to_write_ = aux;
        frame_available_ = true;
        frame_id_++;
        mtx_.unlock();
    }

}


///////////////////////////////////////////////////////////////////////////////
//  Sensor Fusion Calibration: Thermal Calibration & Sensor Geometry Calibration
///////////////////////////////////////////////////////////////////////////////

// Do thermal calibration
bool SensorFusionSync::DoThermalCalibration() {
    
    // Pause synced grabbing
    pause_sync_grab_ = true;

    // Calibrate
    thermal_calib_->Calibrate(*thermal_sensor_);

    // Resume grabbing
    pause_sync_grab_ = false;

    // Everything okay
    return true;
}

// Do geometry calibration
bool SensorFusionSync::DoGeometryCalibration() {

    // Pause synced grabbing
    pause_sync_grab_ = true;

    // Calibrate
    while (!geo_calib_->Calibration(*thermal_sensor_, *depth_sensor_));

    // Resume grabbing
    pause_sync_grab_ = false;

    // Everything okay
    return true;
}

// Run calibrate: thermal calibration and/or sensors geometry calibration
bool SensorFusionSync::Calibrate(bool do_geometry, bool do_thermal) {

    // Check if offline mode
    if (work_offline_) {
        return false;
    }

    bool result = true;

    // Do thermal calibration
    if (do_thermal) {
        result &= DoThermalCalibration();
    }
    // Do sensors geometry calibration
    if (do_geometry) {
        result &= DoGeometryCalibration();
    }

    return result;
}

// Load calibration: thermal calibration and/or sensors geometry calibration
bool SensorFusionSync::LoadCalibration(bool do_geometry, bool do_thermal,
                                       ThermalSensorType sensor_type) {

    // Check if offline mode
    if (work_offline_) {
        return false;
    }

    // Do thermal calibration
    if (do_thermal) {
        thermal_calib_->LoadCalibration(sensor_type);
    }
    // Do sensors geometry calibration
    if (do_geometry) {
        geo_calib_->LoadCalibration(sensor_type);
    }

    return true;
}

///////////////////////////////////////////////////////////////////////////////
//  Sensor Fusion SyncedFrame Access: Thermal Calibration & Sensor Geometry Calibration
///////////////////////////////////////////////////////////////////////////////

// Gets next online frame available
unsigned long long SensorFusionSync::NextOnlineFrame(RawFrame& frame, bool force) {

    mtx_.lock();
    if (frame_available_ || force) {
        memcpy(frame.frame_depth.data(), frame_to_read_->frame_depth.data(),
            frame_depth_size_ * sizeof(unsigned short));
        memcpy(frame.frame_color.data(), frame_to_read_->frame_color.data(),
            frame_color_size_);
        memcpy(frame.frame_temperature.data(), frame_to_read_->frame_temperature.data(),
            frame_temperature_size_ * sizeof(unsigned short));
        memcpy(frame.skeletons.data(), frame_to_read_->skeletons.data(),
            frame_to_read_->skeletons.size() * sizeof(BodySkeleton));
        frame_available_ = false;
    }
    mtx_.unlock();

    return frame_id_;
}

// Gets next offline frame available
unsigned long long SensorFusionSync::NextOfflineFrame(RawFrame& frame, bool force) {
    
    // reset state
    state_ = 0;

    // open raw frame all: one message for all frame channels
    char file_name[255];
    sprintf(file_name, "%s/RAW_ALL_%.4llu.binary", dataset_path_.c_str(), frame_id_);
    FILE* pFile = fopen(file_name, "rb");
    if (pFile) {
        fread(frame.frame_color.data(), 1, frame_color_size_, pFile);
        fread(frame.frame_depth.data(), 1, frame_depth_size_ * sizeof(unsigned short), pFile);
        fread(frame.frame_temperature.data(), 1, frame_temperature_size_ * sizeof(unsigned short), pFile);
        fread(frame.skeletons.data(), 1, frame.skeletons.size() * sizeof(BodySkeleton), pFile);
        fclose(pFile);
        state_ |= FLAG_RGB | FLAG_D | FLAG_T | FLAG_S;
    }
    // If not all in one messgae, try to open one by one
    else {
        // color frame
        sprintf(file_name, "%s/RGB_%.4llu.binary", dataset_path_.c_str(), frame_id_);
        FILE* pFile = fopen(file_name, "rb");
        if (pFile) {
            fread(frame.frame_color.data(), 1, frame_color_size_, pFile);
            fclose(pFile);
            state_ |= FLAG_RGB;
        }
        else {
            //!!! std::cout << "[WARNING][SensorFusionSync::NextOfflineFrame] No color frame."
            //!!!     << std::endl;
        }

        // Depth frame
        sprintf(file_name, "%s/DEPTH_%.4llu.binary", dataset_path_.c_str(), frame_id_);
        pFile = fopen(file_name, "rb");
        if (pFile) {
            fread(frame.frame_depth.data(), 1,
                frame_depth_size_ * sizeof(unsigned short), pFile);
            fclose(pFile);
            state_ |= FLAG_D;
        }
        else {
            //!!! std::cout << "[WARNING][SensorFusionSync::NextOfflineFrame] No depth frame."
            //!!!     << std::endl;
        }

        // TEMPERATURE frame
        sprintf(file_name, "%s/TEMPERATURE_%.4llu.binary", dataset_path_.c_str(), frame_id_);
        pFile = fopen(file_name, "rb");
        if (pFile) {
            fread(frame.frame_temperature.data(), 1,
                frame_temperature_size_ * sizeof(unsigned short), pFile);
            fclose(pFile);
            state_ |= FLAG_T;
        }
        else {
            //!!! std::cout << "[WARNING][SensorFusionSync::NextOfflineFrame] No thermal frame."
            //!!!     << std::endl;
        }

        // Skeletons
        sprintf(file_name, "%s/SKELETONS_%.4llu.binary", dataset_path_.c_str(), frame_id_);
        pFile = fopen(file_name, "rb");
        if (pFile) {
            fread(frame.skeletons.data(), 1,
                frame.skeletons.size() * sizeof(BodySkeleton), pFile);
            fclose(pFile);
            state_ |= FLAG_S;
        }
        else {
            //!!!std::cout << "[WARNING][SensorFusionSync::NextOfflineFrame] No skeletons."
            //!!!    << std::endl;
        }
    }

    return frame_id_++;
}

// Gets next frame available
unsigned long long SensorFusionSync::NextFrame(RawFrame& frame, bool force) {

    if (work_offline_) {
        return NextOfflineFrame(frame, force);
    }
    else {
        return NextOnlineFrame(frame, force);
    }
}


// Gets next online registred frame available
unsigned long long SensorFusionSync::NextOnlineFrame(RawFrame& raw_frame, Frame& frame, bool force) {
    
    // Make sure input data is proper allocated
    if (frame.data.size() == 0 || raw_frame.frame_color.size() == 0 ||
        raw_frame.frame_depth.size() == 0 || raw_frame.frame_temperature.size() == 0) {
        return INVALID_FRAME_BUFFERS;
    }

    // Get frame
    NextOnlineFrame(raw_frame, force);
    
    // Clean frame
    memset(frame.data.data(), 0, frame.data.size() * sizeof(Pixel_RGBDT));

    // Color&Depth registration
    ICoordinateMapper** mapper = depth_sensor_->getCoordinateMapper();
    if (*mapper == nullptr) {
        std::cout << "[Error][SensorFusionSync::NextFrame] Invalid coordinate mapper." << std::endl;
        return INVALID_FRAME_MAPPER;
    }

    color_idx.resize(frame_depth_size_);
    (*mapper)->MapDepthPointsToColorSpace(frame_depth_size_, depth_idx.data(), 
        frame_depth_size_, raw_frame.frame_depth.data(), frame_depth_size_, color_idx.data());

    // Correct occlusions
    const float epsilon = 2.0;
    const float upper_height_C = static_cast<float>(kHeight_C - 1);
    const float upper_width_C = static_cast<float>(kWidth_C - 1);
    const size_t depth_to_occluded_width = 30000;
    for (size_t line_offset = 0; line_offset < frame_depth_size_; line_offset += kWidth_D) {
        size_t line_end = line_offset + kWidth_D;
        for (size_t c = line_offset; c < line_end; ++c) {

            // Current point
            float x = color_idx[c].X;
            float y = color_idx[c].Y;
            // Checks if point within image
            if (x < 0.f || y <= 0.f || y > upper_height_C || x > upper_width_C) {
                frame.mask[c] = false;
                continue;
            }
            frame.mask[c] = true;

            // Compute occlusion search range
            unsigned short min_depth = raw_frame.frame_depth[c];
            size_t border = c + depth_to_occluded_width / min_depth;
            if (border > line_end) {
                border = line_end;
            }

            // Checks for occlusions
            for (size_t t = c + 1; t < border; ++t) {
                // Checks if projectios overlap
                if (std::abs(color_idx[t].X - x) < epsilon){
                    // If overlap detected, check for occlusion
                    if (min_depth < raw_frame.frame_depth[t]) {
                        color_idx[t].X = -1.f;
                    }
                    else {
                        color_idx[c].X = -1.f;
                        frame.mask[c] = false;
                        break;
                    }
                }
            }
        }
    }

    // map points in 3D
    data_3D.resize(frame_depth_size_);
    (*mapper)->MapDepthPointsToCameraSpace(frame_depth_size_, depth_idx.data(),
        frame_depth_size_, raw_frame.frame_depth.data(), frame_depth_size_, data_3D.data());
    memcpy(frame.data_3D.data(), data_3D.data(), frame_depth_size_ * sizeof(CameraSpacePoint));
    GeometryModel calib;
    geo_calib_->getCalibration(calib);

    // register all data
    const float delta_disp = static_cast<float>(calib.deltaDisp);
    for (size_t idx_d = 0; idx_d < frame.data.size(); ++idx_d) {
        
        if (frame.mask[idx_d]) {
            
            // Get color image index
            int y = static_cast<int>(color_idx[idx_d].Y + 0.5);
            int x = static_cast<int>(color_idx[idx_d].X + 0.5);
            int idx_c = (y * kWidth_C + x) * kColor_bpp;
            
            // Get thermal image index
            float z = frame.data_3D[idx_d].z;
            float d = calib.invCoeff / z - delta_disp;
            int thermal_y = static_cast<int>(y + d);
            if (thermal_y < 0 || thermal_y >= kHeight_C) {
                frame.mask[idx_d] = false;
                continue;
            }

            // Get thermal location, check validity and then interpolate
            float x_t, y_t;
            geo_calib_->getThermalPoint(x, thermal_y, x_t, y_t);
            if (x_t < 0.f) {
                frame.mask[idx_d] = false;
                continue;
            }
            int x1 = static_cast<int>(x_t);
            int y1 = static_cast<int>(y_t);
            int top_left_offset = y1 * raw_frame.width_t + x1;
            float d_x = x_t - x1;
            float d_y = y_t - y1;
            float inverse_d_x = 1.f - d_x;
            float inverse_d_y = 1.f - d_y;
            float weight_1 = inverse_d_x * inverse_d_y;
            float weight_2 = d_x  * inverse_d_y;
            float weight_3 = inverse_d_x * d_y;
            float weight_4 = d_x  * d_y;
            unsigned short raw_temperature = static_cast<unsigned short>(
                weight_1 * raw_frame.frame_temperature[top_left_offset] +
                weight_2 * raw_frame.frame_temperature[top_left_offset + 1] +
                weight_3 * raw_frame.frame_temperature[top_left_offset + raw_frame.width_t] +
                weight_4 * raw_frame.frame_temperature[top_left_offset + raw_frame.width_t + 1]);

            frame.data[idx_d].depth = raw_frame.frame_depth[idx_d];
            frame.data[idx_d].r = raw_frame.frame_color[idx_c];
            frame.data[idx_d].g = raw_frame.frame_color[idx_c + 1];
            frame.data[idx_d].b = raw_frame.frame_color[idx_c + 2];
            frame.data[idx_d].temperature = raw_temperature / 100.f - 273.15f;
//                thermal_calib_->RawToTemperature(raw_temperature);
        }
    }

    return frame_id_;
}

// Compute registred frame given raw frame
bool SensorFusionSync::computeRegFrame(RawFrame& raw_frame, Frame& frame) {
    
    // Make sure input data is proper allocated
    if (frame.data.size() == 0 || raw_frame.frame_color.size() == 0 ||
        raw_frame.frame_depth.size() == 0 || raw_frame.frame_temperature.size() == 0) {
        return false;
    }

    // Clean frame (no need to clean data_3d and mask since all position will be assigned)
    memset(frame.data.data(), 0, frame.data.size() * sizeof(Pixel_RGBDT));
    
    // Color&Depth registration
    ICoordinateMapper** mapper = depth_sensor_->getCoordinateMapper();
    if (*mapper == nullptr) {
        std::cout << "[Error][SensorFusionSync::NextFrame] Invalid coordinate mapper." << std::endl;
        return false;
    }

    color_idx.resize(frame_depth_size_);
    (*mapper)->MapDepthPointsToColorSpace(frame_depth_size_, depth_idx.data(),
        frame_depth_size_, raw_frame.frame_depth.data(), frame_depth_size_, color_idx.data());

    // Correct occlusions
    const float epsilon = 2.0;
    const float upper_height_C = static_cast<float>(kHeight_C - 1);
    const float upper_width_C = static_cast<float>(kWidth_C - 1);
    const size_t depth_to_occluded_width = 30000;
    for (size_t line_offset = 0; line_offset < frame_depth_size_; line_offset += kWidth_D) {
        size_t line_end = line_offset + kWidth_D;
        for (size_t c = line_offset; c < line_end; ++c) {

            // Current point
            float x = color_idx[c].X;
            float y = color_idx[c].Y;
            // Checks if point within image
            if (x < 0.f || y <= 0.f || y > upper_height_C || x > upper_width_C) {
                frame.mask[c] = false;
                continue;
            }
            frame.mask[c] = true;

            // Compute occlusion search range
            unsigned short min_depth = raw_frame.frame_depth[c];
            size_t border = c + depth_to_occluded_width / min_depth;
            if (border > line_end) {
                border = line_end;
            }

            // Checks for occlusions
            for (size_t t = c + 1; t < border; ++t) {
                // Checks if projectios overlap
                if (std::abs(color_idx[t].X - x) < epsilon) {
                    // If overlap detected, check for occlusion
                    if (min_depth < raw_frame.frame_depth[t]) {
                        color_idx[t].X = -1.f;
                    }
                    else {
                        color_idx[c].X = -1.f;
                        frame.mask[c] = false;
                        break;
                    }
                }
            }
        }
    }

    // map points in 3D
    data_3D.resize(frame_depth_size_);
    (*mapper)->MapDepthPointsToCameraSpace(frame_depth_size_, depth_idx.data(),
        frame_depth_size_, raw_frame.frame_depth.data(), frame_depth_size_, data_3D.data());
    memcpy(frame.data_3D.data(), data_3D.data(), frame_depth_size_ * sizeof(CameraSpacePoint));
    GeometryModel calib;
    geo_calib_->getCalibration(calib);

    // register all data
    const float delta_disp = static_cast<float>(calib.deltaDisp);
    for (size_t idx_d = 0; idx_d < frame.data.size(); ++idx_d) {

        if (frame.mask[idx_d]) {

            // Get color image index
            int y = static_cast<int>(color_idx[idx_d].Y + 0.5);
            int x = static_cast<int>(color_idx[idx_d].X + 0.5);
            int idx_c = (y * kWidth_C + x) * kColor_bpp;

            // Get thermal image index
            float z = frame.data_3D[idx_d].z;
            float d = calib.invCoeff / z - delta_disp;
            int thermal_y = static_cast<int>(y + d);
            if (thermal_y < 0 || thermal_y >= kHeight_C) {
                frame.mask[idx_d] = false;
                continue;
            }

            // Get thermal location, check validity and then interpolate
            float x_t, y_t;
            geo_calib_->getThermalPoint(x, thermal_y, x_t, y_t);
            if (x_t < 0.f) {
                frame.mask[idx_d] = false;
                continue;
            }
            int x1 = static_cast<int>(x_t);
            int y1 = static_cast<int>(y_t);
            int top_left_offset = y1 * raw_frame.width_t + x1;
            float d_x = x_t - x1;
            float d_y = y_t - y1;
            float inverse_d_x = 1.f - d_x;
            float inverse_d_y = 1.f - d_y;
            float weight_1 = inverse_d_x * inverse_d_y;
            float weight_2 = d_x  * inverse_d_y;
            float weight_3 = inverse_d_x * d_y;
            float weight_4 = d_x  * d_y;
            unsigned short raw_temperature = static_cast<unsigned short>(
                weight_1 * raw_frame.frame_temperature[top_left_offset] +
                weight_2 * raw_frame.frame_temperature[top_left_offset + 1] +
                weight_3 * raw_frame.frame_temperature[top_left_offset + raw_frame.width_t] +
                weight_4 * raw_frame.frame_temperature[top_left_offset + raw_frame.width_t + 1]);

            frame.data[idx_d].depth = raw_frame.frame_depth[idx_d];
            frame.data[idx_d].r = raw_frame.frame_color[idx_c];
            frame.data[idx_d].g = raw_frame.frame_color[idx_c + 1];
            frame.data[idx_d].b = raw_frame.frame_color[idx_c + 2];
            frame.data[idx_d].temperature = raw_temperature / 100.f - 273.15f;
            //                thermal_calib_->RawToTemperature(raw_temperature);
        }
    }
    return true;
}

// Gets next offline registered frame available
unsigned long long SensorFusionSync::NextOfflineFrame(RawFrame& raw_frame, Frame& frame, bool force) {

    // Get frame
    NextOfflineFrame(raw_frame, force);
    frame_id_--; // compensate for raw data loader increment

    // Fused frame
    char file_name[255];
    sprintf(file_name, "%s/RGBDT_%.4llu.binary", dataset_path_.c_str(), frame_id_);
    FILE* pFile = fopen(file_name, "rb");
    if (pFile) {
        fread(frame.data.data(), 1, frame_depth_size_ * sizeof(Pixel_RGBDT), pFile);
        fclose(pFile);
        state_ |= FLAG_RGBDT;
    }
    else {
        //!!!std::cout << "[WARNING][SensorFusionSync::NextOfflineFrame] No RGBD-T frame."
        //!!!    << std::endl;
    }

    // Frame mask
    sprintf(file_name, "%s/MASK_%.4llu.binary", dataset_path_.c_str(), frame_id_);
    pFile = fopen(file_name, "rb");
    if (pFile) {
        std::vector<char> mask(frame_depth_size_);
        fread(mask.data(), 1, frame_depth_size_, pFile);
        fclose(pFile);
        for (size_t idx = 0; idx < frame.mask.size(); ++idx) {
            frame.mask[idx] = mask[idx];
        }
        state_ |= FLAG_MASK;
    }
    else {
        //!!!std::cout << "[WARNING][SensorFusionSync::NextOfflineFrame] No frame mask."
        //!!!    << std::endl;
    }

    // XYZ frame
    sprintf(file_name, "%s/CLOUD_%.4llu.binary", dataset_path_.c_str(), frame_id_);
    pFile = fopen(file_name, "rb");
    if (pFile) {
        fread(frame.data_3D.data(), 1, frame_depth_size_ * sizeof(Pixel_XYZ), pFile);
        fclose(pFile);
        state_ |= FLAG_XYZ;
    }
    else {
        //!!!std::cout << "[WARNING][SensorFusionSync::NextOfflineFrame] No XYZ frame."
        //!!!    << std::endl;
    }

    // Check frame state
    if (state_ == 0) {
        std::cout << "\n No more frames to read." << std::endl;
    }
    else if ((state_ & STATE_FULL_REG_FRAME) != STATE_FULL_REG_FRAME &&
             (state_ & STATE_FULL_RAW_FRAME) == STATE_FULL_RAW_FRAME) {
        if (!computeRegFrame(raw_frame, frame)) {
            std::cout << "\n Offline frame registration failed." << std::endl;
        }
        state_ |= STATE_FULL_REG_FRAME;
    }

    //if (state_ != STATE_FULL_FRAME) {
        //!!! std::cout << "[ERROR][SensorFusionSync::NextOfflineFrame] Unable to "
        //!!!     "read full frame." << std::endl;
    //}

    return frame_id_++;
}

// Gets next registred frame available
unsigned long long SensorFusionSync::NextFrame(RawFrame& raw_frame, Frame& frame, int& status, bool force) {

    unsigned long long frame_id = 0;
    if (work_offline_) {
        frame_id = NextOfflineFrame(raw_frame, frame, force);
        status = state_;
    }
    else {
        frame_id = NextOnlineFrame(raw_frame, frame, force);
        status = STATE_FULL_FRAME;
    }

    return frame_id;
}
