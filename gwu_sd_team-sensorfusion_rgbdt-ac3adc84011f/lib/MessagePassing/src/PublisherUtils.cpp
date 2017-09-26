//============================================================================
// Name        : PublisherUtils.cpp
// Author      : CosmaC
// Date        : January, 2017
// Copyright   : GWU Research
// Description : Utils library for message publishing protocol
//============================================================================

#include <MessagePassing/PublisherUtils.h>
#include <KeyPatch/KeyPatch.h>

// ZeroMQ
#include <zmq.hpp>

// Publish thermal calibration
void publishThermalCalibration(zmq::socket_t *publisher, 
                               SensorFusionSync& sensor_fusion,
                               int metadata_size) {

    const int msg_size = sizeof(ThermalCalibration) + metadata_size; // +1 msg type +4 frame ID
    zmq::message_t msg_calibT(msg_size);
    char* msg_data = (char *)msg_calibT.data();
    msg_data[0] = T_CALIB_FRAME;
    ((int*)(msg_data + 1))[0] = 0;
    memcpy(msg_data + metadata_size, &(sensor_fusion.getThermalCalibration()), msg_size);
    publisher->send(msg_calibT);
}

// Publish geometry calibration
void publishGeometrycCalibration(zmq::socket_t *publisher,
                                 SensorFusionSync& sensor_fusion,
                                 int metadata_size) {

    const int msg_size = sizeof(GeometryModel) + metadata_size; // +1 msg type +4 frame ID
    zmq::message_t msg_calib(msg_size);
    char* msg_data = (char *)msg_calib.data();
    msg_data[0] = G_CALIB_FRAME;
    ((int*)(msg_data + 1))[0] = 0;
    memcpy(msg_data + metadata_size, &(sensor_fusion.getGeometryCalibration()), msg_size);
    publisher->send(msg_calib);
}

// publish registered frame planes
void publishRegFrame(zmq::socket_t *publisher,
                     Frame& frame, int frame_id,
                     const PublisherRegConfig& config) {

    // RGBD-T
    if (config.save_rgbdt && frame.data.size() != 0) {
        const static int msg_size = frame.width * frame.height * sizeof(Pixel_RGBDT);
        zmq::message_t msg(msg_size + config.metadata_size);
        char* msg_data = (char *)msg.data();
        msg_data[0] = REGISTERED_FRAME_RGBDT;
        ((int*)(msg_data + 1))[0] = frame_id;
        memcpy(msg_data + config.metadata_size, frame.data.data(), msg_size);
        publisher->send(msg);
    }

    // Mask
    if (config.save_mask && frame.mask.size() != 0) {
        const static int msg_size = frame.width * frame.height;
        zmq::message_t msg(msg_size + config.metadata_size);
        char* msg_data = (char *)msg.data();
        msg_data[0] = REGISTERED_FRAME_MASK;
        ((int*)(msg_data + 1))[0] = frame_id;
        for (size_t idx = 0; idx < frame.mask.size(); ++idx) {
            if (frame.mask[idx]) {
                msg_data[idx + config.metadata_size] = 1;
            }
            else {
                msg_data[idx + config.metadata_size] = 0;
            }
        }
        publisher->send(msg);
    }

    // XYZ
    if (config.save_xyz && frame.data_3D.size() != 0) {
        const static int msg_size = frame.width * frame.height * sizeof(Pixel_XYZ);
        zmq::message_t msg(msg_size + config.metadata_size);
        char* msg_data = (char *)msg.data();
        msg_data[0] = REGISTERED_FRAME_XYZ;
        ((int*)(msg_data + 1))[0] = frame_id;
        memcpy(msg_data + config.metadata_size, frame.data_3D.data(), msg_size);
        publisher->send(msg);
    }
}

// publish raw frame planes
bool publishRawFrame(zmq::socket_t *publisher,
                     RawFrame& frame, int frame_id,
                     const PublisherRawConfig& confi) {

    // Raw Frame - all frame channels
    if (confi.save_frame) {
        if (frame.frame_color.size() == 0) {
            std::cout << "[ERROR][publishRawFrame] Missing raw color frame. \n";
            return false;
        }
        if (frame.frame_depth.size() == 0) {
            std::cout << "[ERROR][publishRawFrame] Missing raw depth frame. \n";
            return false;
        }
        if (frame.frame_temperature.size() == 0) {
            std::cout << "[ERROR][publishRawFrame] Missing raw thermal frame. \n";
            return false;
        }
        const int msg_size_c = frame.frame_color.size();
        const int msg_size_d = frame.frame_depth.size() * sizeof(unsigned short);
        const int msg_size_t = frame.frame_temperature.size() * sizeof(unsigned short);
        const int msg_size_s = frame.skeletons.size() * sizeof(BodySkeleton);
        const int msg_size = msg_size_c + msg_size_d + msg_size_t + msg_size_s;
        zmq::message_t msg(msg_size + confi.metadata_size);
        char* msg_data = (char *)msg.data();
        msg_data[0] = RAW_FRAME_ALL;
        ((int*)(msg_data + 1))[0] = frame_id;
        memcpy(msg_data + confi.metadata_size,
               frame.frame_color.data(), msg_size_c);
        memcpy(msg_data + confi.metadata_size + msg_size_c,
               frame.frame_depth.data(), msg_size_d);
        memcpy(msg_data + confi.metadata_size + msg_size_c + msg_size_d,
               frame.frame_temperature.data(), msg_size_t);
        memcpy(msg_data + confi.metadata_size + msg_size_c + msg_size_d + msg_size_t,
               frame.skeletons.data(), msg_size_s);
        publisher->send(msg);

        return true;
    }

    bool return_value = true; // all planes are available, if one missing change to false

    // RGB
    if (confi.save_color && frame.frame_color.size() != 0) {
        const int msg_size = frame.frame_color.size();
        zmq::message_t msg(msg_size + confi.metadata_size);
        char* msg_data = (char *)msg.data();
        msg_data[0] = RAW_FRAME_RGB;
        ((int*)(msg_data + 1))[0] = frame_id;
        memcpy(msg_data + confi.metadata_size, frame.frame_color.data(), msg_size);
        publisher->send(msg);
    }
    else if (confi.save_color) {
        return_value = false;
    }

    // D
    if (confi.save_depth && frame.frame_depth.size() != 0) {
        const int msg_size = frame.frame_depth.size() * sizeof(unsigned short);
        zmq::message_t msg(msg_size + confi.metadata_size);
        char* msg_data = (char *)msg.data();
        msg_data[0] = RAW_FRAME_D;
        ((int*)(msg_data + 1))[0] = frame_id;
        memcpy(msg_data + confi.metadata_size, frame.frame_depth.data(), msg_size);
        publisher->send(msg);
    }
    else if (confi.save_depth) {
        return_value = false;
    }

    // T
    if (confi.save_temperature && frame.frame_temperature.size() != 0) {
        const int msg_size = frame.frame_temperature.size() * sizeof(unsigned short);
        zmq::message_t msg(msg_size + confi.metadata_size);
        char* msg_data = (char *)msg.data();
        msg_data[0] = RAW_FRAME_T;
        ((int*)(msg_data + 1))[0] = frame_id;
        memcpy(msg_data + confi.metadata_size, frame.frame_temperature.data(), msg_size);
        publisher->send(msg);
    }
    else if (confi.save_temperature) {
        return_value = false;
    }

    // Skeletons
    if (confi.save_skeletons && frame.skeletons.size() != 0) {
        const int msg_size = frame.skeletons.size() * sizeof(BodySkeleton);
        zmq::message_t msg(msg_size + confi.metadata_size);
        char* msg_data = (char *)msg.data();
        msg_data[0] = RAW_SKELETON;
        ((int*)(msg_data + 1))[0] = frame_id;
        memcpy(msg_data + confi.metadata_size, frame.skeletons.data(), msg_size);
        publisher->send(msg);
    }
    return return_value;
}

// publish root dir
void publishRootDir(zmq::socket_t *publisher,
                    const std::string& dataset_path,
                    int metadata_size) {

    zmq::message_t msg(255 + metadata_size); // +1 msg type +4 frame ID
    char* msg_data = (char *)msg.data();
    msg_data[0] = METADATA_FRAME;
    ((int*)(msg_data + 1))[0] = 0;
    dataset_path.copy(msg_data + metadata_size, dataset_path.length());
    publisher->send(msg);
}

// publish timestamp
void publishTimestamp(zmq::socket_t *publisher,
                      double timestamp,
                      int sensor_temperature,
                      int frame_id,
                      int metadata_size) {

    zmq::message_t msg(sizeof(double) + sizeof(int) + metadata_size); // +1 msg type +4 frame ID
    char* msg_data = (char *)msg.data();
    msg_data[0] = TIMESTAMP;
    ((int*)(msg_data + 1))[0] = frame_id;
    ((double*)(msg_data + metadata_size))[0] = timestamp;
    ((int*)(msg_data + metadata_size + sizeof(double)))[0] = sensor_temperature;
    publisher->send(msg);
}

// Publish the new thermal model
bool publishThermalModel(const ThermalModelLog& thermal_model,
                         zmq::socket_t *publisher,
                         int metadata_size) {

    if (publisher == nullptr || metadata_size == 0) {
        // unable to send the message
        return false;
    }

    int message_size = sizeof(ThermalModelLog);
    zmq::message_t msg(message_size + metadata_size); // +1 msg type +4 frame ID
    char* msg_data = (char *)msg.data();
    msg_data[0] = TM_FRAME;
    ((int*)(msg_data + 1))[0] = 0;
    memcpy(msg_data + metadata_size, &thermal_model, message_size);

    // Everything ready
    return publisher->send(msg);;
}