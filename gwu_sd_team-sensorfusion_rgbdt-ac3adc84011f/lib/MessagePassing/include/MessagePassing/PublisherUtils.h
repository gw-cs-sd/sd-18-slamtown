//============================================================================
// Name        : PublisherUtils.h
// Author      : CosmaC
// Date        : January, 2017
// Copyright   : GWU Research
// Description : Utils library for message publishing protocol
//============================================================================

#pragma once

#include <KeyPatch/KeyPatch.h>

// ZeroMQ
#include <zmq.hpp>

struct PublisherRawConfig {
    bool save_color;
    bool save_depth;
    bool save_temperature;
    bool save_skeletons;
    bool save_frame;
    size_t color_frame_bpp;
    size_t metadata_size;

    PublisherRawConfig(bool sc, bool sd, bool st, bool sk, bool sf, size_t bpp, size_t m_s)
        : save_color(sc),
        save_depth(sd),
        save_temperature(st),
        save_skeletons(sk),
        save_frame(sf),
        color_frame_bpp(bpp),
        metadata_size(m_s) {};
};

struct PublisherRegConfig {
    bool save_rgbdt;
    bool save_xyz;
    bool save_mask;
    size_t metadata_size;

    PublisherRegConfig(bool rgbdt, bool xyz, bool mask, size_t m_s)
        : save_rgbdt(rgbdt),
        save_xyz(xyz),
        save_mask(mask),
        metadata_size(m_s) {};
};

/**
 * @brief Publish root dir
 */
void publishRootDir(zmq::socket_t *publisher, 
                    const std::string& dataset_path,
                    int metadata_size);

/**
 * @brief Publish thermal calibration
 */
void publishThermalCalibration(zmq::socket_t *publisher, 
                               SensorFusionSync& sensor_fusion, 
                               int metadata_size);

/**
 * @brief Publish geometry calibration
 */
void publishGeometrycCalibration(zmq::socket_t *publisher,
                                 SensorFusionSync& sensor_fusion,
                                 int metadata_size);

/**
 * @brief Publish registered frame planes
 */
void publishRegFrame(zmq::socket_t *publisher, 
                     Frame& frame, int frame_id, 
                     const PublisherRegConfig& config);

/**
 * @brief Publish raw frame planes
 */
bool publishRawFrame(zmq::socket_t *publisher, 
                     RawFrame& frame, int frame_id, 
                     const PublisherRawConfig& config);
/**
 * @brief Publish frame timestamp
 */
void publishTimestamp(zmq::socket_t *publisher,
                      double timestamp,
                      int sensor_temperature,
                      int frame_id,
                      int metadata_size);

/**
 * @brief Publish the new thermal model
 */
bool publishThermalModel(const ThermalModelLog& thermal_model,
                         zmq::socket_t *publisher,
                         int metadata_size);
