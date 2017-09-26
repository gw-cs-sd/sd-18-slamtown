//============================================================================
// Name        : ProgramOptions.h
// Author      : CosmaC
// Date        : January, 2017
// Copyright   : GWU Research
// Description : Program options parser
//============================================================================

#pragma once

#include "cxxopts/cxxopts.hpp"
#include "ThermalSensor/ThermalSensor.h"

#include <sstream>

struct ProgramOptions {
    bool view_depth;
    bool view_color;
    bool view_temperature;
    bool view_skeleton;

    bool save_raw_color;
    bool save_raw_depth;
    bool save_raw_temperature;
    bool save_raw_skeleton;
    bool save_raw_frame;

    bool save_reg_RGBDT;
    bool save_reg_XYZ;
    bool save_reg_mask;

    bool warmup_wait;
    bool do_ffc;

    ThermalSensorType type;
    size_t metadata_size;

    ProgramOptions()
        : view_depth(true),
        view_color(true),
        view_temperature(true),
        view_skeleton(true),

        save_raw_color(true),
        save_raw_depth(true),
        save_raw_temperature(true),
        save_raw_skeleton(true),
        save_raw_frame(true),

        save_reg_RGBDT(true),
        save_reg_XYZ(true),
        save_reg_mask(true),

        warmup_wait(true),
        do_ffc(true),

        type(LEPTON_V2),
        metadata_size(5) {};

    std::string toString() {
        std::ostringstream string_stream;
        string_stream << "Program configuration: \n"
            <<"\t- Lepton type: " << (type == LEPTON_V2 ? "LEPTON_V2" : "LEPTON_V3") << std::endl
            << "\t- View color: " << (view_color ? "on" : "off") << std::endl
            << "\t- View depth: " << (view_depth ? "on" : "off") << std::endl
            << "\t- View temperature: " << (view_temperature ? "on" : "off") << std::endl
            << "\t- View skeletons: " << (view_skeleton ? "on" : "off") << std::endl
            << "\t- Save raw color: " << (save_raw_color ? "on" : "off") << std::endl
            << "\t- Save raw depth: " << (save_raw_depth ? "on" : "off") << std::endl
            << "\t- Save raw temperature: " << (save_raw_temperature ? "on" : "off") << std::endl
            << "\t- Save raw skeletons: " << (save_raw_skeleton ? "on" : "off") << std::endl
            << "\t- Save raw frame: " << (save_raw_frame ? "on" : "off") << std::endl
            << "\t- Save RGBDT: " << (save_reg_RGBDT ? "on" : "off") << std::endl
            << "\t- Save XYZ: " << (save_reg_XYZ ? "on" : "off") << std::endl
            << "\t- Save mask: " << (save_reg_mask ? "on" : "off") << std::endl
            << "\t- Warm-up wait: " << (warmup_wait ? "on" : "off") << std::endl
            << "\t- Do FFC after warm-up: " << (do_ffc ? "on" : "off") << std::endl
            << std::endl; 
        return string_stream.str();
    }
};

// Program options parser
ProgramOptions ParseProgramOptions(int argc, char** argv) {

    // Parse input arguments
    cxxopts::Options options("DatasetPublisher",
        "\n\nDatasetPublisher: Grabs data from all the sensors and publishes it using ZMQ. Please"
        " run the DatasetSubscriber first (this app will collect and store to disk all the data).");
    
    options.add_options()
        ("lepton_type", "[REQUIRED] Lepton IR camera model, one from Lepton_V2 and Lepton_V3.",
            cxxopts::value<std::string>())
        
        ("vc", "View color frame.")
        ("vd", "View depth frame.")
        ("vt", "View thermal frame.")
        ("vs", "View skeletons.")

        ("src", "Save to disk raw color frame.")
        ("srd", "Save to disk raw depth frame.")
        ("srt", "Save to disk raw thermal frame.")
        ("srs", "Save to disk raw skeletons.")
        ("srf", "Save to disk raw frame (color+depth+thermal+skeletons).")

        ("s_rgbdt", "Save RGBDT registered frame.")
        ("s_xyz", "Save XYZ registered frame.")
        ("s_mask", "Save registered frame mask.")

        ("warm_up", "Warm-up wait till sensor internal temperature is stable")
        ("ffc", "Do FFC after warmup")
        
        ("h,help", "Print help");

    options.parse(argc, argv);

    if (options.count("help")) {
        std::cout << options.help() << std::endl;
        exit(0);
    }

    if (!options.count("lepton_type")) {
        std::cerr << "[ERROR] Not enough input parameters." << std::endl;
        std::cout << options.help() << std::endl;
        exit(0);
    }

    // Create configuration struct
    ProgramOptions po;
    if (0 == options["lepton_type"].as<std::string>().compare("Lepton_V2")) {
        po.type = LEPTON_V2;
    }
    else if (0 == options["lepton_type"].as<std::string>().compare("Lepton_V3")) {
        po.type = LEPTON_V3;
    }
    else {
        std::cerr << "[ERROR] Unknown Lepton camera type." << std::endl;
        std::cout << options.help() << std::endl;
        exit(0);
    }

    po.view_color = options["vc"].as<bool>();
    po.view_depth = options["vd"].as<bool>();
    po.view_temperature = options["vt"].as<bool>();
    po.view_skeleton = options["vs"].as<bool>();

    po.save_raw_color = options["src"].as<bool>();
    po.save_raw_depth = options["srd"].as<bool>();
    po.save_raw_temperature = options["srt"].as<bool>();
    po.save_raw_skeleton = options["srs"].as<bool>();
    po.save_raw_frame = options["srf"].as<bool>();

    po.save_reg_RGBDT = options["s_rgbdt"].as<bool>();
    po.save_reg_XYZ = options["s_xyz"].as<bool>();
    po.save_reg_mask = options["s_mask"].as<bool>();

    po.warmup_wait = options["warm_up"].as<bool>();
    po.do_ffc = options["ffc"].as<bool>();

    // Show selecetd options
    std::cout << po.toString() << std::endl;

    return po;
}