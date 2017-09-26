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
#include <string>

struct ProgramOptions {
    std::string dataset_path;

    bool view_depth;
    bool view_color;
    bool view_temperature;
    bool view_skeleton;

    bool save_reg_RGBDT;
    bool save_reg_XYZ;
    bool save_reg_mask;

    ThermalSensorType type;
    size_t metadata_size;

    ProgramOptions()
        : dataset_path{""},
        view_depth(true),
        view_color(true),
        view_temperature(true),
        view_skeleton(true),

        save_reg_RGBDT(true),
        save_reg_XYZ(true),
        save_reg_mask(true),

        type(LEPTON_V2),
        metadata_size(5) {};

    std::string toString() {
        std::ostringstream string_stream;
        string_stream << "Program configuration: \n"
            << "\t- Dataset path: " << dataset_path << std::endl
            << "\t- Lepton type: " << (type == LEPTON_V2 ? "LEPTON_V2" : "LEPTON_V3") << std::endl
            << "\t- View color: " << (view_color ? "on" : "off") << std::endl
            << "\t- View depth: " << (view_depth ? "on" : "off") << std::endl
            << "\t- View temperature: " << (view_temperature ? "on" : "off") << std::endl
            << "\t- View skeletons: " << (view_skeleton ? "on" : "off") << std::endl
            << "\t- Save RGBDT: " << (save_reg_RGBDT ? "on" : "off") << std::endl
            << "\t- Save XYZ: " << (save_reg_XYZ ? "on" : "off") << std::endl
            << "\t- Save mask: " << (save_reg_mask ? "on" : "off") << std::endl
            << std::endl; 
        return string_stream.str();
    }
};

// Program options parser
ProgramOptions ParseProgramOptions(int argc, char** argv) {

    // Parse input arguments
    cxxopts::Options options("DatasetRegistration",
        "\n\nDatasetRegistration: Loads raw data from given folder and registers all data planes.");
    
    options.add_options()
        ("d,dataset_path", "[REQUIRED] Dataset path contaning the raw data", 
            cxxopts::value<std::string>())
        ("l,lepton_type", "[REQUIRED] Lepton IR camera model, one from Lepton_V2 and Lepton_V3.",
            cxxopts::value<std::string>())
        
        ("vc", "View color frame.")
        ("vd", "View depth frame.")
        ("vt", "View thermal frame.")
        ("vs", "View skeletons.")

        ("s_rgbdt", "Save RGBDT registered frame.")
        ("s_xyz", "Save XYZ registered frame.")
        ("s_mask", "Save registered frame mask.")

        ("h,help", "Print help");

    options.parse(argc, argv);

    if (options.count("help")) {
        std::cout << options.help() << std::endl;
        exit(0);
    }

    if (!options.count("lepton_type") || !options.count("dataset_path")) {
        std::cerr << "[ERROR] Not enough input parameters." << std::endl;
        std::cout << options.help() << std::endl;
        exit(0);
    }

    // Create configuration struct
    ProgramOptions po;
    po.dataset_path = options["dataset_path"].as<std::string>();

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

    po.save_reg_RGBDT = options["s_rgbdt"].as<bool>();
    po.save_reg_XYZ = options["s_xyz"].as<bool>();
    po.save_reg_mask = options["s_mask"].as<bool>();

    // Show selecetd options
    std::cout << po.toString() << std::endl;

    return po;
}