//============================================================================
// Name        : DatasetSubscriber.cpp
// Author      : CosmaC
// Date        : May, 2016
// Copyright   : GWU Research
// Description : Dataset subscriber collects dataset packages sent by the 
//               publishers (using ZMQ) and it saves it to the disk 
//============================================================================


#include "ProgramOptions.h"
#include "SensorFusion/SensorFusionSync.h"
#include "KeyPatch/KeyPatch.h"

// ZeroMQ
#include <zmq.hpp>

// OpenCV
//#include <opencv2\core.hpp>
//#include <opencv2\highgui.hpp>

// C++/C
#include <string>
#include <iostream>
#include <iterator>
#include <fstream>
#ifndef _WIN32
#include <unistd.h>
#else
#include <windows.h>
#define sleep(n)	Sleep(n)
#endif

///////////////////////////////////////////////////////////////////////////////
ProgramOptions po;

size_t width_c = 0;
size_t width_d = 0;
size_t width_t = 0;

size_t height_c = 0;
size_t height_d = 0;
size_t height_t = 0;
///////////////////////////////////////////////////////////////////////////////

// Save raw frame to file
unsigned long long package_id = 0;
FILE* gFile = nullptr;
void SaveRawFrame(char* msg_data, int frame_id, char* root_dir, int type, int msg_size) {
    
    char file_name[255];
    
    // Raw frame
    static int count = 0;
    if (type == RAW_FRAME_ALL) {
        int msg_size_ref = (width_c*height_c * po.color_frame_bbp) + 
                           (width_d*height_d*sizeof(unsigned short)) +
                           (width_t*height_t*sizeof(unsigned short));
        if (msg_size < msg_size_ref) {
            std::cout << "[WARNING][SaveRawFrame] Invalid raw frame size." << std::endl;
        }
        if (count == 0) {
            sprintf(file_name, "%s/RAW_ALL_%.4llu.binary", root_dir, package_id++);
            gFile = fopen(file_name, "wb");
        }
        ++count;

        if (gFile) {
            fwrite(msg_data, 1, msg_size, gFile);
        }

        if (count == po.batch_size) {
            fclose(gFile);
            count = 0;
        }
    }
    // Color frame
    else if (type == RAW_FRAME_RGB) {
        if (msg_size != (width_c*height_c*po.color_frame_bbp)) {
            std::cout << "[WARNING][SaveRawFrame] Invalid color frame size." << std::endl;
        }

        sprintf(file_name, "%s/RGB_%.4d.binary", root_dir, frame_id);
        FILE* pFile = fopen(file_name, "wb");
        if (pFile) {
            fwrite(msg_data, 1, msg_size, pFile);
            fclose(pFile);
        }
        //sprintf(file_name, "%s/RGB_%.4d.png", root_dir, frame_id);
        //cv::Mat color_img(height_c, width_c, CV_8UC4, msg_data);
        //cv::imwrite(file_name, color_img);
    }
    // Depth frame
    else if (type == RAW_FRAME_D) {
        if (msg_size != (width_d*height_d*sizeof(unsigned short))) {
            std::cout << "[WARNING][SaveRawFrame] Invalid depth frame size." << std::endl;
        }

        sprintf(file_name, "%s/DEPTH_%.4d.binary", root_dir, frame_id);
        FILE* pFile = fopen(file_name, "wb");
        if (pFile) {
            fwrite(msg_data, 1, msg_size, pFile);
            fclose(pFile);
        }
    }
    // TEMPERATURE frame
    else if (type == RAW_FRAME_T) {
        if (msg_size != (width_t*height_t*sizeof(unsigned short))) {
            std::cout << "[WARNING][SaveRawFrame] Invalid thermal frame size." << std::endl;
        }

        sprintf(file_name, "%s/TEMPERATURE_%.4d.binary", root_dir, frame_id);
        FILE* pFile = fopen(file_name, "wb");
        if (pFile) {
            fwrite(msg_data, 1, msg_size, pFile);
            fclose(pFile);
        }
    }
    // Skeletons
    else if (type == RAW_SKELETON) {
        
        sprintf(file_name, "%s/SKELETONS_%.4d.binary", root_dir, frame_id);
        FILE* pFile = fopen(file_name, "wb");
        if (pFile) {
            fwrite(msg_data, 1, msg_size, pFile);
            fclose(pFile);
        }
    }
}


// Save registered frame to file
void SaveRegFrame(char* msg_data, int frame_id, char* root_dir, int type, int msg_size) {

    // Color-Depth-Temperature frame
    char file_name[255];
    if (type == REGISTERED_FRAME_RGBDT) {
        if (msg_size != (width_d * height_d * sizeof(Pixel_RGBDT))) {
            std::cout << "[WARNING][SaveRegFrame] Invalid registered frame size." << std::endl;
        }

        sprintf(file_name, "%s/RGBDT_%.4d.binary", root_dir, frame_id);
        FILE* pFile = fopen(file_name, "wb");
        if (pFile) {
            fwrite(msg_data, 1, msg_size, pFile);
            fclose(pFile);
        }
    }
    // Mask frame (write as a char array)
    else if (type == REGISTERED_FRAME_MASK) {
        if (msg_size != (width_d * height_d)) {
            std::cout << "[WARNING][SaveRegFrame] Invalid frame mask size." << std::endl;
        }
        
        sprintf(file_name, "%s/MASK_%.4d.binary", root_dir, frame_id);
        FILE* pFile = fopen(file_name, "wb");
        if (pFile) {
            fwrite(msg_data, 1, msg_size, pFile);
            fclose(pFile);
        }
    }
    // 3D data frame
    else if (type == REGISTERED_FRAME_XYZ) {
        if (msg_size != (width_d * height_d * sizeof(Pixel_XYZ))) {
            std::cout << "[WARNING][SaveRegFrame] Invalid XYZ frame size." << std::endl;
        }

        sprintf(file_name, "%s/CLOUD_%.4d.binary", root_dir, frame_id);
        FILE* pFile = fopen(file_name, "wb");
        if (pFile) {
            fwrite(msg_data, 1, msg_size, pFile);
            fclose(pFile);
        }
    }
}


// Save geometry calibration
void SaveGCalib(char* msg_data, const char* root_dir) {

    GeometryModel* calib = (GeometryModel*)(msg_data);
    // Saves calibration log
    char file_name[255];
    sprintf(file_name, "%s/geometry_model.log", root_dir);
    std::ofstream myfile;
    myfile.open(file_name);
    myfile << "\nHomography: \n"
        << calib->h[0] << "  " << calib->h[1] << "  " << calib->h[2] << std::endl
        << calib->h[3] << "  " << calib->h[4] << "  " << calib->h[5] << std::endl
        << calib->h[6] << "  " << calib->h[7] << "  " << calib->h[8] << std::endl;
    myfile << "Depth params: \n   - deltaDisp=" << calib->deltaDisp
        << "\n   - invCoeffDisp=" << calib->invCoeff
        << "\n   - Color image resolution=" << calib->width_c << " x " << calib->height_c
        << "\n   - Depth image resolution=" << calib->width_d << " x " << calib->height_d
        << "\n   - Thermal image resolution=" << calib->width_t << " x " << calib->height_t
        << "\n   - Thermal image scale=" << calib->scale_t
        << std::endl;
    myfile.close();

    // Save image resolution
    width_c = calib->width_c;
    width_t = calib->width_t;
    width_d = calib->width_d;
    height_c = calib->height_c;
    height_t = calib->height_t;
    height_d = calib->height_d;

    // Save to binary file calibration structure
    sprintf(file_name, "%s/geometry_model.bin", root_dir);
    std::ofstream output_file(file_name, std::ios::binary);
    output_file.write((char*)calib, sizeof(GeometryModel));
    output_file.close();
}


// Save thermal calibration
void SaveTCalib(char* msg_data, const char* root_dir) {
    
    ThermalModel* model = (ThermalModel*)(msg_data);
    // Saves calibration log
    char file_name[255];
    sprintf(file_name, "%s/thermal_model.log", root_dir);
    std::ofstream myfile;
    myfile.open(file_name);
    myfile << "Thermal model:"
        << "\n   - Slope=" << model->fnc_slope
        << "\n   - Offset=" << model->fnc_offset
        << "\n   - Correlation=" << model->fnc_correlation
        << std::endl;
    myfile.close();

    // Save to binary file calibration structure
    sprintf(file_name, "%s/thermal_model.bin", root_dir);
    std::ofstream output_file(file_name, std::ios::binary);
    output_file.write((char*)model, sizeof(ThermalModel));
    output_file.close();
}


// Save thermal model log
void SaveThermalModel(char* msg_data) {

    ThermalModelLog* thermal_model = (ThermalModelLog*)msg_data;

    // Open thermal profile file coresponding to the given body id
    char filename[255] = "";
    sprintf(filename, "ThermalProfile_%d.log", thermal_model->id);
    FILE* file = fopen(filename, "a");
    if (file) {
        fprintf(file, "%d \t%f \t", thermal_model->id, thermal_model->timestamp);
        for (size_t i = 0; i < MAX_KeyPatch; ++i) {
            fprintf(file, "%f \t", thermal_model->temperature_avg[i]);
        }
        for (size_t i = 0; i < MAX_KeyPatch; ++i) {
            fprintf(file, "%f \t", thermal_model->temperature_variance[i]);
        }
        for (size_t i = 0; i < MAX_KeyPatch; ++i) {
            fprintf(file, "%f \t", thermal_model->z_avg[i]);
        }
        for (size_t i = 0; i < MAX_KeyPatch; ++i) {
            fprintf(file, "%f \t", thermal_model->x[i]);
        }
        for (size_t i = 0; i < MAX_KeyPatch; ++i) {
            fprintf(file, "%f \t", thermal_model->y[i]);
        }
        for (size_t i = 0; i < MAX_KeyPatch; ++i) {
            fprintf(file, "%f \t", thermal_model->z[i]);
        }
        fprintf(file, "%d \n", thermal_model->sensor_temperature);
        fclose(file);
    }
    else {
        std::cerr << "\n[ERROR][SaveThermalModel] Unable to open log file: " 
            << filename << std::endl;
    }
}


// Save frame timestamp
void SaveTimestamp(const char *data, int frame_id, const char *root_dir) {

    constexpr size_t size_timestamp = sizeof(double) + sizeof(int); // timestamp + sensor_temp
    char file_name[255];
    sprintf(file_name, "%s/timestamps.binary", root_dir);
    FILE* file = fopen(file_name, "ab");
    if (file) {
        fwrite(&frame_id, sizeof(int), 1, file);
        fwrite(data, size_timestamp, 1, file);
        fclose(file);
    }
    else {
        std::cerr << "\n[ERROR][SaveTimestamp] Unable to open log file: "
            << file_name << std::endl;
    }
}

// Main loop
int main(int argc, char** argv) {

    // Parse command line arguments 
    po = ParseProgramOptions(argc, argv);
    int metadata_size = po.metadata_size; 

    ///  Prepare subscriber context and socket (0MQ)
    zmq::context_t context(1);
    zmq::socket_t subscriber(context, ZMQ_SUB);
    subscriber.setsockopt(ZMQ_RCVHWM, 4000);
    subscriber.connect("tcp://localhost:5555");
    subscriber.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    /// Log info (processed vs skipped frames for each type)
    int frames = 0;	            // #number of processed frames
    int skipped_frames = 0;	    // #number of skipped frames
    int previous_frame_id = -1;	// id of the latest frame received

    /// Infinite loop
    int frame_id = 0;	// Received frame id
    int raw_planes = 0; // Number of image planes for the raw image, maxim 3
    int reg_planes = 0; // Number of image planes for the registered image, maxim 3
    char root_dir[255] = ".";
    while (true) {

        // Gets the message
        zmq::message_t msg;
        subscriber.recv(&msg);

        // Unpacks the meesage
        int msg_size = msg.size() - metadata_size; // 5 bytes for metadata
        char* msg_data = (char *)msg.data();
        if (msg.size() > 4) {
            frame_id = ((int*)(msg_data + 1))[0];
        }
        //std::cout << "Received " << msg.size() << " bytes with id " << frame_id << std::endl;

        // Process the message
        if (msg_data[0] == TIMESTAMP) {
            if (previous_frame_id != frame_id) { // new frame

                if ((previous_frame_id + 1) != frame_id) { // wrong frame id
                    skipped_frames += frame_id - (previous_frame_id + 1);
                }
                ++frames;
                previous_frame_id = frame_id;
                raw_planes = 0;
                reg_planes = 0;
            }

            SaveTimestamp(msg_data + metadata_size, frames, root_dir);
        }
        else if (msg_data[0] == RAW_FRAME_ALL) {
            // Timestamp message always comes first, so the id should be alreay set
            // If missing timestamp, the ids will be different
            if (previous_frame_id != frame_id) {
                std::cout << "[WARNING] Something went wrong. Missing timestamp message.\n";
                if ((previous_frame_id + 1) != frame_id) { // wrong frame id
                    skipped_frames += frame_id - (previous_frame_id + 1);
                }
                ++frames;
                previous_frame_id = frame_id;
            }
            SaveRawFrame(msg_data + metadata_size, frames, root_dir, msg_data[0], msg_size);
        }
        else if (msg_data[0] == RAW_FRAME_RGB ||
            msg_data[0] == RAW_FRAME_D || 
            msg_data[0] == RAW_FRAME_T ||
            msg_data[0] == RAW_SKELETON) {

            // Update debug data
            if (previous_frame_id == frame_id) { // new plane
                raw_planes++;
            }
            else { // new frame

                if ((previous_frame_id + 1) != frame_id) { // wrong frame id
                    skipped_frames += frame_id - (previous_frame_id + 1);
                }
                ++frames;
                previous_frame_id = frame_id;
                raw_planes = 1;
                reg_planes = 0;
            }
            // Save frame
            SaveRawFrame(msg_data + metadata_size, frames, root_dir, msg_data[0], msg_size);
        }
        else if (msg_data[0] == REGISTERED_FRAME_RGBDT ||
                 msg_data[0] == REGISTERED_FRAME_MASK ||
                 msg_data[0] == REGISTERED_FRAME_XYZ) {

            if (previous_frame_id == frame_id) { // new plane
                reg_planes++;
            }
            else { // new frame

                if ((previous_frame_id + 1) != frame_id) { // wrong frame id
                    skipped_frames += frame_id - (previous_frame_id + 1);
                }
                
                ++frames;
                previous_frame_id = frame_id;
                raw_planes = 0;
                reg_planes = 1;
            }
            // Save frame
            SaveRegFrame(msg_data + metadata_size, frames, root_dir, msg_data[0], msg_size);
        }
        else if (msg_data[0] == G_CALIB_FRAME) {
            // geometric calibration model
            SaveGCalib(msg_data + metadata_size, root_dir);
        }
        else if (msg_data[0] == T_CALIB_FRAME) {
            // thermal calibration model
            SaveTCalib(msg_data + metadata_size, root_dir);
        }
        else if (msg_data[0] == METADATA_FRAME) {
            // Root dir
            sprintf(root_dir, "%s", msg_data + metadata_size);
            previous_frame_id = frame_id - 1;
            continue;
        }
        else if (msg_data[0] == TM_FRAME) {
            // New thermal model
            SaveThermalModel(msg_data + metadata_size);
        }
        else if (msg_data[0] == END_FRAME) {
            // Quit message
            break;
        }
        else {
            // Unknown message
            std::cout << "Unknown message" << std::endl;
            continue;
        }

        // Show log
        if (frame_id % 9 == 0) {
            printf("\rProcessed frames %d - skipped %d", frames, skipped_frames);
        }

    } // infinite loop

    // Stop ZMQ
    subscriber.close();
    context.close();
    
    // Quit Subscriber
    return 0;
}