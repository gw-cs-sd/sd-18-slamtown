//============================================================================
// Name        : DatasetPublisher.cpp
// Author      : CosmaC
// Date        : September, 2016
// Copyright   : GWU Research
// Description : Dataset publisher collects data from all the sensors and it
//               publishes (over ZMQ) to be stored by thestored/processed by
//               the subscribers
//============================================================================

#include "ProgramOptions.h"
#include "SensorFusion/SensorFusionSync.h"
#include "ImageProcessing/ImageUtils.h"
#include "MessagePassing/PublisherUtils.h"

// OpenCV
#include <opencv/cv.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// C/C++
#include <direct.h>

// ZeroMQ
#include <zmq.hpp>

// ZeroMQ driver
// Create publisher
zmq::context_t *context = nullptr;
zmq::socket_t *publisher = nullptr;

// Create dataset folder
void CreateFolderStruct(std::string& dataset_path) {

    // Root folder
    time_t now = time(0);
    struct tm  tstruct;
    tstruct = *localtime(&now);
    char root[255];
    strftime(root, 255, "f:/GWU/Datasets/Dataset_%Y-%m-%d_%H-%M", &tstruct);
    mkdir(root);
    dataset_path = root;
}

enum AppState {WARM_UP, PAUSE, COLLECT};

// Publisher main loop
void main(int argc, char** argv) {

    // Read 
    ProgramOptions po = ParseProgramOptions(argc, argv);
    AppState app_state = COLLECT;
    if (po.warmup_wait) {
        app_state = WARM_UP;
    }
    const bool publish_reg_frame_plane = po.save_reg_mask | po.save_reg_RGBDT | 
                                         po.save_reg_XYZ;
    const bool publish_raw_frame_plane = po.save_raw_color | po.save_raw_depth | 
                                         po.save_raw_temperature | po.save_raw_skeleton |
                                         po.save_raw_frame;

    // ZeroMQ driver
    context = new zmq::context_t(1);
    publisher = new zmq::socket_t(*context, ZMQ_PUB);
    publisher->setsockopt(ZMQ_SNDHWM, 4000);
    publisher->bind("tcp://*:5555");
    while (!publisher->connected()) {
        Sleep(500);
    }
    Sleep(1000); // wait to finish connection, avoid lossing packages because init

    // Dataset root dir
    std::string dataset_path;
    CreateFolderStruct(dataset_path);
    publishRootDir(publisher, dataset_path, po.metadata_size);
    
    // Create sensors interfaces
    SensorFusionSync sensor_fusion(po.type);
    //SensorFusionSync sensor_fusion("F:/GWU/Datasets/Dataset_2017-08-22_15-41/", 1);
    // config sensors rig
    bool hr_i = sensor_fusion.start();
    if (!hr_i) {
        std::cerr << "[Error][DatasetPublisher] Unable to connect to the 3D sensing device."
            << std::endl;
        exit(0);
    }

    // Load pre-calibrated models
    sensor_fusion.LoadCalibration(true, true, po.type);
    // Save thermal calibration
    publishThermalCalibration(publisher, sensor_fusion, po.metadata_size);
    publishGeometrycCalibration(publisher, sensor_fusion, po.metadata_size);

    // Prepare bufers
    RawFrame frame;
    sensor_fusion.AllocateAndInitializeRawFrame(frame);

    Frame reg_frame;
    sensor_fusion.AllocateAndInitializeRegFrame(reg_frame);

    // Grab and show frames
    int metadata_size = 5; // +1 msg type; +4 frameID
    unsigned int old_id = 9999999;
    unsigned int new_id = 0;
    int status = 0;
    int nb_frames = 0;
    int nb_all_frames = 0;
    clock_t begin = clock();
    clock_t clock_0 = clock();
    double timestamp;
    float center_temperature = -1.f;
    while (1) {

        // Grab new frame
        if (sensor_fusion.HasNewFrame()) {
            new_id = sensor_fusion.NextFrame(frame, reg_frame, status);
            if (status == 0) {
                break;
            }
            clock_t clock_now = clock();
            timestamp = double(clock_now - clock_0) / CLOCKS_PER_SEC;
        }
        ++nb_all_frames;

        // Check for new frame
        if (old_id != new_id && app_state == COLLECT) {

            // Count the new frame
            old_id = new_id;
            ++nb_frames;

            // Publish timestamp
            publishTimestamp(publisher, timestamp, sensor_fusion.getSensorTemperature(), new_id, metadata_size);

            // Publish raw frame
            if (publish_raw_frame_plane) {
                PublisherRawConfig raw_config{ po.save_raw_color, po.save_raw_depth,
                                               po.save_raw_temperature, po.save_raw_skeleton,
                                               po.save_raw_frame, sensor_fusion.getColorBPP(),
                                               po.metadata_size };
                if (!publishRawFrame(publisher, frame, new_id, raw_config)) {
                    std::cout << "[WARNING] Unable to publish all requested raw frame channels.\n";
                }
            }

            // Publish registered frame
            if (publish_reg_frame_plane) {
                PublisherRegConfig reg_config{po.save_reg_RGBDT, po.save_reg_XYZ,
                                              po.save_reg_mask, po.metadata_size};
                publishRegFrame(publisher, reg_frame, new_id, reg_config);
            }
        }

        // View frame & Control
        // TO DO this may take too much time.... try to run only when new frame in
        VisualizationConfig config_visualization{ po.view_depth, po.view_color,
            po.view_temperature, po.view_skeleton, false,
            sensor_fusion.getSensorTemperature(), app_state };
        config_visualization.center_temperature = center_temperature;
        center_temperature = -1.f;  // reset central pixel temperature
        char key = ViewFrame(reg_frame, frame, config_visualization);
        if (key == 27) { // esc
            cv::destroyAllWindows();
            break;
        }
        // Do thermal calibration
        else if (key == 't') {
            sensor_fusion.Calibrate(false, true);
            publishThermalCalibration(publisher, sensor_fusion, po.metadata_size);
            publishRootDir(publisher, dataset_path, metadata_size);
        }
        // Do geometry calibration
        else if (key == 'g') {
            sensor_fusion.Calibrate(true, false);
            publishGeometrycCalibration(publisher, sensor_fusion, po.metadata_size);
            publishRootDir(publisher, dataset_path, metadata_size);
        }
        // Load calibration
        else if (key == 'l') {
            sensor_fusion.LoadCalibration(true, true, po.type);
            publishThermalCalibration(publisher, sensor_fusion, po.metadata_size);
            publishGeometrycCalibration(publisher, sensor_fusion, po.metadata_size);
        }
        // Pause/Resume data collection
        else if (key == 'p') {
            app_state = PAUSE;
        }
        else if (key == 'r') {
            app_state = COLLECT;
        }
        // Do FFC
        else if (key == 'f') {
            sensor_fusion.doFFC();
        }
        // Create new storage folder
        else if (key == 'n') {
            CreateFolderStruct(dataset_path);
            publishRootDir(publisher, dataset_path, po.metadata_size);
        }
        // View pixel temperature
        else if (key == 'v') {
            const size_t center_idx = (reg_frame.width * reg_frame.height) / 2 + reg_frame.width / 2;
            center_temperature = reg_frame.data[center_idx].temperature;
        }
        // Basic visualization
        else if (key == '1') {
            po.view_color = false;
            po.view_depth = false;
            po.view_skeleton = true;
            po.view_temperature = false;
            cv::destroyAllWindows();
        }
        // Advance visualization: Skeleton + Temperature
        else if (key == '2') {
            po.view_color = false;
            po.view_depth = false;
            po.view_skeleton = true;
            po.view_temperature = true;
            cv::destroyAllWindows();
        }
        // Advance visualization: Skeleton + Temperature + Color
        else if (key == '3') {
            po.view_color = true;
            po.view_depth = false;
            po.view_skeleton = true;
            po.view_temperature = true;
            cv::destroyAllWindows();
        }
        // Help: print options
        else if (key == 'h') {
            std::cout << "\n\n Options: " << std::endl;
            std::cout << "\t- t: do thermal calibration" << std::endl;
            std::cout << "\t- g: do geometry calibration" << std::endl;
            std::cout << "\t- l: load calibration" << std::endl;
            std::cout << "\t- p: pause dataset collection" << std::endl;
            std::cout << "\t- r: resume dataset collection" << std::endl;
            std::cout << "\t- f: run FFC on demand" << std::endl;
            std::cout << "\t- n: create and publish new folder to store data" << std::endl;
            std::cout << "\t- v: view true temperature of the pixel in the middle" << std::endl;
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

    // Stop sensors
    hr_i = sensor_fusion.stop();
    if (!hr_i) {
        std::cerr << "[Error][DatasetPublisher] Unable to close the thermal imager device."
            << std::endl;
        exit(0);
    }

    // ZeroMQ driver
    zmq::message_t quit_msg(metadata_size); // +1 msg type +4 frame ID
    char* msg_data = (char *)quit_msg.data();
    msg_data[0] = END_FRAME;
    publisher->send(quit_msg);

    publisher->close();
    context->close();
    delete context;
    delete publisher;
}