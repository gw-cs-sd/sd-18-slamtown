//============================================================================
// Name        : DatasetRegistration.cpp
// Author      : CosmaC
// Date        : August, 2017
// Copyright   : GWU Research
// Description : Register a raw dataset
//============================================================================

#include <ProgramOptions.h>
#include <SensorFusion/SensorFusionSync.h>
#include <ImageProcessing/ImageUtils.h>
#include <KeyPatch/KeyPatch.h>
#include <MessagePassing/PublisherUtils.h>

// OpenCV
#include <opencv/cv.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

// ZeroMQ
#include <zmq.hpp>

struct TimestampIDPair {
    double timestamp;
    int id;
    int sensor_temp;
};
// ZeroMQ driver
// Create publisher
zmq::context_t *context = nullptr;
zmq::socket_t *publisher = nullptr;

///////////////////////////////////////////////////////////////////////////////
// MAIN APP
///////////////////////////////////////////////////////////////////////////////
void main(int argc, char* argv[]) {

    // Check inputs
    ProgramOptions po = ParseProgramOptions(argc, argv);

    // Create sensors interfaces
    SensorFusionSync sensor_fusion(po.dataset_path, 1);
    bool hr_i = sensor_fusion.start(false, true);
    if (!hr_i) {
        std::cerr << "[Error][SensorFusion] Unable to connect to the 3D sensing device."
            << std::endl;
        exit(0);
    }

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
    publishRootDir(publisher, po.dataset_path.c_str(), po.metadata_size);

    // Load pre-calibrated models
    //sensor_fusion.LoadCalibration(true, true);
    publishThermalCalibration(publisher, sensor_fusion, po.metadata_size);
    publishGeometrycCalibration(publisher, sensor_fusion, po.metadata_size);

    // Prepare bufers
    RawFrame frame;
    sensor_fusion.AllocateAndInitializeRawFrame(frame);

    Frame reg_frame;
    sensor_fusion.AllocateAndInitializeRegFrame(reg_frame);

    // Grab and show frames
    unsigned long long old_id = 9999999;
    unsigned long long new_id = 0;
    int nb_frames = 0;
    int nb_all_frames = 0;
    int frame_status = 0;
    size_t old_index = 0;
    clock_t begin = clock();
    while (1) {

        // Grab
        if (sensor_fusion.HasNewFrame()) {
            new_id = sensor_fusion.NextFrame(frame, reg_frame, frame_status);
            // If frame registration fails or no more frames --> stop
            if ((frame_status&STATE_FULL_REG_FRAME) != STATE_FULL_REG_FRAME) {
                break;
            }
        }
        ++nb_all_frames;

        // Check for new frame
        if (old_id != new_id) {

            // Count the new frame
            old_id = new_id;
            ++nb_frames;

            // Publish frame
            PublisherRegConfig reg_config{ po.save_reg_RGBDT, po.save_reg_XYZ,
                po.save_reg_mask, po.metadata_size };
            publishRegFrame(publisher, reg_frame, new_id, reg_config);
        }

        // View frame
        VisualizationConfig config_visualization {po.view_depth, po.view_color,
            po.view_temperature, po.view_skeleton, false, 0, 3};
        char key = ViewFrame(reg_frame, frame, config_visualization);
        if (key == 27) { // esc
            cv::destroyAllWindows();
            break;
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
            std::cout << "\t- 1,2,3: Change between visualization modes" << std::endl;
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
        std::cerr << "[Error][ViewerFusion] Unable to close the thermal imager device."
            << std::endl;
        exit(0);
    }

    // ZeroMQ driver
    zmq::message_t quit_msg(po.metadata_size); // +1 msg type +4 frame ID
    char* msg_data = (char *)quit_msg.data();
    msg_data[0] = END_FRAME;
    publisher->send(quit_msg);

    publisher->close();
    context->close();
    delete context;
    delete publisher;
}