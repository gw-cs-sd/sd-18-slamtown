//============================================================================
// Name        : ViewerKeyPatch.cpp
// Author      : CosmaC
// Date        : November, 2016
// Copyright   : GWU Research
// Description : Thermal modeling keypatches viewer
//============================================================================

#include <SensorFusion/SensorFusionSync.h>
#include <ImageProcessing/ImageUtils.h>
#include <KeyPatch/KeyPatch.h>

#include <opencv/cv.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


// App configuration
bool view_depth = true;
bool view_color = true;
bool view_temperature = true;
bool view_skeleton = true;
bool view_keypatches = true;
constexpr ThermalSensorType type = LEPTON_V2;

void main(int argc, char* argv[]) {

    if (argc == 1) {
        std::cout << "\n[Error] Please specify the dataset path.\n"
            "DatasetPlayer.exe path_to_dataset" << std::endl;
        exit(0);
    }

    // Create sensors interfaces
    SensorFusionSync sensor_fusion(argv[1], 100);

    // config depth sensor
    bool hr_i = sensor_fusion.start();
    if (!hr_i) {
        std::cerr << "[Error][SensorFusion] Unable to connect to the 3D sensing device."
            << std::endl;
        exit(0);
    }

    // Load pre-calibrated models
    //sensor_fusion.LoadCalibration(true, true);

    // Prepare bufers
    RawFrame frame;
    sensor_fusion.AllocateAndInitializeRawFrame(frame);

    Frame reg_frame;
    sensor_fusion.AllocateAndInitializeRegFrame(reg_frame);

    std::vector<BodyThermalModel> bodies;

    // Grab and show frames
    unsigned long long old_id = 9999999;
    unsigned long long new_id = 0;
    int status = 0;
    int nb_frames = 0;
    int nb_all_frames = 0;
    clock_t begin = clock();
    while (1) {

        // Grab
        if (sensor_fusion.HasNewFrame()) {
            new_id = sensor_fusion.NextFrame(frame, reg_frame, status);
            if (status == 0) {
                break;
            }
        }
        ++nb_all_frames;

        // Check for new frame
        if (old_id != new_id) {

            // Count the new frame
            old_id = new_id;
            ++nb_frames;

            // Compute thermal model
            bodies.clear();
            ComputeBodyThermalModel(reg_frame, frame, bodies);
        }

        // View frame
        VisualizationConfig config_visualization{ view_depth, view_color,
            view_temperature, view_skeleton, view_keypatches, 0, 3 };
        char key = ViewFrame(reg_frame, frame, bodies, config_visualization);
        if (key == 27) {
            cv::destroyAllWindows();
            break;
        }
        else if (key == 'c') {
            sensor_fusion.Calibrate(true, true);
        }
        else if (key == 'l') {
            sensor_fusion.LoadCalibration(true, true, type);
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
}