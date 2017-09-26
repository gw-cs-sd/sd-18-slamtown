//============================================================================
// Name        : ThermalModelRecorder.cpp
// Author      : CosmaC
// Date        : December, 2016
// Copyright   : GWU Research
// Description : Thermal models recorder
//============================================================================

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

// App configuration
bool view_depth = false;
bool view_color = false;
bool view_temperature = true;
bool view_skeleton = true;
bool view_keypatches = true;
constexpr ThermalSensorType type = LEPTON_V2;
const int metadata_size = 5;
bool publish_thermal_model = true;

bool LoadTimestamps(const std::string& path, std::vector<TimestampIDPair>& timestamps) {

    if (path.compare("") == 0) {
        return false;
    }
    
    char filename[255];
    sprintf(filename, "%s/timestamps.binary", path.c_str());
    std::ifstream input_file(filename, std::ios::in | std::ios::binary);
    if (!input_file.is_open()) {
        std::cout << "[ERROR] Unable to open timestamps file:" << filename << std::endl;
        return false;
    }

    TimestampIDPair ts;
    int count = 0;
    while (input_file.read(reinterpret_cast<char *>(&ts.id), sizeof(int)) &&
           input_file.read(reinterpret_cast<char *>(&ts.timestamp), sizeof(double)) &&
           input_file.read(reinterpret_cast<char *>(&ts.sensor_temp), sizeof(int))) {
        timestamps.push_back(ts);
        printf("Timestamp %f for %d\n", ts.timestamp, ts.id);
    }
    input_file.close();
    return true;
}

void FindClosestTimestamp(const int id, 
                          const std::vector<TimestampIDPair>& timestamps,
                          size_t& previous_ts_index,
                          double& timestamp,
                          int& sensor_temp) {
    
    // Find frame index based on frame id and timesamps ids
    size_t i = previous_ts_index;
    for ( ; i < timestamps.size(); ++i) {
        if (id <= timestamps[i].id) {
            break;
        }
    }

    timestamp = -1.0;
    sensor_temp = -1;
    // Frame index after last timestamp
    if (i == timestamps.size() && (id - timestamps.back().id) < 5) {
        timestamp = timestamps.back().timestamp;
        sensor_temp = timestamps.back().sensor_temp;
    }
    // Frame index same as previous frame
    else if (i == previous_ts_index && (timestamps[i].id - id) < 5) {
        timestamp = timestamps[i].timestamp;
        sensor_temp = timestamps[i].sensor_temp;
    }
    // Frame index between 2 timestamps (due to frames drop)
    else if (i > previous_ts_index && i < timestamps.size()) {
        // in between 2 ids
        double norm = 1.0 / (timestamps[i].id - timestamps[i-1].id);
        double w1 = 1.0 - (id - timestamps[i-1].id) * norm;
        double w2 = 1.0 - (timestamps[i].id - id) * norm;
        timestamp = w1 * timestamps[i - 1].timestamp + w2 * timestamps[i].timestamp;
        sensor_temp = static_cast<int>(w1 * timestamps[i - 1].sensor_temp +
                                       w2 * timestamps[i].sensor_temp);
    }

    // Previous frame index become current frame index
    previous_ts_index = i;
}

bool MatchScore(const Frame&  prev_frame,
                const cv::Point2i& prev_joint,
                const Frame& frame,
                cv::Point2i& joint,
                const int radius,
                std::vector<float>& scores) {

    // SAD over a window
    const int width = frame.width;
    const int height = frame.height;
    const auto& data = frame.data;
    const auto& mask = frame.mask;
    const auto& prev_data = prev_frame.data;
    const auto& prev_mask = prev_frame.mask;

    // check data validity
    if (prev_joint.x < radius || prev_joint.y < radius ||
        joint.x < radius || joint.y < radius ||
        prev_joint.x + radius >= width || prev_joint.y + radius >= height ||
        joint.x + radius >= width || joint.y + radius >= height) {
        return false;
    }

    // Compute scores
    float depth_sum = 0;
    float temp_sum = 0;
    float color_sum = 0;
    size_t count = 0;
    for (int col = -radius; col <= radius; ++col) {
        for (int row = -radius; row <= radius; ++row) {
            int idx = (joint.x + col) + (joint.y + row)*width;
            int prev_idx = (prev_joint.x + col) + (prev_joint.y + row)*width;
            if (!mask[idx] || !prev_mask[prev_idx]) {
                continue;
            }
            count++;
            depth_sum += std::abs(data[idx].depth - prev_data[prev_idx].depth);
            temp_sum += std::abs(data[idx].temperature - prev_data[prev_idx].temperature);
            color_sum += (std::abs(data[idx].r - prev_data[prev_idx].r) +
                          std::abs(data[idx].g - prev_data[prev_idx].g) +
                          std::abs(data[idx].b - prev_data[prev_idx].b))/3.f;
        }
    }
    if (count == 0) {
        return false;
    }

    // Store matching scores
    scores.push_back(depth_sum / count);
    scores.push_back(temp_sum / count);
    scores.push_back(color_sum / count);
    return true;
}

bool BestMatch(const Frame&  prev_frame, 
               const cv::Point2i& prev_joint,
               const Frame& frame,
               cv::Point2i& joint) {

    /////////////////////////////////////////////
    constexpr int window_width = 15;
    constexpr int window_height = 10;
    constexpr int step = 1;
    constexpr int radius = 20; // SAD kernel size = 2 * radius + 1
    /////////////////////////////////////////////
    constexpr float w_d = 0.2f * (1.f / 100.f); // 100 mm
    constexpr float w_t = 0.4f * (1.f / 2.f);   // 2 degrees
    constexpr float w_c = 0.4f * (1.f / 50.f);  // 50 units

    float min_score = 999999.f;
    cv::Point2i min_pos;
    for (int col = joint.x - window_width; col < joint.x + window_width; col+=step) {
        for (int row = joint.y - window_height; row < joint.y + window_height; row+=step) {

            std::vector<float> scores; 
            if (!MatchScore(prev_frame, prev_joint,
                            frame, cv::Point2i(col, row),
                            radius, scores)) {
                continue;
            }

            // Store min score
            if (scores.size() > 0) {
                float total_score = w_d * scores[0] + w_t * scores[1] + w_c * scores[2];
                if (total_score < min_score) {
                    min_score = total_score;
                    min_pos = cv::Point2i(col, row);
                }
            }
        }
    }

    if (min_score < 0.3f) {
        // Debug code
        /*const float min_th = 10.f;
        const float max_th = 40.f;
        cv::Mat temperature_image;
        FrameToThermalImage(frame, temperature_image, min_th, max_th, true);
        cv::rectangle(temperature_image, 
            cv::Rect(joint.x-radius, joint.y-radius, 2*radius, 2*radius),
            cv::Scalar(0,0,0), 2);
        cv::rectangle(temperature_image,
            cv::Rect(min_pos.x - radius, min_pos.y - radius, 2 * radius, 2 * radius),
            cv::Scalar(255, 255, 255), 2);
        cv::imshow("Refine", temperature_image);
        cv::waitKey(10);*/
        joint = min_pos;
        return true;
    }

    return false;
}

void RefineJoint(const BodySkeleton& prev_skeleton,
                 const Frame& prev_frame,
                 BodySkeleton& skeleton, 
                 const Frame& frame,
                 JointType type) {

    if (prev_skeleton.joints[type].is_valid) {

        cv::Point2i prev_joint(prev_skeleton.joints[type].x, prev_skeleton.joints[type].y);
        cv::Point2i joint(prev_joint);
        if (!skeleton.joints[type].is_valid) {
            if (BestMatch(prev_frame, prev_joint, frame, joint)) {
                skeleton.joints[type].x = joint.x;
                skeleton.joints[type].y = joint.y;
                skeleton.joints[type].is_valid = true;
            }
        }
        else {
            if (BestMatch(prev_frame, prev_joint, frame, joint)) {
                skeleton.joints[type].x = (joint.x + skeleton.joints[type].x) / 2;
                skeleton.joints[type].y = (joint.y + skeleton.joints[type].y) / 2;
            }
        }
    }
}

void RefineLocation(const BodySkeleton& prev_skeleton,
                    const Frame& prev_frame,
                    BodySkeleton& skeleton,
                    const Frame& frame) {

    // Check left arm
    // Set previous joint location
    cv::Point2i prev_joint(0, 0);
    size_t prev_size = 0;
    if (prev_skeleton.joints[JointType_HandLeft].is_valid) {
        ++prev_size;
        prev_joint.x += prev_skeleton.joints[JointType_HandLeft].x;
        prev_joint.y += prev_skeleton.joints[JointType_HandLeft].y;
    }
    if (prev_skeleton.joints[JointType_WristLeft].is_valid) {
        ++prev_size;
        prev_joint.x += prev_skeleton.joints[JointType_WristLeft].x;
        prev_joint.y += prev_skeleton.joints[JointType_WristLeft].y;
    }
    if (prev_size > 0) {

        // Compute avg location
        prev_joint.x /= prev_size;
        prev_joint.y /= prev_size;

        // Set new joint location
        cv::Point2i joint(0, 0);
        size_t size = 0;
        if (skeleton.joints[JointType_HandLeft].is_valid) {
            ++size;
            joint.x += skeleton.joints[JointType_HandLeft].x;
            joint.y += skeleton.joints[JointType_HandLeft].y;
        }
        if (skeleton.joints[JointType_WristLeft].is_valid) {
            ++size;
            joint.x += skeleton.joints[JointType_WristLeft].x;
            joint.y += skeleton.joints[JointType_WristLeft].y;
        }
        if (size > 0) {
            joint.x /= size;
            joint.y /= size;
        }
        else {
            joint = prev_joint;
        }

        BestMatch(prev_frame, prev_joint, frame, joint);
        skeleton.joints[JointType_HandLeft].x = joint.x;
        skeleton.joints[JointType_HandLeft].y = joint.y;
        skeleton.joints[JointType_HandLeft].is_valid = true;
        skeleton.joints[JointType_WristLeft].x = joint.x;
        skeleton.joints[JointType_WristLeft].y = joint.y;
        skeleton.joints[JointType_WristLeft].is_valid = true;
    }

    // Check right arm
    // Set previous joint location
    prev_joint = cv::Point2i(0, 0);
    prev_size = 0;
    if (prev_skeleton.joints[JointType_HandRight].is_valid) {
        ++prev_size;
        prev_joint.x += prev_skeleton.joints[JointType_HandRight].x;
        prev_joint.y += prev_skeleton.joints[JointType_HandRight].y;
    }
    if (prev_skeleton.joints[JointType_WristRight].is_valid) {
        ++prev_size;
        prev_joint.x += prev_skeleton.joints[JointType_WristRight].x;
        prev_joint.y += prev_skeleton.joints[JointType_WristRight].y;
    }
    if (prev_size > 0) {

        // Compute avg location
        prev_joint.x /= prev_size;
        prev_joint.y /= prev_size;

        // Set new joint location
        cv::Point2i joint(0, 0);
        size_t size = 0;
        if (skeleton.joints[JointType_HandRight].is_valid) {
            ++size;
            joint.x += skeleton.joints[JointType_HandRight].x;
            joint.y += skeleton.joints[JointType_HandRight].y;
        }
        if (skeleton.joints[JointType_WristRight].is_valid) {
            ++size;
            joint.x += skeleton.joints[JointType_WristRight].x;
            joint.y += skeleton.joints[JointType_WristRight].y;
        }
        if (size > 0) {
            joint.x /= size;
            joint.y /= size;
        }
        else {
            joint = prev_joint;
        }

        BestMatch(prev_frame, prev_joint, frame, joint);
        skeleton.joints[JointType_HandRight].x = joint.x;
        skeleton.joints[JointType_HandRight].y = joint.y;
        skeleton.joints[JointType_HandRight].is_valid = true;
        skeleton.joints[JointType_WristRight].x = joint.x;
        skeleton.joints[JointType_WristRight].y = joint.y;
        skeleton.joints[JointType_WristRight].is_valid = true;
    }
}

void CheckSkeleton(const Frame& prev_frame,
                   const RawFrame& prev_raw_frame,
                   const Frame& frame,
                   RawFrame& raw_frame) {

    std::vector<BodySkeleton>& skeletons = raw_frame.skeletons;
    const std::vector<BodySkeleton>& previous_skeletons = prev_raw_frame.skeletons;
    
    for (auto& skeleton : skeletons) {
        
        // Check if it matches a previous skeleton

        for (const auto& prev_skeleton : previous_skeletons) {
            if (skeleton.is_tracked && prev_skeleton.is_tracked &&
                skeleton.id == prev_skeleton.id) {
                //RefineLocation(prev_skeleton, prev_frame, skeleton, frame);
                RefineJoint(prev_skeleton, prev_frame, skeleton, frame, JointType_HandLeft);
                RefineJoint(prev_skeleton, prev_frame, skeleton, frame, JointType_WristLeft);
                RefineJoint(prev_skeleton, prev_frame, skeleton, frame, JointType_HandRight);
                RefineJoint(prev_skeleton, prev_frame, skeleton, frame, JointType_WristRight);
                break;
            }
        }
    }
}

///////////////////////////////////////////////////////////////////////////////
// MAIN APP
///////////////////////////////////////////////////////////////////////////////
void main(int argc, char* argv[]) {

    // Check inputs
    if (argc == 1) {
        std::cout << "\n[Error] Please specify the dataset path.\n"
            "DatasetPlayer.exe path_to_dataset" << std::endl;
        exit(0);
    }

    // Load timestamps
    std::vector<TimestampIDPair> timestamps;
    if (!LoadTimestamps(argv[1], timestamps)) {
        exit(0);
    }

    // Create sensors interfaces
    SensorFusionSync sensor_fusion(argv[1], 1);
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

    // Load pre-calibrated models
    //sensor_fusion.LoadCalibration(true, true);

    // Prepare bufers
    size_t idx_c = 0;
    size_t idx_p = 1;
    RawFrame frame[2];
    sensor_fusion.AllocateAndInitializeRawFrame(frame[idx_c]);
    sensor_fusion.AllocateAndInitializeRawFrame(frame[idx_p]);

    Frame reg_frame[2];
    sensor_fusion.AllocateAndInitializeRegFrame(reg_frame[idx_c]);
    sensor_fusion.AllocateAndInitializeRegFrame(reg_frame[idx_p]);

    std::vector<BodyThermalModel> bodies;

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
            new_id = sensor_fusion.NextFrame(frame[idx_c], reg_frame[idx_c], frame_status);
            if (frame_status == 0) {
                break;
            }
        }
        ++nb_all_frames;

        // Check for new frame
        if (old_id != new_id) {

            // Count the new frame
            old_id = new_id;
            ++nb_frames;

            // Check skeleton consistency
            CheckSkeleton(reg_frame[idx_p], frame[idx_p], reg_frame[idx_c], frame[idx_c]);

            // Compute thermal model
            bodies.clear();
            ComputeBodyThermalModel(reg_frame[idx_c], frame[idx_c], bodies);
            
            // Get timestamp
            double timestamp = 0.0;
            int sensor_temperature = 0;
            FindClosestTimestamp(new_id, timestamps, old_index, timestamp, sensor_temperature);

            // Store thermal model
            if (publish_thermal_model) {
                for (const auto& body : bodies) {
                    ThermalModelLog thermal_model(body);
                    thermal_model.timestamp = timestamp;
                    thermal_model.sensor_temperature = sensor_temperature;
                    publishThermalModel(thermal_model, publisher, metadata_size);
                }
            }
        }

        // View frame
        VisualizationConfig config_visualization {view_depth, view_color, 
            view_temperature, view_skeleton, view_keypatches, 0, 3};
        char key = ViewFrame(reg_frame[idx_c], frame[idx_c], bodies, config_visualization);
        if (key == 27) {
            cv::destroyAllWindows();
            break;
        }
        else if (key == 'l') {
            sensor_fusion.LoadCalibration(true, true, type);
        }
        else if (key == 'p') {
            if (publish_thermal_model) {
                printf("\n[ThermalModselRecorder] Pause logging data. \n");
                publish_thermal_model = false;
            }
            else {
                printf("\n[ThermalModelRecorder] Resume logging data. \n");
                publish_thermal_model = true;
            }
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

        // Update active index
        idx_p = idx_c;
        idx_c = (idx_c + 1) % 2;
    }

    // Stop sensors
    hr_i = sensor_fusion.stop();
    if (!hr_i) {
        std::cerr << "[Error][ViewerFusion] Unable to close the thermal imager device."
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