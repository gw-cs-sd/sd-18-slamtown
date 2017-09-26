//============================================================================
// Name        : KeyPatch.cpp
// Author      : CosmaC
// Date        : November, 2016
// Copyright   : GWU Research
// Description : KeyPatches detector based on fused frame and skeleton points
//============================================================================

#include <KeyPatch/KeyPatch.h>
#include <SensorFusion/SensorFusionSync.h>

#include <queue>

#define PATCH_TEMP_MAX_DIFF 0.5 // temperature in Celsius degrees
#define PATCH_DEPTH_MAX_DIFF 10 // depth in mm
const float PATCH_RADIUS_MAX_2D[MAX_KeyPatch] = { 5.f, 10.f, 15.f,
                                               5.f, 10.f, 15.f,
                                              10.f,  5.f, 15.f };
                                              //!!!10.f, 10.f };
const float PATCH_RADIUS_MAX_3D[MAX_KeyPatch] = { 0.03f, 0.06f, 0.09f,
                                                  0.03f, 0.06f, 0.09f,
                                                  0.06f, 0.03f, 0.09f };
                                                  //!!!0.06f, 0.06f };

/*
 * @brief Updates the neighbors list be adding valid and similar neighbors
 *        of the current point
 */
void UpdateNeighbors(const Frame& reg_frame, int idx, int ref_x, int ref_y, float radius_max,
                    std::vector<bool>& visited, std::queue<int>& patch_pixels_idx) {

    // Prepare buffers
    const int width = static_cast<int>(reg_frame.width);
    const int height = static_cast<int>(reg_frame.height);
    const std::vector<bool>& valid = reg_frame.mask;
    const std::vector<Pixel_RGBDT>& pixels = reg_frame.data;
    const std::vector<Pixel_XYZ>& cloud = reg_frame.data_3D;
    const Pixel_RGBDT ref_pixel = pixels[idx];
    const float sq_radius_max = radius_max * radius_max;

    // Select distance metric
    bool use_cloud = true;
    if (cloud.empty()) {
        use_cloud = false;
    }

    // Lookup tables
    const int nb_neighbors = 8;
    //const int offset_x[] = {    -1,      0,     1, -1, 1,    -1,     0,     1};
    //const int offset_y[] = {-width, -width,-width,  0, 0, width, width, width};
    const int offset_xy[] = { -width - 1, -width, -width + 1,
                                     - 1,                  1,
                               width - 1,  width,  width + 1};

    // Test neighbors
    for (int n = 0; n < nb_neighbors; ++n) {

        // Compute neighbor index
        int new_idx = idx + offset_xy[n];
        if (new_idx >= 0 && new_idx < pixels.size() &&
            !visited[new_idx] && valid[new_idx]) {

            // Check if within the patch bounds 
            if (use_cloud) { // vheck distance in 3D space
                int ref_idx = ref_x + width * ref_y;
                Pixel_XYZ pt1 = cloud[ref_idx];
                Pixel_XYZ pt2 = cloud[new_idx];
                double distance = (pt1.x - pt2.x)*(pt1.x - pt2.x) + 
                    (pt1.y - pt2.y)*(pt1.y - pt2.y) + (pt1.z - pt2.z)*(pt1.z - pt2.z);
                if (distance > sq_radius_max) {
                    continue;
                }
            }
            else { // check distance in 2D space
                if (std::abs(ref_x - (new_idx%width)) > 20 ||
                    std::abs(ref_y - (new_idx / width)) > 20) {
                    continue;
                }
            }

            // Mark as visited
            visited[new_idx] = true;

            // Check similarity
            if (std::abs(ref_pixel.temperature - pixels[new_idx].temperature) < PATCH_TEMP_MAX_DIFF && 
                std::abs(ref_pixel.depth - pixels[new_idx].depth) < PATCH_DEPTH_MAX_DIFF) {

                // Add to the queue
                patch_pixels_idx.push(new_idx);
            }
        }
    }

}

/*
 * @brief Detects a key patch area 
 */
void KeypatchArea(const Frame& reg_frame, const SkeletonJoint& joint, KeyPatch& patch) {

    // Prepare buffers
    const int width = static_cast<int>(reg_frame.width);
    const int height = static_cast<int>(reg_frame.height);
    const std::vector<bool>& valid = reg_frame.mask;
    std::vector<bool> visited(width*height);
    JointLookup jlk;
    float max_radius = PATCH_RADIUS_MAX_3D[jlk.joint_to_patch_lookup[joint.type]];
    if (reg_frame.data_3D.empty()) {
        max_radius = PATCH_RADIUS_MAX_2D[jlk.joint_to_patch_lookup[joint.type]];
    }

    // Make sure point within the image boundaries and valid
    if (joint.x >= 0 && joint.x < width && joint.y >= 0 && joint.y < height &&
        joint.is_valid && valid[joint.x + joint.y*width]) {

        // Special case: Left wrist
        int idx = joint.x + joint.y*width;
        if (!patch.pt_x.empty()) {
            idx = (joint.x + patch.pt_x[0])/2 + (joint.y + patch.pt_y[0])/2 * width;
            patch.pt_x.clear();
            patch.pt_y.clear();
        }
        
        // Init queue
        std::queue<int> patch_pixels_idx;
        patch_pixels_idx.push(idx);
        visited[idx] = true;
        int ref_x = idx % width;
        int ref_y = idx / width;

        // Run patch area detect
        while (!patch_pixels_idx.empty()) {

            // Take point out
            idx = patch_pixels_idx.front();
            patch_pixels_idx.pop();
           
            // Save patch point
            patch.pt_x.push_back(idx % width);
            patch.pt_y.push_back(idx / width);

            // Check neighbours and update queue
            UpdateNeighbors(reg_frame, idx, ref_x, ref_y, max_radius, visited, patch_pixels_idx);
        }
    }
}


/*
* @brief Detects a key patch area by image segmentation
*/
void UpdateClusterCenter(const std::vector<Pixel_RGBDT>& pixels, const std::vector<int>& k_idx,
                         double& k_color, double k_temp, double k_depth) {
    k_color = 0.0;
    k_temp = 0.0;
    k_depth = 0.0;
    if (k_idx.size() == 0) {
        return;
    }

    for (auto idx : k_idx) {
        k_color += (pixels[idx].r + pixels[idx].g + pixels[idx].b) / 3.0;
        k_temp += pixels[idx].temperature;
        k_depth += pixels[idx].depth;
    }
    k_color /= k_idx.size();
    k_temp /= k_idx.size();
    k_depth /= k_idx.size();
}

void ComputeClusterCenterByAvg(const Frame& reg_frame,
                               const cv::Point2i& center,
                               const int window_width,
                               const int window_height,
                               std::vector<double>& k_color,
                               std::vector<double>& k_temp,
                               std::vector<double>& k_depth) {

    // Clean clusters
    for (size_t i = 0; i < k_color.size(); ++i) {
        k_color[i] = 0.0;
        k_temp[i] = 0.0;
        k_depth[i] = 0.0;
    }

    // Prepare buffers
    const int width = static_cast<int>(reg_frame.width);
    const int height = static_cast<int>(reg_frame.height);
    const std::vector<bool>& valid = reg_frame.mask;
    const std::vector<Pixel_RGBDT>& pixels = reg_frame.data;

    // Compute center
    //double avg_color = 0;
    double avg_temp = 0;
    //double avg_depth = 0;
    size_t count = 0;
    for (int row = center.y - window_height; row < center.y + window_height; ++row) {
        for (int col = center.x - window_width; col < center.x + window_width; ++col) {

            // check for image boundaries
            if (row < 0 || row >= height || col < 0 || col >= width) {
                continue;
            }

            // Check for valid index
            int idx = row * width + col;
            if (valid[idx]) {
                //avg_color += (pixels[idx].r + pixels[idx].g + pixels[idx].b) / 3.0;
                avg_temp += pixels[idx].temperature;
                //avg_depth += pixels[idx].depth;
                ++count;
            }
        }
    }
    if (count == 0) {
        return;
    }

    //const std::vector<double> offset_color = {-50, 50};
    const std::vector<double> offset_temp = {3, -3, 7, -7 };
    //const std::vector<double> offset_depth = { 100, 100 };
    for (size_t i = 0; i < k_color.size(); ++i) {
        //k_color[i] = avg_color / count - offset_color[i];
        k_temp[i] = avg_temp / count - offset_temp[i];
        //k_depth[i] = avg_depth / count - offset_depth[i];
    }
}

double ComputeClusterTemp(size_t idx, const std::vector<int>& k_idx, const std::vector<Pixel_RGBDT>& pixels) {
    if (k_idx.size() == 0) return 0.0;

    double temperature = 0.0;
    for (auto i : k_idx) {
        temperature += pixels[i].temperature;
    }
    return temperature / k_idx.size();
}

double CLAMP_TO(double x, double T) { return x > T ? T : x; }
void HandKeypatchAreaBySegmentation(const Frame& reg_frame, const cv::Point2i& center, KeyPatch& patch) {

    // Prepare buffers
    const int width = static_cast<int>(reg_frame.width);
    const int height = static_cast<int>(reg_frame.height);
    const std::vector<bool>& valid = reg_frame.mask;
    const std::vector<Pixel_RGBDT>& pixels = reg_frame.data;
    const std::vector<Pixel_XYZ>& cloud = reg_frame.data_3D;

    ///////////////////////////////////////////////////////////////////////////
    const int window_width = 20;
    const int window_height = 20;
    const int k = 2;
    size_t iter = 15;
    double k_dist_threshold = 0.6;
    std::vector<double> k_color(k);
    std::vector<double> k_temp(k);
    std::vector<double> k_depth(k);
    ComputeClusterCenterByAvg(reg_frame, center, window_width, window_height, k_color, k_temp, k_depth);
    //ComputeClusterCenterByHistograms(reg_frame, center, window_width, window_height, k_color, k_temp, k_depth);
    ///////////////////////////////////////////////////////////////////////////
    const double max_color_diff = 50.0;
    const double max_temp_diff = 10.0;
    const double max_depth_diff = 200.0;
    const double w_color = 0.3 * (1.0 / max_color_diff);
    const double w_temp = 0.5 * (1.0 / max_temp_diff);
    const double w_depth = 0.2 * (1.0 / max_depth_diff);
    std::vector<std::vector<int>> k_idx(k);
    ///////////////////////////////////////////////////////////////////////////

    for (size_t i = 0; i < iter; ++i) {

        // Clean cluster indexes
        for (auto& vec : k_idx) {
            vec.clear();
        }
        
        // Run clustering
        for (int row = center.y - window_height; row < center.y + window_height; ++row) {
            for (int col = center.x - window_width; col < center.x + window_width; ++col) {

                // check for image boundaries
                if (row < 0 || row >= height || col < 0 || col >= width) {
                    continue;
                }

                // Check for valid index
                int idx = row * width + col;
                if (valid[idx]) {

                    int min_k = -1;
                    double min_dist = 1.0;
                    for (size_t k_i = 0; k_i < k; ++k_i) {
                        double k_dist = w_color * CLAMP_TO(std::abs(k_color[k_i] - (pixels[idx].r + pixels[idx].g + pixels[idx].b) / 3.0), max_color_diff) +
                            w_temp * CLAMP_TO(std::abs(k_temp[k_i] - pixels[idx].temperature), max_temp_diff) +
                            w_depth * CLAMP_TO(std::abs(k_depth[k_i] - pixels[idx].depth), max_depth_diff);
                        
                        if (k_dist < min_dist) {
                            min_dist = k_dist;
                            min_k = k_i;
                        }
                    }

                    if (min_dist < k_dist_threshold) {
                        k_idx[min_k].push_back(idx);
                    }
                }
            }
        }

        // Update clusters
        for (size_t k_i = 0; k_i < k; ++k_i) {
            UpdateClusterCenter(pixels, k_idx[k_i], k_color[k_i], k_temp[k_i], k_depth[k_i]);
        }
    }

    //size_t max_idx = std::distance(k_temp.begin(), std::max_element(k_temp.begin(), k_temp.end()));
    size_t id_s_1 = 0;
    size_t max_size_1 = k_idx[id_s_1].size();
    size_t id_s_2 = 0;
    size_t max_size_2 = 0;
    for (size_t k_i = 1; k_i < k; ++k_i) {
        if (k_idx[k_i].size() > max_size_1) {
            max_size_2 = max_size_1;
            id_s_2 = id_s_1;
            max_size_1 = k_idx[k_i].size();
            id_s_1 = k_i;
        }
        else if (k_idx[k_i].size() > max_size_2) {
            max_size_2 = k_idx[k_i].size();
            id_s_2 = k_i;
        }
    }

    size_t id_t_1 = 0;
    double max_temp_1 = ComputeClusterTemp(id_t_1, k_idx[id_t_1], pixels);
    size_t id_t_2 = 0;
    double max_temp_2 = 0;
    for (size_t k_i = 1; k_i < k; ++k_i) {
        double temp = ComputeClusterTemp(k_i, k_idx[k_i], pixels);
        if (temp > max_temp_1) {
            max_temp_2 = max_temp_1;
            id_t_2 = id_t_1;
            max_temp_1 = temp;
            id_t_1 = k_i;
        }
        else if (temp > max_temp_2) {
            max_temp_2 = temp;
            id_t_2 = k_i;
        }
    }
    size_t idx = id_s_1;
    if (id_t_1 == id_s_1) {
        idx = id_t_1;
    }
    else if (id_t_1 == id_s_2) {
        idx = id_t_1;
    }
    else if (id_s_1 == id_t_2) {
        idx = id_s_1;
    }
    for (auto idx : k_idx[idx]) {
        patch.pt_x.push_back(idx % width);
        patch.pt_y.push_back(idx / width);
    }
}

void HandKeypatchArea(const Frame& reg_frame, const SkeletonJoint joints[JointType_Count],
                      BodyThermalModel& body_thermal_model, KeyPatch_Type type) {

    // Detect start location
    JointLookup joint_lookup;
    cv::Point2i kp_pos(0,0);
    size_t size = 0;
    for (size_t i = 0; i < JointType_Count; ++i) {

        // check joint type
        int joint_to_patch_idx = joint_lookup.joint_to_patch_lookup[joints[i].type];
        if (joints[i].is_valid && joint_to_patch_idx == type) {
            ++size;
            kp_pos.x += joints[i].x;
            kp_pos.y += joints[i].y;
        }
    }

    // Detect patch
    if (size > 0) {
        kp_pos.x /= size;
        kp_pos.y /= size;
        HandKeypatchAreaBySegmentation(reg_frame, kp_pos, body_thermal_model.patches[type]);
    }

    // Check keypatch validity
    if (body_thermal_model.patches[type].pt_x.size() == 0) {
        body_thermal_model.patches[type].is_valid = false;
    }
    else {
        body_thermal_model.patches[type].is_valid = true;
    }
}

/**
 * @brief Detects the keypatches of the human body in the current frame
 *        given the detected skeleton
 */
bool KeypatchDetect(const Frame& reg_frame, const BodySkeleton& body_skeleton,
                    BodyThermalModel& body_thermal_model) {

    // Check if the body was tracked
    if (!body_skeleton.is_tracked) {
        return false;
    }

    // Transfer body id
    body_thermal_model.id = body_skeleton.id;

    // Treat differently the hand keypatch
    HandKeypatchArea(reg_frame, body_skeleton.joints, body_thermal_model, KP_LeftHand);
    HandKeypatchArea(reg_frame, body_skeleton.joints, body_thermal_model, KP_RightHand);


    // Parse joints
    JointLookup joint_lookup;
    for (const auto& joint : body_skeleton.joints) {

        // check joint type
        int joint_to_patch_idx = joint_lookup.joint_to_patch_lookup[joint.type];
        if (joint_to_patch_idx < 0 || joint_to_patch_idx == KP_LeftHand || joint_to_patch_idx == KP_RightHand) {
            continue;
        }
        
        // Check joint validity
        if (joint.is_valid) {
            KeypatchArea(reg_frame, joint, body_thermal_model.patches[joint_to_patch_idx]);
            body_thermal_model.patches[joint_to_patch_idx].is_valid = true;
            //printf("Skeleton %d - Joint %d - New joint detected with size %d \n", body_skeleton.id,
            //    joint.type, body_thermal_model.patches[joint_to_patch_idx].pt_x.size());
        }
        else {
            body_thermal_model.patches[joint_to_patch_idx].is_valid = false;
        }

        // check if patch within the image
        if (body_thermal_model.patches[joint_to_patch_idx].pt_x.size() == 0) {
            body_thermal_model.patches[joint_to_patch_idx].is_valid = false;
        }
    }
    
    // everything is fine :)
    return true;
}

/**
 * @brief Computes the themal profile of each patch of the body
 */
void KeypatchThermalModel(const Frame& reg_frame, BodyThermalModel& body_model) {

    // Prepare buffers
    const int width = static_cast<int>(reg_frame.width);
    const int height = static_cast<int>(reg_frame.height);
    const std::vector<Pixel_RGBDT>& pixels = reg_frame.data;
    const std::vector<Pixel_XYZ>& xyz = reg_frame.data_3D;
    
    // Parse body patches
    for (auto& kp : body_model.patches) {

        // Reset metadata
        kp.temperature_avg = 0.0;
        kp.temperature_variance = 0.0;
        kp.x = .0f;
        kp.y = .0f;
        kp.z = .0f;
        kp.z_avg = 0.0;

        if (kp.is_valid && kp.pt_x.size() > 0) {
            
            // Use first point of each patch to define the (x,y) location into the world
            // The first point should be always the kinect detected joint point
            kp.x = xyz[kp.pt_x[0] + width * kp.pt_y[0]].x;
            kp.y = xyz[kp.pt_x[0] + width * kp.pt_y[0]].y;
            kp.z = xyz[kp.pt_x[0] + width * kp.pt_y[0]].z;

            for (size_t i = 0; i < kp.pt_x.size(); ++i) {
                size_t idx = kp.pt_x[i] + width * kp.pt_y[i];
                kp.temperature_avg += pixels[idx].temperature;
                kp.z_avg += pixels[idx].depth;
            }
            kp.temperature_avg /= kp.pt_x.size();
            kp.z_avg /= kp.pt_x.size();

            for (size_t i = 0; i < kp.pt_x.size(); ++i) {
                size_t idx = kp.pt_x[i] + width * kp.pt_y[i];
                kp.temperature_variance += (pixels[idx].temperature - kp.temperature_avg) *
                    (pixels[idx].temperature - kp.temperature_avg);
            }
            kp.temperature_variance /= kp.pt_x.size();
        }
    }
}

/**
 * @brief Computes the themal model of all bodies detected in the frame
 */
void ComputeBodyThermalModel(const Frame& reg_frame, const RawFrame& raw_frame,
                             std::vector<BodyThermalModel>& bodies) {

    // Parse bodies
    for (const auto& body_skeleton : raw_frame.skeletons) {
        
        BodyThermalModel body_model;
        if (KeypatchDetect(reg_frame, body_skeleton, body_model)) {
            KeypatchThermalModel(reg_frame, body_model);
            bodies.push_back(body_model);
        }
    }
}