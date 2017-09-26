//============================================================================
// Name        : KeyPatch.h
// Author      : CosmaC
// Date        : November, 2016
// Copyright   : GWU Research
// Description : KeyPatches detector based on fused frame and skeleton points
//============================================================================

#pragma once

#include "SensorFusion/SensorFusionSync.h"

#include <Kinect.h>

// Body patches types definition
#define MAX_KeyPatch 9 //!!!
enum KeyPatch_Type {KP_LeftHand, KP_LeftArm, KP_LeftShoulder, 
                    KP_RightHand, KP_RightArm, KP_RightShoulder,
                    KP_Head, KP_Neck, KP_Torso};
                    //!!! KP_LeftLeg, KP_RightLeg};

// Body patch definition
struct KeyPatch {
    bool is_valid;
    std::vector<int> pt_x;
    std::vector<int> pt_y;
    float x;
    float y;
    float z;
    double z_avg;
    double temperature_avg;
    double temperature_variance;
};

// Keypatch coloring scheme
const unsigned char keypatch_color_table[3 * MAX_KeyPatch] = {
    255, 0, 0,
    255, 100, 0,
    255, 180, 0,
    0, 0, 255,
    0, 100, 255,
    0, 180, 255,
    0, 100, 0,
    100, 180, 100,
    0, 255, 0
//!!!   255, 200, 50,
//!!!    50, 200, 255
};

// Body model definition
struct BodyThermalModel {
    unsigned int id;
    std::array<KeyPatch, MAX_KeyPatch> patches;
    double thermal_comfort;
    double skin_temperature;
    double clothing_temperature;
};

struct ThermalModelLog {
    double timestamp;
    unsigned int id;
    int sensor_temperature;
    float x[MAX_KeyPatch];
    float y[MAX_KeyPatch];
    float z[MAX_KeyPatch];
    float z_avg[MAX_KeyPatch];
    float temperature_avg[MAX_KeyPatch];
    float temperature_variance[MAX_KeyPatch];

    ThermalModelLog(const BodyThermalModel& body) {
        id = body.id;
        for (size_t i = 0; i < MAX_KeyPatch; ++i) {
            if (body.patches[i].is_valid) {
                x[i] = body.patches[i].x;
                y[i] = body.patches[i].y;
                z[i] = body.patches[i].z;
                z_avg[i] = static_cast<float>(body.patches[i].z_avg);
                temperature_avg[i] = 
                    static_cast<float>(body.patches[i].temperature_avg);
                temperature_variance[i] = body.patches[i].temperature_variance;
            }
            else {
                z_avg[i] = 0.f;
                temperature_avg[i] = 0.f;
                temperature_variance[i] = 0.f;
            }
        }
    };
};

// Lookup table for patches to joints conversion
struct JointLookup {
    int joint_to_patch_lookup[JointType_Count];
    int patch_to_joint_lookup[MAX_KeyPatch];
        
    JointLookup() {

        joint_to_patch_lookup[JointType_SpineBase] = -1;
        joint_to_patch_lookup[JointType_SpineMid] = KP_Torso;
        joint_to_patch_lookup[JointType_Neck] = KP_Neck;
        joint_to_patch_lookup[JointType_Head] = KP_Head;
        joint_to_patch_lookup[JointType_ShoulderLeft] = KP_LeftShoulder;
        joint_to_patch_lookup[JointType_ElbowLeft] = KP_LeftArm;
        joint_to_patch_lookup[JointType_WristLeft] = KP_LeftHand;
        joint_to_patch_lookup[JointType_HandLeft] = KP_LeftHand;
        joint_to_patch_lookup[JointType_ShoulderRight] = KP_RightShoulder;
        joint_to_patch_lookup[JointType_ElbowRight] = KP_RightArm;
        joint_to_patch_lookup[JointType_WristRight] = KP_RightHand;
        joint_to_patch_lookup[JointType_HandRight] = KP_RightHand;
        joint_to_patch_lookup[JointType_HipLeft] = -1;
        joint_to_patch_lookup[JointType_KneeLeft] = -1; //!!! KP_LeftLeg;
        joint_to_patch_lookup[JointType_AnkleLeft] = -1;
        joint_to_patch_lookup[JointType_FootLeft] = -1;
        joint_to_patch_lookup[JointType_HipRight] = -1;
        joint_to_patch_lookup[JointType_KneeRight] = -1; //!!! KP_RightLeg;
        joint_to_patch_lookup[JointType_AnkleRight] = -1;
        joint_to_patch_lookup[JointType_FootRight] = -1;
        joint_to_patch_lookup[JointType_SpineShoulder] = -1;
        joint_to_patch_lookup[JointType_HandTipLeft] = -1;
        joint_to_patch_lookup[JointType_ThumbLeft] = -1;
        joint_to_patch_lookup[JointType_HandTipRight] = -1;
        joint_to_patch_lookup[JointType_ThumbRight] = -1;

        patch_to_joint_lookup[KP_LeftHand] = JointType_HandLeft;
        patch_to_joint_lookup[KP_LeftArm] = JointType_ElbowLeft;
        patch_to_joint_lookup[KP_LeftShoulder] = JointType_ShoulderLeft;
        patch_to_joint_lookup[KP_RightHand] = JointType_HandRight;
        patch_to_joint_lookup[KP_RightArm] = JointType_ElbowRight;
        patch_to_joint_lookup[KP_RightShoulder] = JointType_ShoulderRight;
        patch_to_joint_lookup[KP_Head] = JointType_Head;
        patch_to_joint_lookup[KP_Neck] = JointType_Neck;
        patch_to_joint_lookup[KP_Torso] = JointType_SpineMid;
        //!!! patch_to_joint_lookup[KP_LeftLeg] = JointType_KneeLeft;
        //!!! patch_to_joint_lookup[KP_RightLeg] = JointType_KneeRight;
    }
};


/**
 * @brief Detects the keypatches of the human body in the current frame
 *        given the detected skeleton
 */
bool KeypatchDetect(const Frame& reg_frame, const BodySkeleton& skeleton,
                    BodyThermalModel& body_thermal_model);

/**
 * @brief Computes the themal model of all bodies detected in the frame
 */
void ComputeBodyThermalModel(const Frame& reg_frame, const RawFrame& raw_frame,
                             std::vector<BodyThermalModel>& bodies);
