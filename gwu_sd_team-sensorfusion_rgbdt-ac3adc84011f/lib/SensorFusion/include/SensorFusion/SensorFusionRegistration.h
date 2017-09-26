//============================================================================
// Name        : SensorFusionRegistration.h
// Author      : CosmaC
// Date        : October, 2016
// Copyright   : GWU Research
// Description : Sensor fusion stream registration
//============================================================================

#pragma once

#include "3DSensor/3DSensor.h"
#include "ThermalSensor/ThermalSensor.h"

// OpenCV
#include <opencv/cv.h>
#include <opencv/cxcore.h>

#define PI 3.1415926535897932384626433832795

// System calibration structure
struct GeometryModel {
    double h[9];
    int deltaDisp;
    float invCoeff;
    int width_c;
    int height_c;
    int width_t;
    int height_t;
    int scale_t;
    int width_d;
    int height_d;
};

class GeometryCalibration {

public:
    // Constructor
    GeometryCalibration() {
        calib_.invCoeff = 1;
        calib_.deltaDisp = 0;
        calib_.width_c = 0;
        calib_.height_c = 0;
        calib_.width_t = 0;
        calib_.height_t = 0;
        calib_.width_d = 0;
        calib_.height_d = 0;
        memset(calib_.h, 0, sizeof(calib_.h));
        calib_.h[0] = 1.0;
        calib_.h[4] = 1.0;
        calib_.h[8] = 1.0;
    }

    // Do depth sensor to thermal imager calibration
    bool Calibration(ThermalSensor& thermal_sensor, Sensor3D& depth_sensor);

    // Load depth sensor to thermal imager calibration
    bool LoadCalibration(ThermalSensorType sensor_type);

    // Get/Set calibration model
    inline bool getCalibration(GeometryModel& calibration) {
        // Checks if calibration was properly loaded or excuted, else return false
        if (calib_.width_c == 0 || calib_.height_c == 0 ||
            calib_.width_d == 0 || calib_.height_d == 0 ||
            calib_.width_t == 0 || calib_.height_t == 0 ||
            calib_.scale_t == 0) {
            return false;
        }

        calibration = calib_;
        return true;
    };

    inline void setCalibration(const GeometryModel& calibration) {
        calib_ = calibration;
        RecomputeLookupTables();
    };

    // Get thermal location of a point in RGB image
    inline void getThermalPoint(int x_c, int y_c, float& x_t, float& y_t) {
        x_t = lookup_table_x[x_c + y_c * calib_.width_c];
        y_t = lookup_table_y[x_c + y_c * calib_.width_c];
    }

private:
    // Calibration 
    GeometryModel calib_;
    std::vector<float> lookup_table_x;
    std::vector<float> lookup_table_y;

    // TODO: Move this to 3D sensor class
    const float kBaseline = 0.026f; // 26mm baseline between thermal and depth sensor

    // Calibration pattern params
    const int kNx = 6;
    const int kNy = 6;
    const int k_thermal_pixel_threshold_ = 180;     // Thermal image pixel intensity threshol
                                                    // RGB image pattern detection
    const int k_thermal_scale_up_ = 8;              // Up-scale for the thermal image
    const int k_edge_magnitude_threshold_ = 200;    // Edge magnitude tresholding
    const int k_kernel_half_size_x_ = 70;           // No-edge ROI width size
    const int k_kernel_half_size_y_ = 200;          // No-edge ROI height size

    /**
     * @brief Use the calibration structure to compute the warping meshes
     *        for fast calibration
     */
    void RecomputeLookupTables();

    /**
     * @brief T-RGB calibration based on a vertical stereo system model.
     *        Computes the geometrical relation between the two sensors, given a set
     *        point correspondences between the two images.
     *
     * @param[in]  correspondences      Set of point correspondences between the 2 image planes
     * @param[in]  depth_XYZ            Current frame depth image
     * @param[in]  disp_2_depth_coeff   Disparity to depth conversion function coefficient
     * @param[out] homography           Geometrical relation between the 2 image planes (persepctive transform)
     * @param[out] delta_disp           Disparity offset, computed for the depth of
     *                                  the calibration pattern, during the calibration stage
     * @param[in]  thermal_img          Thermmal image [just for debugging]
     * @param[in]  bgr_img              Color image [just for debugging]
     * @param[in]  show_undistorted     Debugging tools: activate debugging
     * @param[in]  is_vertical_stereo   Camera system configuration
     *
     * @returns Calibration success or failure. In the case of a failure,
     *          the calibration must be re-run for a new frame.
     */
    HRESULT stereoCalib(std::vector<CvPoint2D32f> correspondences[2],
                        const CameraSpacePoint* depth_XYZ,
                        float disp_2_depth_coeff,
                        cv::Mat& homography,
                        int &delta_disp,
                        const cv::Mat &thermal_img, const cv::Mat &bgr_img,
                        int show_undistorted, bool is_vertical_stereo);

    /**
     * @brief Chessboard pattern corner detector for thermal and color images
     *
     * @param[in]  img              Given image (Color or thermal).
     * @param[in]  nx               Number of corners in a column of the pattern.
     * @param[in]  ny               Number of corners in a row of the pattern.
     * @param[out] points           Pattern detected corners.
     * @param[in]  ir_flag          If on(true) uses corner detector for thermal image.
     *                              Otherwise detects corners in a color image.
     * @param[in]  display_corners  Debugging tools: activate debugging
     *
     * @returns Corner detection success or failure. In the case of a failure,
     *          the corners of the pattern were not proprely detecte, and the
     *          detection must be re-run for a new frame.
     */
    HRESULT stereoCornersDetect(const cv::Mat &img,
                                int nx, int ny,
                                std::vector<CvPoint2D32f>& points,
                                int ir_flag, int display_corners);

    /**
     * @brief Computes chessboard corners for the color(RGB) image
     *
     * @param[in]  img          Given color image.
     * @param[in]  nx           Number of corners in a column of the pattern.
     * @param[in]  ny           Number of corners in a row of the pattern.
     * @param[out] corners      Detected pattern corners (these are appended to the array)
     * @param[in]  corners_nb   Number of detected corners in the current image.
     * @param[in]  debug        Debugging tools: activate debugging.
     *
     * @retuns  1 on success, and 0 if unable to find all corners
     */
    int RGBCornersDetect(const cv::Mat &img,
                         int nx, int ny,
                         CvPoint2D32f* corners,
                         int *corners_nb,
                         int debug = 0);

    /**
     * @brief Computes chessboard corners for the thermal image
     *
     * @param[in]  img          Given thermal image.
     * @param[in]  nx           Number of corners in a column of the pattern.
     * @param[in]  ny           Number of corners in a row of the pattern.
     * @param[out] corners      Detected pattern corners (these are appended to the array)
     * @param[in]  corners_nb   Number of detected corners in the current image.
     * @param[in]  debug        Debugging tools: activate debugging.
     *
     * @retuns  1 on success, and 0 if unable to find all corners
     */
    int IRCornersDetect(const cv::Mat &img,
                        int nx, int ny,
                        CvPoint2D32f* corners,
                        int *corners_nb,
                        int debug = 0);
};
