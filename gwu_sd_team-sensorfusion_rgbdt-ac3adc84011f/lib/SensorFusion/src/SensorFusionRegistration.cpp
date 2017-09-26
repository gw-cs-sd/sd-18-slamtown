//============================================================================
// Name        : SensorFusionRegistration.h
// Author      : CosmaC
// Date        : October, 2016
// Copyright   : GWU Research
// Description : Sensor fusion stream registration
//============================================================================

#include "SensorFusion/SensorFusionRegistration.h"

// OpenCV
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

// C/C++
#include <iostream>
#include <cstdlib>
#include <fstream>


/**
 * @brief Compares two given points based on their X coordiante
 *
 * @param[in] p1  First given 2D point
 * @param[in] p2  Second given 2D point
 *
 * @returns true if first point is bigger than the second one; false otherwise.
 */
bool sortX(const cv::Point2f &p1, const cv::Point2f &p2) {
    return p1.x > p2.x;
}


/**
 * @brief Compares two given points based on their Y coordiante
 *
 * @param[in] p1  First given 2D point
 * @param[in] p2  Second given 2D point
 *
 * @returns true if first point is bigger than the second one; false otherwise.
 */
bool sortY(const cv::Point2f &p1, const cv::Point2f &p2) {
    return p1.y > p2.y;
}


/**
 * @brief Computes chessboard corners for the thermal image
 */
int GeometryCalibration::IRCornersDetect(const cv::Mat &img,
                    int nx, int ny,
                    CvPoint2D32f* corners,
                    int *corners_nb,
                    int debug) {

    // IR img size and border
    const int width = img.cols;
    const int height = img.rows;
    const int step = img.step;
    const int x_border = 10;
    const int y_border = 10;
    unsigned char* img_data = (unsigned char*)img.data;

    // Tresholding
    cv::Mat out = cv::Mat::zeros(height, width, CV_8UC1);
    unsigned char* out_data = out.data;
    for (int i = y_border; i < height - y_border; ++i) {
        for (int j = x_border; j < width - x_border; ++j) {
            if (img_data[i * step + j] > k_thermal_pixel_threshold_)
                out_data[i * step + j] = 255;
        }
    }

    // Dilate and erode
    const int dilation_size = 2;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
        cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
        cv::Point(dilation_size, dilation_size));
    cv::dilate(out, out, element);
    cv::erode(out, out, element);

    // Find blobs
    int count = 0;
    for (int i = y_border; i < height - y_border; ++i) {
        for (int j = x_border; j < width - x_border; ++j) {
            if (out_data[i * step + j] == 255)
                count++;
        }
    }

    // Tests if enough points detected, and not too many
    if (count < nx*ny) {    
        std::cerr << "[Warning][Calibration::IRCornersDetect] Not enough points."
            << std::endl;
        out.release();
        element.release();
        return 0;
    }
    else if (count > height*width*0.2) {
        std::cerr << "[Warning][Calibration::IRCornersDetect] Too many points."
            << std::endl;
        out.release();
        element.release();
        return 0;
    }

    // Prepare for clustering
    cv::Mat points = cv::Mat(count, 2, CV_32F);
    count = 0;
    for (int i = y_border; i < height - y_border; ++i) {
        for (int j = x_border; j < width - x_border; ++j) {
            if (out_data[i * step + j] == 255) {
                points.at<float>(count, 0) = (float)i;
                points.at<float>(count, 1) = (float)j;
                count++;
            }
        }
    }

    // Run clustering
    cv::Mat labels;
    cv::Mat centers_ir;
    cv::kmeans(points, nx*ny, labels,
        cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, 0.01),
        5, cv::KMEANS_PP_CENTERS, centers_ir);

    // Debug results
    if (1 == debug) {
        cv::Mat out_2 = cv::Mat::zeros(out.rows, out.cols, CV_8UC3);
        unsigned char* o_data = out_2.data;

        // Draw blobs
        for (int i = 0; i < points.rows; ++i) {
            o_data[(int)(points.at<float>(i, 0) * out_2.step +
                3 * points.at<float>(i, 1) + 2)] = 255;
        }
        // Draw centers
        for (int i = 0; i < centers_ir.rows; ++i) {
            o_data[(int)(centers_ir.at<float>(i, 0) + 0.5) * out_2.step +
                3 * (int)(centers_ir.at<float>(i, 1) + 0.5)] = 255;
            o_data[(int)(centers_ir.at<float>(i, 0) + 0.5) * out_2.step +
                3 * (int)(centers_ir.at<float>(i, 1) + 0.5) + 1] = 255;
            o_data[(int)(centers_ir.at<float>(i, 0) + 0.5) * out_2.step +
                3 * (int)(centers_ir.at<float>(i, 1) + 0.5) + 2] = 255;
        }
        // Display
        cv::imshow("IR Centers", out_2);
        out_2.release();
    }

    // Save corners
    *corners_nb = centers_ir.rows;
    for (int i = 0; i < centers_ir.rows; i++) {
        corners[i].x = centers_ir.at<float>(i, 1);
        corners[i].y = centers_ir.at<float>(i, 0);;
    }

    // Clean
    out.release();
    element.release();
    points.release();
    labels.release();
    centers_ir.release();

    // Everything succeed
    return 1;
}


/**
 * @brief Computes chessboard corners for the color(RGB) image
 */
int GeometryCalibration::RGBCornersDetect(const cv::Mat &img,
                     int nx, int ny,
                     CvPoint2D32f* corners,
                     int *corners_nb,
                     int debug) {

    // Pre-processing
    const int width = img.cols;
    const int height = img.rows;
    cv::Mat img_r;
    cv::GaussianBlur(img, img_r, cv::Size(9, 9), 0, 0);

    // Detect edges
    cv::Mat img_t;
    cv::Canny(img_r, img_t, 20, 20 * 3, 3);
    if (1 == debug) cv::imshow("Edges", img_t);

    // Compute integral image
    cv::threshold(img_t, img_t, k_edge_magnitude_threshold_, 1, cv::THRESH_BINARY);
    cv::Mat integral;
    cv::integral(img_t, integral);
    img_t.release();

    // Find ROI
    int x_min = integral.cols;
    int x_max = 0;
    int y_min = integral.rows;
    int y_max = 0;
    //for (int i = k_kernel_half_size_y_; i < integral.rows - k_kernel_half_size_y_; ++i) {
    //    for (int j = k_kernel_half_size_x_; j < integral.cols - k_kernel_half_size_x_; ++j) {
    //        int tl = integral.at<int>(i - k_kernel_half_size_y_, j - k_kernel_half_size_x_);
    //        int tr = integral.at<int>(i - k_kernel_half_size_y_, j + k_kernel_half_size_x_);
    //        int bl = integral.at<int>(i + k_kernel_half_size_y_, j - k_kernel_half_size_x_);
    //        int br = integral.at<int>(i + k_kernel_half_size_y_, j + k_kernel_half_size_x_);
    //        int regionSum = br - bl - tr + tl;

    //        if (regionSum == 0) {
    //            if (i < y_min) y_min = i;
    //            if (i > y_max) y_max = i;
    //            if (j < x_min) x_min = j;
    //            if (j > x_max) x_max = j;
    //        }
    //    }
    //}
    //x_min -= k_kernel_half_size_x_ / 2;
    //y_min -= k_kernel_half_size_y_ / 2;
    //x_max += k_kernel_half_size_x_ / 2;
    //y_max += k_kernel_half_size_y_ / 2;

    // Up limit
    int band_width = 100;
    int band_height = 30;
    y_min = integral.rows / 2;
    int x = integral.cols / 2;
    while (y_min != 0) {
        int tl = integral.at<int>(y_min - band_height, x - band_width);
        int tr = integral.at<int>(y_min - band_height, x + band_width);
        int bl = integral.at<int>(y_min + band_height, x - band_width);
        int br = integral.at<int>(y_min + band_height, x + band_width);
        int regionSum = br - bl - tr + tl;

        if (regionSum > 200) {
            break;
        }
        --y_min;
    }
    y_max = integral.rows / 2;
    while (y_max < integral.rows - 1) {
        int tl = integral.at<int>(y_max - band_height, x - band_width);
        int tr = integral.at<int>(y_max - band_height, x + band_width);
        int bl = integral.at<int>(y_max + band_height, x - band_width);
        int br = integral.at<int>(y_max + band_height, x + band_width);
        int regionSum = br - bl - tr + tl;

        if (regionSum > 200) {
            break;
        }
        ++y_max;
    }

    band_width = 30;
    band_height = 100;
    int y = integral.rows / 2;
    x_min = integral.cols / 2;
    while (x_min != 0) {
        int tl = integral.at<int>(y - band_height, x_min - band_width);
        int tr = integral.at<int>(y - band_height, x_min + band_width);
        int bl = integral.at<int>(y + band_height, x_min - band_width);
        int br = integral.at<int>(y + band_height, x_min + band_width);
        int regionSum = br - bl - tr + tl;

        if (regionSum > 200) {
            break;
        }
        --x_min;
    }
    x_max = integral.cols / 2;
    while (x_max < integral.cols - 1) {
        int tl = integral.at<int>(y - band_height, x_max - band_width);
        int tr = integral.at<int>(y - band_height, x_max + band_width);
        int bl = integral.at<int>(y + band_height, x_max - band_width);
        int br = integral.at<int>(y + band_height, x_max + band_width);
        int regionSum = br - bl - tr + tl;

        if (regionSum > 200) {
            break;
        }
        ++x_max;
    }

    if (1 == debug) {
        cv::Mat mask = cv::Mat::zeros(img_r.rows, img_r.cols, CV_8UC1);
        unsigned char* d_mask = mask.data;
        for (int i = k_kernel_half_size_y_; i < integral.rows - k_kernel_half_size_y_; i++) {
            for (int j = k_kernel_half_size_x_; j < integral.cols - k_kernel_half_size_x_; j++) {
                int tl = integral.at<int>(i - k_kernel_half_size_y_, j - k_kernel_half_size_x_);
                int tr = integral.at<int>(i - k_kernel_half_size_y_, j + k_kernel_half_size_x_);
                int bl = integral.at<int>(i + k_kernel_half_size_y_, j - k_kernel_half_size_x_);
                int br = integral.at<int>(i + k_kernel_half_size_y_, j + k_kernel_half_size_x_);
                int regionSum = br - bl - tr + tl;
                if (regionSum == 0)
                    d_mask[i * mask.step + j] = 255;
            }
        }
        cv::imshow("Mask", mask);
        mask.release();
    }

    // Guard condition
    if (x_min >= x_max || y_min >= y_max) {
        return 0;
    }

    // Apply 2D filter
    const int kernel_size = 9;
    float k_data[] = { 2, 2,  2,  2,  2,  2,  2, 2, 2,
        2, 2,  2,  2,  2,  2,  2, 2, 2,
        2, 2,  1,  1,  1,  1,  1, 2, 2,
        2, 2, -4, -9, -9, -9, -4, 2, 2,
        2, 2, -4,-22,-22,-22, -4, 2, 2,
        2, 2, -4, -9, -9, -9, -4, 2, 2,
        2, 2,  1,  1,  1,  1,  1, 2, 2,
        2, 2,  2,  2,  2,  2,  2, 2, 2,
        2, 2,  2,  2,  2,  2,  2, 2, 2 };
    cv::Mat kernel = cv::Mat(kernel_size, kernel_size, CV_32F, k_data);
    cv::Mat output;

    // Crop region of interest
    cv::Rect roi = cv::Rect(x_min, y_min, x_max - x_min, y_max - y_min);
    cv::Mat img_roi = img_r(roi);
    cv::Mat mask_roi = integral(roi);
    cv::filter2D(img_roi, output, -1, kernel, cv::Point(-1, -1), 0, cv::BORDER_DEFAULT);
    img_r.release();

    // Dilate and erode
    const int dilation_size = 3;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
        cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
        cv::Point(dilation_size, dilation_size));
    cv::dilate(output, output, element);
    cv::erode(output, output, element);
    element.release();

    // Clustering
    int count = 0;
    unsigned char* o_data = output.data;
    for (int i = 20; i < output.rows - 20; i++) {
        for (int j = 30; j < output.cols - 30; j++) {
            if (o_data[i * output.step + j] > 200) {
                int tl = mask_roi.at<int>(i - 20, j - 30);
                int tr = mask_roi.at<int>(i - 20, j - 20);
                int bl = mask_roi.at<int>(i + 20, j - 30);
                int br = mask_roi.at<int>(i + 20, j - 20);
                int regionSum = br - bl - tr + tl;
                tl = mask_roi.at<int>(i - 20, j + 20);
                tr = mask_roi.at<int>(i - 20, j + 30);
                bl = mask_roi.at<int>(i + 20, j + 20);
                br = mask_roi.at<int>(i + 20, j + 30);
                regionSum += br - bl - tr + tl;

                if (regionSum == 0)
                    count++;
            }
        }
    }
    cv::Mat points = cv::Mat(count, 2, CV_32F);
    count = 0;
    for (int i = 20; i < output.rows - 20; i++) {
        for (int j = 30; j < output.cols - 30; j++) {
            if (o_data[i * output.step + j] > 200) {
                int tl = mask_roi.at<int>(i - 20, j - 30);
                int tr = mask_roi.at<int>(i - 20, j - 20);
                int bl = mask_roi.at<int>(i + 20, j - 30);
                int br = mask_roi.at<int>(i + 20, j - 20);
                int regionSum = br - bl - tr + tl;
                tl = mask_roi.at<int>(i - 20, j + 20);
                tr = mask_roi.at<int>(i - 20, j + 30);
                bl = mask_roi.at<int>(i + 20, j + 20);
                br = mask_roi.at<int>(i + 20, j + 30);
                regionSum += br - bl - tr + tl;

                if (regionSum == 0) {
                    points.at<float>(count, 0) = (float)i;
                    points.at<float>(count, 1) = (float)j;
                    count++;
                }
            }
        }
    }
    integral.release();

    // Test if enough points detected, and not too many
    if ((points.rows < nx*ny) || (points.rows > img.rows*img.cols*0.1)) {
        std::cerr << "[Warning][Calibration::RGBCornersDetect] Too many points."
            << std::endl;
        return 0;
    }


    // Run clustering
    cv::Mat labels;
    cv::Mat centers_rgb;
    cv::kmeans(points, nx*ny, labels,
        cv::TermCriteria(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 1000, 0.01),
        5, cv::KMEANS_PP_CENTERS, centers_rgb);

    // Debug results
    if (1 == debug)
    {
        //printf("\n Rows %d and cols %d", output.rows, output.cols);
        cv::Mat out_2 = cv::Mat::zeros(output.rows, output.cols, CV_8UC3);
        o_data = out_2.data;
        for (int i = 0; i < points.rows; i++) {
            o_data[(int)(points.at<float>(i, 0) * out_2.step + 3 * points.at<float>(i, 1) + 2)] = 255;
        }

        for (int i = 0; i < centers_rgb.rows; i++) {
            o_data[(int)(centers_rgb.at<float>(i, 0) + 0.5) * out_2.step + 3 * (int)(centers_rgb.at<float>(i, 1) + 0.5)] = 255;
            o_data[(int)(centers_rgb.at<float>(i, 0) + 0.5) * out_2.step + 3 * (int)(centers_rgb.at<float>(i, 1) + 0.5) + 1] = 255;
            o_data[(int)(centers_rgb.at<float>(i, 0) + 0.5) * out_2.step + 3 * (int)(centers_rgb.at<float>(i, 1) + 0.5) + 2] = 255;
        }
        // Display blobs
        cv::namedWindow("RGB_Centers");
        cv::imshow("RGB_Centers", out_2);
        out_2.release();
        //printf("///Done!");
    }

    // Add ROI padding to the detected points
    for (int i = 0; i < centers_rgb.rows; ++i) {
        centers_rgb.at<float>(i, 0) += y_min;
        centers_rgb.at<float>(i, 1) += x_min;
    }

    // Save corners
    *corners_nb = centers_rgb.rows;
    for (int i = 0; i < centers_rgb.rows; ++i) {
        corners[i].x = centers_rgb.at<float>(i, 1);
        corners[i].y = centers_rgb.at<float>(i, 0);
    }

    // Clean
    output.release();
    points.release();
    labels.release();
    return 1;
}


/**
 * @brief Chessboard pattern corner detector for thermal and color images
 */
HRESULT GeometryCalibration::stereoCornersDetect(const cv::Mat &img,
                            int nx, int ny,
                            std::vector<CvPoint2D32f>& points,
                            int ir_flag, int display_corners) {

    // Number of corners
    int n = nx*ny;
    std::vector<CvPoint2D32f> temp(n);

    // Detect corners
    int count = 0;
    int result = 0;
    if (1 == ir_flag) {
        result = IRCornersDetect(img, nx, ny, &temp[0], &count, display_corners);
    }
    else {
        result = RGBCornersDetect(img, nx, ny, &temp[0], &count, display_corners);
    }

    // Checks if corect detected all corners
    if ((result != 1) || (count != n)) {
        std::cerr << "[Warning][Calibration::stereoCornersDetect] Not all corners were detected."
            << std::endl;
        return E_FAIL;
    }

    // Sort corners
    std::sort(temp.begin(), temp.end(), sortY);
    for (int i = 0; i < ny; i++) {
        std::sort(temp.begin() + i*nx, temp.begin() + (i + 1)*nx, sortX);
    }

    // Show corners
    if (display_corners) {
        IplImage* cimg = cvCreateImage(cvSize(img.cols, img.rows), 8, 3);
        CvMat img_aux = img;
        cvCvtColor(&img_aux, cimg, CV_GRAY2BGR);
        cvDrawChessboardCorners(cimg, cvSize(nx, ny), &temp[0], count, result);
        if (1 == ir_flag)
            cvShowImage("corners IR", cimg);
        else
            cvShowImage("corners RGB", cimg);
        cvReleaseImage(&cimg);
    }

    // Save detection
    int N = points.size();
    points.resize(N + n, cvPoint2D32f(0, 0));
    copy(temp.begin(), temp.end(), points.begin() + N);

    // Everything is fine :)
    return S_OK;
}


/**
 * @brief T-RGB calibration based on a vertical stereo system model.
 *        Computes the geometrical relation between the two sensors, given a set
 *        point correspondences between the two images.
 */
HRESULT GeometryCalibration::stereoCalib(std::vector<CvPoint2D32f> correspondences[2],
                    const CameraSpacePoint* depth_XYZ,
                    float disp_2_depth_coeff,
                    cv::Mat& homography,
                    int &delta_disp,
                    const cv::Mat &thermal_img, const cv::Mat &bgr_img,
                    int show_undistorted, bool is_vertical_stereo) {

    // Numbe of points
    int n = correspondences[0].size();

    // Move points in a Mat structure
    cv::Mat _imagePoints1 = cv::Mat(1, n, CV_32FC2, &correspondences[1][0]);
    cv::Mat _imagePoints2 = cv::Mat(1, n, CV_32FC2, &correspondences[0][0]);

    // Compute homography
    homography = cv::findHomography(_imagePoints1, _imagePoints2, CV_RANSAC, 1);

    // Compute deltaDisp
    int halfW = bgr_img.cols / 2;
    int halfH = bgr_img.rows / 2;
    const int window = 100;
    float d = 0;
    int count = 0;
    for (int i = halfH - window; i <= halfH + window; ++i)
        for (int j = halfW - window; j <= halfW + window; ++j) {
            CameraSpacePoint p = depth_XYZ[i * bgr_img.cols + j];
            // Check if valid depth
            if (p.Z != std::numeric_limits<float>::infinity() &&
                p.Z != -std::numeric_limits<float>::infinity()) {
                d += disp_2_depth_coeff / p.Z;
                count++;
            }
        }
    delta_disp = (int)(d / count + 0.5);

    // Show homography quality
    if (show_undistorted) {
        cv::Mat img1, img2;
        cv::resize(bgr_img, img1, cv::Size(bgr_img.cols / 4, bgr_img.rows / 4));
        cv::Mat pair;
        cv::warpPerspective(thermal_img, img2, homography, bgr_img.size(),
            CV_INTER_LINEAR + CV_WARP_FILL_OUTLIERS);
        cv::resize(img2, img2, cv::Size(img2.cols / 4, img2.rows / 4));

        if (!is_vertical_stereo) {
            pair = cv::Mat(img1.rows, img1.cols * 2, CV_8UC3);
            cv::Rect roi(0, 0, img1.cols, img1.rows);
            cv::Mat part = pair(roi);
            cv::cvtColor(img1, part, CV_GRAY2BGR);

            roi.x = img1.cols;
            part = pair(roi);
            cv::cvtColor(img2, part, CV_GRAY2BGR);

            for (int j = 0; j < img1.rows; j += 16)
                cv::line(pair, cv::Point(0, j), cv::Point(img1.cols * 2, j), CV_RGB(0, 255, 0));
        }
        else {
            pair = cv::Mat(img1.rows * 2, img1.cols, CV_8UC3);
            cv::Rect roi(0, 0, img1.cols, img1.rows);
            cv::Mat part = pair(roi);
            cv::cvtColor(img1, part, CV_GRAY2BGR);

            roi.y = img1.rows;
            part = pair(roi);
            cv::cvtColor(img2, part, CV_GRAY2BGR);
            for (int j = 0; j < img1.cols; j += 16)
                cv::line(pair, cv::Point(j, 0), cv::Point(j, img1.rows * 2), CV_RGB(0, 255, 0));
        }
        cv::imshow("rectified", pair);
        char k = cv::waitKey(0);
        if (k == ' ') {
            // redo calibration
            return E_FAIL;
        }
    }

    // Good calibration
    cv::destroyAllWindows();
    return S_OK;
}


/**
* @brief Use the calibration structure to compute the warping meshes
*        for fast calibration
*/
void GeometryCalibration::RecomputeLookupTables() {

    // Recompute calibration model
    printf("[INFO] Recompute calibration lookup tables... ");

    if (calib_.width_c == 0 || calib_.height_c == 0 ||
        calib_.width_t == 0 || calib_.height_t == 0 ||
        calib_.scale_t == 0) {
        std::cout << "[INFO][GeometryCalibration::RecomputeLookupTables] Empty calibration model."
            << std::endl;
        return;
    }

    // resize lookup tables
    lookup_table_x.resize(calib_.width_c * calib_.height_c);
    lookup_table_y.resize(calib_.width_c * calib_.height_c);

    // Computes lookup tables
    cv::Mat homography(cv::Size(3, 3), CV_64FC1, calib_.h);
    cv::Mat h_inv = homography.inv();
    float scale_x = 1.f / calib_.scale_t;
    float scale_y = 1.f / calib_.scale_t;
    size_t idx = 0;
    for (int i = 0; i < calib_.height_c; ++i) {
        for (int j = 0; j < calib_.width_c; ++j) {

            // Compute IR location
            float new_col = (j * h_inv.at<double>(0, 0) + i * h_inv.at<double>(0, 1) + h_inv.at<double>(0, 2)) /
                (j * h_inv.at<double>(2, 0) + i * h_inv.at<double>(2, 1) + h_inv.at<double>(2, 2));
            float new_line = (j * h_inv.at<double>(1, 0) + i * h_inv.at<double>(1, 1) + h_inv.at<double>(1, 2)) /
                (j * h_inv.at<double>(2, 0) + i * h_inv.at<double>(2, 1) + h_inv.at<double>(2, 2));
            
            // Color pixel projected onto thermal image
            lookup_table_x[idx] = new_col * scale_x;
            lookup_table_y[idx] = calib_.height_t - new_line * scale_y - 1; // add a flip here for speed purposes
            // the thermal image comes flipped from the sensor, so while warping the image we apply the flip too

            // Checks if projection is still within image area
            if (lookup_table_x[idx] < 1.f || lookup_table_y[idx] < 1.f ||
                lookup_table_x[idx] > (calib_.width_t-1) || lookup_table_y[idx] > (calib_.height_t-1)) {

                lookup_table_x[idx] = -1.f;
                lookup_table_y[idx] = -1.f;
            }
            ++idx;
        }
    }
    printf(" Done! \n");
}

// Utils: detect chessboard in thermal image
//void thermalImageChessboard(const std::vector<unsigned short> &temperature_u16,
//                            cv::Mat &img) {
//
//    if (temperature_u16.size() != img.rows*img.cols) {
//        std::cout << "[thermalImageChessboard] incompatible inputs." << std::endl;
//        return;
//    }
//
//    //long long sum = 0;
//    //for (int i = 0; i < temperature_u16.size(); ++i) {
//    //    sum += temperature_u16[i];
//    //}
//    //unsigned short avg = static_cast<unsigned short>(sum / temperature_u16.size());
//    unsigned char* img_data = img.data;
//    const double min = 7900;
//    const double max = 8000;
//    memset(img_data, 0, temperature_u16.size());
//    for (int i = 0; i < temperature_u16.size(); ++i) {
//        /*if (temperature_u16[i] > avg) {
//            img_data[i] = 255;
//        }*/
//        double t_v = (temperature_u16[i] - min) / (max - min) * 255.0;
//        if (t_v < 0) {
//            t_v = 0;
//        }
//        else if (t_v > 255) {
//            t_v = 255;
//        }
//        img_data[i] = static_cast<unsigned char>(t_v);
//    }
//}

// Do depth sensor to thermal imager calibration
//bool OpenCVCalibration(ThermalSensor& thermal_sensor, Sensor3D& depth_sensor) {
//
//    int T_s = 0;
//    float T_o = 0.f;
//    const int width_c = depth_sensor.getColorFrameWidth();
//    const int height_c = depth_sensor.getColorFrameHeight();
//    const int bpp_c = depth_sensor.getColorFrameBPP();
//    const int width_d = depth_sensor.getDepthFrameWidth();
//    const int height_d = depth_sensor.getDepthFrameHeight();
//    const int width_t = thermal_sensor.getFrameWidth();
//    const int height_t = thermal_sensor.getFrameHeight();
//    std::vector<unsigned short> temperature_u16(width_t * height_t);
//    std::vector<unsigned short> frame_depth(width_d * height_d);
//    std::vector<unsigned char> frame_color(height_c * width_c * bpp_c);
//   
//    const int fixed_width = 1280;
//    const int fixes_height = 960;
//    const int t_scale = fixed_width / width_t;
//    const int c_width_offset = (width_c - fixed_width) / 2;
//    const int c_height_offset = (height_c - fixes_height) / 2;
//    const cv::Rect rect(c_width_offset, c_height_offset, 
//        width_c - 2* c_width_offset, height_c - 2* c_height_offset);
//
//    const int nx = 6;
//    const int ny = 9;
//    const int n = nx*ny;
//    std::vector<std::vector<cv::Point2f> > corners[2];
//
//    // Collect 10 images
//    int count = 0;
//    while (count < 10) {
//        
//        // Get images
//        while (!thermal_sensor.hasNewFrame() || !depth_sensor.hasNewFrame()) {
//            Sleep(100);
//        }
//        depth_sensor.nextFrame(frame_depth.data(), frame_color.data(), true);
//        thermal_sensor.nextFrame(temperature_u16.data(), T_s, T_o, true);
//
//        // Process images
//        cv::Mat t_gray(height_t, width_t, CV_8UC1);
//        thermalImageChessboard(temperature_u16, t_gray);
//        cv::flip(t_gray, t_gray, 0);
//        cv::Mat t_gray_scaled;
//        cv::resize(t_gray, t_gray_scaled, cv::Size(fixed_width, fixes_height));
//
//        cv::Mat rgb = cv::Mat(height_c, width_c, CV_8UC4, frame_color.data());
//        cv::Mat rgb_roi = rgb(rect);
//
//        // Get corners
//        //if (corners[0][count].size() != n || corners[1][count].size() != n) {
//        //    continue;
//        //}
//
//        cv::imshow("RGB ROI", rgb_roi);
//        cv::imshow("T", t_gray);
//        char key = cv::waitKey(50);
//        if (key == 27) { // esc
//            break;
//        }
//        else if (key == ' ') { // keep
//            count++;
//        }
//    }
//    cv::destroyAllWindows();
//    if (count < 10) {
//        // force exit
//        return true;
//    }
//
//    // Stereo calib
//
//    // Stereo rectify
//
//    // Compute lookup tables
//    return true;
//}

bool GeometryCalibration::Calibration(ThermalSensor& thermal_sensor, Sensor3D& depth_sensor) {

    //if (1) {
    //    return OpenCVCalibration(thermal_sensor, depth_sensor);
    //}

    // Compute disparity to depth conversion
    float inv_coeff_disp_ = depth_sensor.getColorFrameHeight() / 
        (2 * tan(depth_sensor.getColorVerticalFOV() * 0.5f * PI / 180.0)) * kBaseline;
    // Kinect sensor invers coefficient for disparity computation
    // FOV = 2 arctan (x / (2 f)) or f = x / (2 * tan(FOV / 2))
    // H-FOV = 84.1 => f = 1064.31 (0.5*width/tan(84.1/2)) => 
    // disp_2_depth = B * f = 30.86 (for B = 29mm = 0.029m)
    // V-HOV = 53.8 => f = 1064.39 (0.5*height/tan(53.8/2)) =>
    // disp_2_depth = B * f = 30.86 (for B = 29mm = 0.029m)

    // Wait for new frame
    int T_s = 0;
    float T_o = 0.f;
    const int width_c = depth_sensor.getColorFrameWidth();
    const int height_c = depth_sensor.getColorFrameHeight();
    const int bpp_c = depth_sensor.getColorFrameBPP();
    const int width_d = depth_sensor.getDepthFrameWidth();
    const int height_d = depth_sensor.getDepthFrameHeight();
    const int width_t = thermal_sensor.getFrameWidth();
    const int height_t = thermal_sensor.getFrameHeight();
    std::vector<unsigned char> temperature_u8(width_t * height_t);
    std::vector<unsigned short> frame_depth(width_d * height_d);
    std::vector<unsigned char> frame_color(height_c * width_c * bpp_c);

    size_t num_img = 0;
    std::vector<CvPoint2D32f> points[2];
    bool debug_calib = true;
    cv::Mat ir_img_;
    cv::Mat kinect_gray_r_;
    while (num_img < 4) {
        while (!thermal_sensor.hasNewFrame() || !depth_sensor.hasNewFrame()) {
            Sleep(100);
        }
        depth_sensor.nextFrame(frame_depth.data(), frame_color.data(), true);
        thermal_sensor.nextFrame(temperature_u8.data(), T_s, T_o, true);
        
        // Compute corners IR
        ir_img_ = cv::Mat(cv::Size(width_t, height_t), CV_8UC1, temperature_u8.data());
        cv::flip(ir_img_, ir_img_, 0);
        cv::resize(ir_img_, ir_img_, cv::Size(width_t * k_thermal_scale_up_, height_t * k_thermal_scale_up_));
        HRESULT r1 = stereoCornersDetect(ir_img_, kNx, kNy, points[1], 1, debug_calib);

        // Compute corners RGB
        cv::Mat kinect_gray_(cv::Size(width_c, height_c), CV_8UC4, frame_color.data());
        cv::cvtColor(kinect_gray_, kinect_gray_r_, CV_BGRA2GRAY);
        HRESULT r2 = stereoCornersDetect(kinect_gray_r_, kNx, kNy, points[0], 0, debug_calib);

        if (FAILED(r1) || FAILED(r2)) {
            continue;
        }
        num_img++;
    }
    // Compute Homography
    cv::Mat homography;     // perspective projection between IR and RGB image planes
    int delta_disp_ = 0;    // disparity shift because of the non-infinity calibration
/*    if (FAILED(r1) || FAILED(r2)) {
        std::cerr << "[Error][GeometryCalibration] Unable to detect all corners."
            << std::endl;
        return false;
    }
    else*/ {
        // Computes homography
        ICoordinateMapper** mapper = depth_sensor.getCoordinateMapper();
        std::vector<CameraSpacePoint> depth_XYZ(width_c * height_c);
        HRESULT hr = (*mapper)->MapColorFrameToCameraSpace(width_d * height_d,
            (UINT16*)frame_depth.data(),
            width_c * height_c,
            depth_XYZ.data());

        HRESULT r3 = stereoCalib(points, depth_XYZ.data(),
            inv_coeff_disp_, homography, delta_disp_,
            ir_img_, kinect_gray_r_, 1, true);

        // Cheks if calibration failed
        if (FAILED(r3)) {
            std::cerr << "[Warning][GeometryCalibration] Unable to compute homography."
                << std::endl;
            return false;
        }

        // Store calibration parameters
        calib_.height_c = depth_sensor.getColorFrameHeight();
        calib_.width_c = depth_sensor.getColorFrameWidth();
        calib_.width_d = depth_sensor.getDepthFrameWidth();
        calib_.height_d = depth_sensor.getDepthFrameHeight();
        calib_.height_t = thermal_sensor.getFrameHeight();
        calib_.width_t = thermal_sensor.getFrameWidth();
        calib_.scale_t = k_thermal_scale_up_;
        calib_.deltaDisp = delta_disp_;
        calib_.invCoeff = inv_coeff_disp_;
        memcpy(&calib_.h, homography.data, 8 * 9); // 8 bytes, 9 elements

        // Saves calibration log
        std::ofstream myfile;
        myfile.open("geometry_model.log");
        myfile << "\nHomography: \n" << homography << std::endl;
        myfile << "Depth params: \n   - deltaDisp=" << delta_disp_
            << "\n   - invCoeffDisp=" << inv_coeff_disp_
            << "\n   - Color image resolution=" << calib_.width_c << " x " << calib_.height_c
            << "\n   - Depth image resolution=" << calib_.width_d << " x " << calib_.height_d
            << "\n   - Thermal image resolution=" << calib_.width_t << " x " << calib_.height_t
            << "\n   - Thermal image scale=" << calib_.scale_t
            << std::endl;
        myfile.close();

        // Save to file calibration
        std::ofstream output_file("geometry_model.bin", std::ios::binary);
        output_file.write((char*)&calib_, sizeof(calib_));
        output_file.close();

        // Compute the lookup tables for fast warping
        RecomputeLookupTables();
    }

    if (debug_calib) {
        char k = cv::waitKey(50);
        if (k == 27) {
            // force exit
            return true;
        }
    }

    // Everything okay
    return true;
}


// Load depth sensor to thermal imager calibration
bool GeometryCalibration::LoadCalibration(ThermalSensorType sensor_type) {

    // Load from file calibration
    printf("[INFO] Load geometry calibration from file... ");

    std::string file_name;
    if (sensor_type == LEPTON_V2) {
        file_name = "l2_geometry_model.bin";
    }
    else { // Lepton V3
        file_name = "l3_geometry_model.bin";
    }
    std::ifstream input_file(file_name, std::ios::binary);
    if (!input_file.is_open()) {
        printf("\n\tError opening file ref_geometry_model.csv to read calibration model.\n");
        return false;
    }
    input_file.read((char*)&calib_, sizeof(calib_));
    input_file.close();

    printf("\n\tDelta disp: %d \n\tInverse coefficient %f \n\tHomography:\n"
        "\t %f  %f  %f\n\t %f  %f  %f\n\t %f  %f  %f\n", calib_.deltaDisp, calib_.invCoeff,
        calib_.h[0], calib_.h[1], calib_.h[2], calib_.h[3], calib_.h[4], calib_.h[5],
        calib_.h[6], calib_.h[7], calib_.h[8]);
    printf("\t Color image resolution: %d x %d\n", calib_.width_c, calib_.height_c);
    printf("\t Depth image resolution: %d x %d\n", calib_.width_d, calib_.height_d);
    printf("\t Thermal image resolution: %d x %d\n", calib_.width_t, calib_.height_t);
    printf("\t Thermal image scale: %d\n", calib_.scale_t);

    // Recompute lookup tables
    RecomputeLookupTables();

    return true;
}
