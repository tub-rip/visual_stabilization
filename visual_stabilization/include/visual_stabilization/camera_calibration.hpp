#ifndef CAMERA_CALIBRATION_H
#define CAMERA_CALIBRATION_H

#include "ros/ros.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <opencv2/opencv.hpp>

class cameraCalibration{
  private:
    int frameWidth_, frameHeight_;
    cv::Mat K_cv_, D_cv_;
    cv::Mat map1_, map2_; // undistortion maps OpenCV
    tf2::Matrix3x3 K_tf_, K_tf_inverse_;
    bool applyUndistortion_ = true, fishEyeLensDistortion_ = false;

    void loadCameraCalibration(std::vector<double> K, std::vector<double> D);
    void computeUniditortTable(bool fisheye);

  public:
    cameraCalibration();
    ~cameraCalibration(void);

    std::vector <cv::Point2d> undistortTable;
    void undistortFrame(cv::Mat &im);

    // Get
    tf2::Matrix3x3 getCameraMatrixtf2();
    tf2::Matrix3x3 getInverseCameraMatrixtf2();

    // Set
    void setIntrinsicCameraInfo(int width, int height, std::vector<double> K, std::vector<double> D, bool fisheye);
    bool activateUndistortion();
};

#endif // CAMERA_CALIBRATION_H
