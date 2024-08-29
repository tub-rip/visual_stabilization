#ifndef VISION_ORIENTATION_STABILIZER_H
#define VISION_ORIENTATION_STABILIZER_H

#include "ros/ros.h"
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Matrix3x3.h>

class visionStabilizer{
  private:
    tf2::Matrix3x3 T_, K_, K_inv_;
    double T11_, T12_, T13_, T21_, T22_, T23_, T31_, T32_, T33_;

  public:
    visionStabilizer();
    ~visionStabilizer();

    void setCameraMatrix(tf2::Matrix3x3 K);
    void setTransformation(tf2::Quaternion q);
    tf2::Matrix3x3 getTransformation();
    cv::Point2d stabilizeEventOrientation(double x, double y);
};
#endif //VISION_ORIENTATION_STABILIZER_H
