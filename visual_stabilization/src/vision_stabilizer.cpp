#include "visual_stabilization/vision_stabilizer.hpp"

visionStabilizer::visionStabilizer(){}
visionStabilizer::~visionStabilizer(){}

void visionStabilizer::setCameraMatrix(tf2::Matrix3x3 K){
  K_ = K;
  K_inv_ = K_.inverse();
}

void visionStabilizer::setTransformation(tf2::Quaternion q){
  tf2::Matrix3x3 R(q);
  T_ = K_*R*K_inv_;

  // Update matrix parameters
  T11_ = T_.getRow(0).x(); 
  T12_ = T_.getRow(0).y(); 
  T13_ = T_.getRow(0).z(); 

  T21_ = T_.getRow(1).x(); 
  T22_ = T_.getRow(1).y(); 
  T23_ = T_.getRow(1).z(); 

  T31_ = T_.getRow(2).x(); 
  T32_ = T_.getRow(2).y(); 
  T33_ = T_.getRow(2).z(); 
}

tf2::Matrix3x3 visionStabilizer::getTransformation(){
  return T_;
}

cv::Point2d visionStabilizer::stabilizeEventOrientation(double x, double y){
  double z = x*T31_ + y*T32_ + T33_;
  double u = (x*T11_ + y*T12_ + T13_)/z;
  double v = (x*T21_ + y*T22_ + T23_)/z;

  return cv::Point2d(u,v);
}
