#include "visual_stabilization/camera_calibration.hpp"

cameraCalibration::cameraCalibration(){}
cameraCalibration::~cameraCalibration(){}


void cameraCalibration::setIntrinsicCameraInfo(int width, int height, std::vector<double> K, std::vector<double> D, bool fisheye){

  frameWidth_ = width;
  frameHeight_ = height;

  fishEyeLensDistortion_ = fisheye;
  if(fishEyeLensDistortion_)
    ROS_INFO("Fisheye undistortion activated");

  loadCameraCalibration(K,D);
  computeUniditortTable(fishEyeLensDistortion_);
}

void cameraCalibration::loadCameraCalibration(std::vector<double> K, std::vector<double> D){
  // Initialize the camera matrix and distortion coefficients opencv
  K_cv_ = cv::Mat(3, 3, CV_32F);
  if(fishEyeLensDistortion_)
    D_cv_ = cv::Mat(4, 1, CV_32F, cv::Scalar(0)); // bound to max 4 coef due to Opencv issues
  else
    D_cv_ = cv::Mat(D.size(), 1, CV_32F, cv::Scalar(0));

  // Fill camera matrix and distrotion opencv 
  for (int i = 0; i < 3; i++){
    for (int j = 0; j < 3; j++)
      K_cv_.at<float>(j,i) = K[i+j*3];
  }

  double sum = 0.0;
  for (int i = 0; i < D.size(); i++){
    sum += D[i];
    if(fishEyeLensDistortion_ && i<4)
      D_cv_.at<float>(i) = D[i];
    else
      D_cv_.at<float>(i) = D[i];
  }

  // Check for lens distortion
  if(sum == 0.0){
    ROS_INFO("Zero distortion coeficients, undistortion disabled ");
    applyUndistortion_ = false;
  }

  if(fishEyeLensDistortion_)
    cv::fisheye::initUndistortRectifyMap(K_cv_, D_cv_, cv::Matx33d::eye(), K_cv_, cv::Size(frameWidth_,frameHeight_), CV_16SC2, map1_, map2_);
  else
    cv::initUndistortRectifyMap(K_cv_, D_cv_, cv::Matx33d::eye(), K_cv_, cv::Size(frameWidth_,frameHeight_), CV_16SC2, map1_, map2_);

  K_tf_ = tf2::Matrix3x3(K[0], K[1], K[2], K[3], K[4], K[5], K[6], K[7], K[8]);
  K_tf_inverse_ = K_tf_.inverse();

  ROS_INFO_STREAM("Calibration Matrix: " << K_cv_);
  ROS_INFO_STREAM("Distortion coefficients: " <<D_cv_);

}

void cameraCalibration::computeUniditortTable(bool fisheye){

  if(applyUndistortion_){
    std::vector<cv::Point2d> image_pixels;
    for (int y = 0; y<frameHeight_; y++){
      for (int x = 0; x<frameWidth_; x++)
        image_pixels.push_back(cv::Point2d(x,y));
    }

    if(fisheye)
      cv::fisheye::undistortPoints(image_pixels, undistortTable, K_cv_, D_cv_, cv::Matx33d::eye(), K_cv_);
    else
      cv::undistortPoints(image_pixels, undistortTable, K_cv_, D_cv_, cv::Matx33d::eye(), K_cv_);
  }
  else
    ROS_INFO("The undistortion is deactivated. Undistortion table empty");
}

void cameraCalibration::undistortFrame(cv::Mat &im){
  if (applyUndistortion_){
    cv::Mat im_original = im.clone();
    cv::remap(im_original, im, map1_, map2_, cv::INTER_CUBIC,
             CV_HAL_BORDER_CONSTANT);
  }
}

bool cameraCalibration::activateUndistortion(){
  return applyUndistortion_;
}

tf2::Matrix3x3 cameraCalibration::getCameraMatrixtf2(){
  return K_tf_;
}

tf2::Matrix3x3 cameraCalibration::getInverseCameraMatrixtf2(){
  return K_tf_inverse_;
}
