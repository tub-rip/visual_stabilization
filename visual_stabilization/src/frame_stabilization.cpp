
#include "visual_stabilization/frame_stabilization.hpp"

int main(int argc, char **argv){
  ros::init(argc, argv, "frame_stabilization_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_public;

  frameStabilization rc(nh, nh_public);
  ros::spin();
}

frameStabilization::frameStabilization(ros::NodeHandle &nh, ros::NodeHandle &nh_public){

  bool orientationFromImu = false, orientationFromOdometry = false;
  nh.getParam("get_orientation_from_imu",orientationFromImu);
  if(orientationFromImu){
    ROS_INFO("Getting orientation from IMU");
    std::string external_imu_topic;
    nh.getParam("topic_external_imu_param",external_imu_topic);
    externalImuSub_ = nh_public.subscribe(external_imu_topic, 0, &frameStabilization::externalImuCallback, this);
  }
  else{
    ROS_INFO("Getting orientation from mocap");
    std::string external_pose_topic;
    nh.getParam("topic_external_pose_param",external_pose_topic);
    externalPoseSub_ = nh_public.subscribe(external_pose_topic, 0, &frameStabilization::externalPoseCallback, this);
  }
  std::string image_topic;
  nh.getParam("topic_image_param", image_topic);
  imageSub_ = nh_public.subscribe(image_topic, 0, &frameStabilization::imageCallback, this);

  warpImagePub_ = nh_public.advertise<sensor_msgs::Image>("undistorted_image", 1);
  warpImageRoiPub_ = nh_public.advertise<sensor_msgs::Image>("stabilized_image", 1);
  warpExtendedImagePub_ = nh_public.advertise<sensor_msgs::Image>("extended_stabilized_image", 1);

  nh.getParam("enable_saccades",enableSaccades_);
  nh.getParam("canvas_gain",canvas_gain_);
  if (canvas_gain_ < 1.0){
    canvas_gain_ = 1.0;
    ROS_INFO("Invalid canvas gain. Setting gain to 1.0");
  }


  std::vector <double> Kc, Dc;
  nh.getParam("image_width", cameraWidth_);
  nh.getParam("image_height", cameraHeight_);
  saccade_area_ = (int)(cameraWidth_/6.0);
  nh.getParam("K", Kc);
  nh.getParam("D", Dc);
  bool fisheye_distortion = false;
  nh.getParam("fisheye_distortion",fisheye_distortion);
  nh.getParam("enable_undistortion",undistortFrame_);
  cameraCalib_.setIntrinsicCameraInfo(cameraWidth_,cameraHeight_,Kc,Dc,fisheye_distortion);
  stabilizer_.setCameraMatrix(cameraCalib_.getCameraMatrixtf2());

  bool useExternalExtrinsicsRotation = false;
  nh.getParam("use_external_extrinsic_rotation", useExternalExtrinsicsRotation);
  if(useExternalExtrinsicsRotation){
    std::vector <double> quat;
    nh.getParam("q_external", quat);
    tf2::Quaternion q_external = tf2::Quaternion(quat[0],quat[1],quat[2],quat[3]);
    camOrientation_.setExternalRotation(q_external);
  }

  nh.getParam("record_bag", record_bag_);
  if(record_bag_){
    std::string file_name;
    nh.getParam("file_name", file_name);
    bag_.open(file_name, rosbag::bagmode::Write);
  }
}

frameStabilization::~frameStabilization(){
  if(record_bag_)
    bag_.close();
}

void frameStabilization::initializeParameters(){
  camW2_ = cameraWidth_/2;
  camH2_ = cameraHeight_/2;
  extendedW_ = (int)canvas_gain_*cameraWidth_;
  extendedH_ = (int)canvas_gain_*cameraHeight_;
}

void frameStabilization::imageCallback(const sensor_msgs::ImageConstPtr& msg){
  static bool firstIteration = true;
  static std::vector <cv::Point2f> refPoints;
  static cv::Mat img, imgStaticRoi, imgMovingRoi, imgExtended;

  if (!poseDataReady_)
    return;

  if (firstIteration){
    initializeParameters();
    firstIteration = false;
  }

  publishTime_ = msg->header.stamp;
  img = cv_bridge::toCvShare(msg, "mono8")->image;

  // Undistort image
  if(undistortFrame_)
    cameraCalib_.undistortFrame(img);
  
  // Find the closes pose sample to image
  bool valid = matchImageOrientation();
  if (!valid)
    return;

  // Set the reference orientation
  if(saccade_ && enableSaccades_)
    stabilizer_.setTransformation(camOrientation_.getPreviousCameraOrientation());
  else
    stabilizer_.setTransformation(camOrientation_.getCameraOrientation());

  // Get transformation from stabilizer
  H_ = stabilizer_.getTransformation();
  cv::Mat Tmatrix2 = getPerspectiveTransformationMatrix();
  // Allocate warp perspective image
  imgStaticRoi =  cv::Mat::zeros(cameraHeight_, cameraWidth_, img.type());
  imgExtended =  cv::Mat::zeros(extendedH_, extendedW_, img.type());

  // Perpective warp
  cv::warpPerspective(img, imgStaticRoi, Tmatrix2, imgStaticRoi.size(), cv::INTER_CUBIC);
  cv::warpPerspective(img, imgExtended, Tmatrix2, imgExtended.size(), cv::INTER_CUBIC);

  publish(img, imgStaticRoi, imgExtended);
}

void frameStabilization::publish(cv::Mat &im1, cv::Mat &im2, cv::Mat &im3){
  sample_counter_++;
  std_msgs::Header header_msg;
  header_msg.stamp = publishTime_;

  imMsg_ = cv_bridge::CvImage(header_msg, "mono8", im1).toImageMsg();
  warpImagePub_.publish(imMsg_);
  if(record_bag_)
    bag_.write("/visual_stabilization/image", header_msg.stamp, imMsg_);
  imMsg_ = cv_bridge::CvImage(header_msg, "mono8", im2).toImageMsg();
  warpImageRoiPub_.publish(imMsg_);
  if(record_bag_)
    bag_.write("/visual_stabilization/stabilized_image", header_msg.stamp, imMsg_);
  imMsg_ = cv_bridge::CvImage(header_msg, "mono8", im3).toImageMsg();
  warpExtendedImagePub_.publish(imMsg_);

  if(saccade_){
    ROS_INFO("Saccade, reference have changed. Image No : %d ",sample_counter_);
    saccade_ = false;
  }
}

cv::Mat frameStabilization::getPerspectiveTransformationMatrix(){
  cv::Mat P = cv::Mat(3, 3, CV_32F);
  for (int i = 0; i < 3; i++){
      P.at<float>(i,0) = H_[i].x();
      P.at<float>(i,1) = H_[i].y();
      P.at<float>(i,2) = H_[i].z();
  }
  return P;
}

void frameStabilization::externalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  static bool firstIteration = true;

  if(firstIteration){
    if(enableSaccades_){
      bool change = camOrientation_.updateAdaptativeReferenceByPixelLocation(msg->pose.orientation, msg->header.stamp, cameraCalib_.getCameraMatrixtf2(), cameraCalib_.getInverseCameraMatrixtf2(), camW2_, camH2_, saccade_area_, cameraWidth_, cameraHeight_);
    }
    else
      camOrientation_.update(msg->pose.orientation);
    firstIteration = false;
  }

  geometry_msgs::PoseStamped currentPose = *msg;
  rot_list_.push_back(currentPose.pose.orientation);
  rot_time_list_.push_back(currentPose.header);

  if(!poseDataReady_)
    poseDataReady_ = true;
}

void frameStabilization::externalImuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  static bool firstIteration = true;

  if(firstIteration){
    if(enableSaccades_){
      bool change = camOrientation_.updateAdaptativeReferenceByPixelLocation(msg->orientation, msg->header.stamp, cameraCalib_.getCameraMatrixtf2(), cameraCalib_.getInverseCameraMatrixtf2(), camW2_, camH2_, saccade_area_, cameraWidth_, cameraHeight_);
    }
    else
      camOrientation_.update(msg->orientation);
    firstIteration = false;
  }

  sensor_msgs::Imu currentImu = *msg;
  rot_list_.push_back(currentImu.orientation);
  rot_time_list_.push_back(currentImu.header);

  if(!poseDataReady_)
    poseDataReady_ = true;
}

bool frameStabilization::matchImageOrientation(){
  double image_t = publishTime_.toSec();

  bool sample_found = false;
  if (rot_time_list_.size()>1){
    int index = 0;
    for (int i=0; i<rot_time_list_.size()-1; i++ ){
      if(rot_time_list_[i].stamp.toSec() <= image_t && image_t < rot_time_list_[i+1].stamp.toSec()){
        index = i;
        updateTimeMatchOrientation(rot_list_[index], rot_time_list_[index]);
        sample_found = true;
        break;
      }
    }

    if (sample_found){
      // Erase old samples
      rot_list_.erase(rot_list_.begin() , rot_list_.begin() + index);
      rot_time_list_.erase(rot_time_list_.begin() , rot_time_list_.begin() + index);
    }
    else{ // check if at least the last sample occurred before the image
      if(image_t >= rot_time_list_[rot_time_list_.size()-1].stamp.toSec()){
        updateTimeMatchOrientation(rot_list_[rot_list_.size()-1], rot_time_list_[rot_time_list_.size()-1]);
        sample_found = true;
        /* CLEAN BOTH VECTORS */
        rot_list_.clear();
        rot_time_list_.clear();
      }
    }

    return sample_found;
  }
  else if(rot_time_list_.size() == 1){
    if (image_t>= rot_time_list_[0].stamp.toSec()){
      updateTimeMatchOrientation(rot_list_[0], rot_time_list_[0]);
      sample_found = true;
    }
    /* CLEAN BOTH VECTORS */
    rot_list_.clear();
    rot_time_list_.clear();
    return sample_found;
  }
  else // the list is empty
    return sample_found;
}

void frameStabilization::updateTimeMatchOrientation(geometry_msgs::Quaternion q, std_msgs::Header header){
  if(enableSaccades_){
    bool change = camOrientation_.updateAdaptativeReferenceByPixelLocation(q, header.stamp, cameraCalib_.getCameraMatrixtf2(), cameraCalib_.getInverseCameraMatrixtf2(), camW2_, camH2_, saccade_area_, cameraWidth_, cameraHeight_);
    if(change)
      saccade_ = true;
  }
  else
    camOrientation_.update(q);
}