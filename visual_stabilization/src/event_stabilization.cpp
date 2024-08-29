
#include "visual_stabilization/event_stabilization.hpp"

int main(int argc, char **argv){
  ros::init(argc, argv, "event_stabilization_node");
  ros::NodeHandle nh("~");
  ros::NodeHandle nh_public;

  eventStabilization rc(nh, nh_public);
  ros::spin();
}

eventStabilization::eventStabilization(ros::NodeHandle &nh, ros::NodeHandle &nh_public){

  bool orientationFromImu = false, orientationFromOdometry = false;
  nh.getParam("get_orientation_from_imu",orientationFromImu);
  if(orientationFromImu){
    ROS_INFO("Getting orientation from IMU");
    std::string external_imu_topic;
    nh.getParam("topic_external_imu_param",external_imu_topic);
    externalImuSub_ = nh_public.subscribe(external_imu_topic, 0, &eventStabilization::externalImuCallback, this);
  }
  else{
    ROS_INFO("Getting orientation from mocap");
    std::string external_pose_topic;
    nh.getParam("topic_external_pose_param",external_pose_topic);
    externalPoseSub_ = nh_public.subscribe(external_pose_topic, 0, &eventStabilization::externalPoseCallback, this);
  }
  std::string event_topic, imu_topic;
  nh.getParam("topic_event_param", event_topic);
  nh.getParam("topic_imu_param", imu_topic);
  eventSub_ = nh_public.subscribe(event_topic, 0, &eventStabilization::eventCallback, this);

  eventImageUndistortPub_ = nh_public.advertise<sensor_msgs::Image>("undistorted_event_image", 1);
  eventImageStabilizedPub_ = nh_public.advertise<sensor_msgs::Image>("stabilized_event_image", 1);
  eventImageStabilizedExtendedPub_ = nh_public.advertise<sensor_msgs::Image>("extended_stabilized_event_image", 1);

  nh.getParam("enable_saccades",enableSaccades_);
  nh.getParam("canvas_gain", canvas_gain_);
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
  nh.getParam("enable_undistortion",undistort_events_);
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

  int accNevents, roiTh, n_rows, n_cols, r;
  double accTime;
  std::string accumlationType;
  // Parameters event accumulator
  nh.getParam("accumulation_type", accumlationType);
  nh.getParam("accumulation_time", accTime);
  nh.getParam("number_of_accumulated_events", accNevents);
  nh.getParam("roi_threshold", roiTh);
  nh.getParam("number_of_columns", n_cols);
  nh.getParam("number_of_rows", n_rows);
  eventWindower_ = eventWindowing(accumlationType, accNevents, roiTh, accTime, n_rows, n_cols);

  nh.getParam("record_bag", record_bag_);
  if(record_bag_){
    std::string file_name;
    nh.getParam("file_name", file_name);
    bag_.open(file_name, rosbag::bagmode::Write);
  }
}
eventStabilization::~eventStabilization(){
  if(record_bag_)
    bag_.close();
}

void eventStabilization::initializeParameters(double tStart){
  camW2_ = cameraWidth_/2;
  camH2_ = cameraHeight_/2;
  heightCanvasExtended_  = (int)cameraHeight_*canvas_gain_;
  widthCanvasExtended_ = (int)cameraWidth_*canvas_gain_;
  eventWindower_.initParameters(cameraWidth_, cameraHeight_, tStart);
}

void eventStabilization::eventCallback(const dvs_msgs::EventArray::ConstPtr& msg){
  static bool firstIteration = true;

  if (!poseDataReady_)
    return;

  int n_events = msg->events.size();
  if (n_events == 0)
    return;

  if (firstIteration){
    initializeParameters(msg->events[0].ts.toSec());
    firstIteration = false;
  }

  dEvent e;
  for(auto e_raw : msg->events){
    e = dEvent(e_raw.ts.toSec(), (double)e_raw.x, (double)e_raw.y, e_raw.polarity);
    // undistort event
    if(cameraCalib_.activateUndistortion() && undistort_events_){
      cv::Point2d p = cameraCalib_.undistortTable[(int)(e.y*cameraWidth_) + (int)e.x];
      e.x = p.x; e.y = p.y;
    }
    // Accumulate event
    eventBuffer_.push_back(e);
    eventTimeBuffer_.push_back(e_raw.ts);

    // Check triggering condition
    if(eventWindower_.update((int)e.x, (int)e.y, e.ts)){
      publishTime_ = e_raw.ts;

      // Update camera orientation with matched event timestamp
      if (matchEventOrientation()){
        // Update the last orientation before the reference changes. Thus, if positive, the condition updates the stabilization reference ....
        if(enableSaccades_ && saccade_){
          stabilizer_.setTransformation(camOrientation_.getPreviousCameraOrientation());
          ROS_INFO("Saccade, reference orientation have changed. Batch No : %d ",sample_counter_);
        }
        else
          stabilizer_.setTransformation(camOrientation_.getCameraOrientation());
        // stabilize events
        applyStabilization();
        // Publish events and render event frames
        publishAndRender();
        if(enableSaccades_ && saccade_)
          saccade_ = false;
        sample_counter_++;
      }

      // clear the buffer of events and timestamps
      eventBuffer_.clear();
      eventTimeBuffer_.clear();
    }
  }
}

void eventStabilization::applyStabilization(){
  eventStabilized_.clear();
  cv::Point2d pStabilized;
  for (int i = 0; i < eventBuffer_.size(); i++){
    pStabilized = stabilizer_.stabilizeEventOrientation(eventBuffer_[i].x,eventBuffer_[i].y);
    eventStabilized_.push_back(dEvent(eventBuffer_[i].ts, pStabilized.x, pStabilized.y, eventBuffer_[i].polarity));
  }
}

void eventStabilization::publishAndRender(){
  std_msgs::Header header_msg;
  header_msg.stamp = publishTime_;

  // event msgs to save bag
  dvs_msgs::EventArray events_msg_undistorted, events_msg_stabilized;
  // event frames
  cv::Mat eventImage = cv::Mat(cameraHeight_,cameraWidth_, CV_8UC3, cv::Scalar(255,255,255));
  cv::Mat eventImageStabilized = cv::Mat(cameraHeight_,cameraWidth_, CV_8UC3, cv::Scalar(255,255,255));
  cv::Mat eventImageStabilizedExtended = cv::Mat(heightCanvasExtended_,widthCanvasExtended_, CV_8UC3, cv::Scalar(255,255,255));

  // Prepare undistorted events
  for (int i = 0; i< eventBuffer_.size(); i++){
    int x = (int)round(eventBuffer_[i].x);
    int y = (int)round(eventBuffer_[i].y);
    if(x > 0 && x < cameraWidth_ && y > 0 && y < cameraHeight_){
      if (eventBuffer_[i].polarity)
        eventImage.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(255,0,0);
      else
        eventImage.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(0,0,255);

      if(record_bag_){  // Save undistored events
        dvs_msgs::Event e_undistorted;
        e_undistorted.x = x;
        e_undistorted.y = y;
        e_undistorted.ts = eventTimeBuffer_[i];
        e_undistorted.polarity = eventBuffer_[i].polarity;
        events_msg_undistorted.events.push_back(e_undistorted);
      }
    }
  }

  imMsg_ = cv_bridge::CvImage(header_msg, "bgr8", eventImage).toImageMsg();
  eventImageUndistortPub_.publish(imMsg_);

  // Prepare stabilized events
  for (int i = 0; i<eventStabilized_.size(); i++){
    int x = (int)round(eventStabilized_[i].x);
    int y = (int)round(eventStabilized_[i].y);
    if(x > 0 && x < cameraWidth_ && y > 0 && y < cameraHeight_){
      if (eventStabilized_[i].polarity)
        eventImageStabilized.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(255,0,0);
      else
        eventImageStabilized.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(0,0,255);

      if(record_bag_){    // Save stabilized events
        dvs_msgs::Event e_stabilized;
        e_stabilized.x = x;
        e_stabilized.y = y;
        e_stabilized.ts = eventTimeBuffer_[i];
        e_stabilized.polarity = eventStabilized_[i].polarity;
        events_msg_stabilized.events.push_back(e_stabilized);
      }
    }

    if(x > 0 && x < widthCanvasExtended_ && y > 0 && y < heightCanvasExtended_){
      if (eventStabilized_[i].polarity)
        eventImageStabilizedExtended.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(255,0,0);
      else
        eventImageStabilizedExtended.at<cv::Vec3b>(cv::Point(x,y)) = cv::Vec3b(0,0,255);
    }
  }

  imMsg_ = cv_bridge::CvImage(header_msg, "bgr8", eventImageStabilized).toImageMsg();
  eventImageStabilizedPub_.publish(imMsg_);

  imMsg_ = cv_bridge::CvImage(header_msg, "bgr8", eventImageStabilizedExtended).toImageMsg();
  eventImageStabilizedExtendedPub_.publish(imMsg_);

  if(record_bag_){
    events_msg_undistorted.header.stamp = eventTimeBuffer_[0];
    events_msg_undistorted.width = cameraWidth_;
    events_msg_undistorted.height = cameraHeight_;
    events_msg_stabilized.header.stamp = eventTimeBuffer_[0];
    events_msg_stabilized.width = cameraWidth_;
    events_msg_stabilized.height = cameraHeight_;

    // Save undistort events
    int n_events_in_buffer = eventTimeBuffer_.size()-1;
    bag_.write("/visual_stabilization/events", eventTimeBuffer_[n_events_in_buffer], events_msg_undistorted);
    // Save stabilized events
    bag_.write("/visual_stabilization/stabilized_events", eventTimeBuffer_[n_events_in_buffer], events_msg_stabilized);
  }
}

void eventStabilization::externalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
  static bool firstIteration = true;

  if (firstIteration){
    firstIteration = false;
    if(enableSaccades_){
      bool change = camOrientation_.updateAdaptativeReferenceByPixelLocation(msg->pose.orientation, msg->header.stamp, cameraCalib_.getCameraMatrixtf2(), cameraCalib_.getInverseCameraMatrixtf2(), camW2_, camH2_, saccade_area_, cameraWidth_, cameraHeight_);
      if(change)
        saccade_ = true;
    }
    else
      camOrientation_.update(msg->pose.orientation);
  }

  if(!poseDataReady_)
    poseDataReady_ = true;

  geometry_msgs::PoseStamped currentPose = *msg;
  rot_list_.push_back(currentPose.pose.orientation);
  rot_time_list_.push_back(currentPose.header);
}

void eventStabilization::externalImuCallback(const sensor_msgs::Imu::ConstPtr& msg){
  static bool firstIteration = true;

  if (firstIteration){
    firstIteration = false;
    if(enableSaccades_){
      bool change = camOrientation_.updateAdaptativeReferenceByPixelLocation(msg->orientation, msg->header.stamp, cameraCalib_.getCameraMatrixtf2(), cameraCalib_.getInverseCameraMatrixtf2(), camW2_, camH2_, saccade_area_, cameraWidth_, cameraHeight_);
      if(change)
        saccade_ = true;
    }
    else
      camOrientation_.update(msg->orientation);
  }

  if(!poseDataReady_){
    poseDataReady_ = true;
  }

  sensor_msgs::Imu currentImu = *msg;
  rot_list_.push_back(currentImu.orientation);
  rot_time_list_.push_back(currentImu.header);
}

bool eventStabilization::matchEventOrientation(){
  double image_t = eventTimeBuffer_[0].toSec();

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
      else
        ROS_INFO("Warning: Last rotation sample is after first event in the list");
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
    if (!sample_found)
      ROS_INFO("Warning: Rotation sample is after first event in the list");
    return sample_found;
  }
  else{ // the list is empty
    ROS_INFO("Warning: Rotation buffer is empty");
    return sample_found;
  }
}

void eventStabilization::updateTimeMatchOrientation(geometry_msgs::Quaternion q, std_msgs::Header header){
  if(enableSaccades_){
    bool change = camOrientation_.updateAdaptativeReferenceByPixelLocation(q, header.stamp, cameraCalib_.getCameraMatrixtf2(), cameraCalib_.getInverseCameraMatrixtf2(), camW2_, camH2_, saccade_area_, cameraWidth_, cameraHeight_);
    if(change)
      saccade_ = true;
  }
  else
    camOrientation_.update(q);
}