#ifndef EVENT_STABILIZATION_H
#define EVENT_STABILIZATION_H

#include "ros/ros.h"
#include <rosbag/bag.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Time.h>
#include <std_msgs/Header.h>
#include <dvs_msgs/Event.h>
#include <dvs_msgs/EventArray.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

#include "visual_stabilization/d_event.hpp"
#include "visual_stabilization/camera_calibration.hpp"
#include "visual_stabilization/event_windowing.hpp"
#include "visual_stabilization/camera_rotation_stabilizer.hpp"
#include "visual_stabilization/vision_stabilizer.hpp"

class eventStabilization{
  private:

    ros::Publisher eventImageUndistortPub_, eventImageStabilizedPub_, eventImageStabilizedExtendedPub_;
    ros::Subscriber eventSub_, externalPoseSub_, externalImuSub_;
    sensor_msgs::ImagePtr imMsg_;
    int cameraWidth_ = 240, cameraHeight_ = 180, sample_counter_ = 0, saccade_area_ = 40, 
    heightCanvasExtended_, widthCanvasExtended_, camW2_, camH2_;
    bool poseDataReady_ = false, enableSaccades_ = false, saccade_ = false, 
    record_bag_ = false, undistort_events_ = true;
    double canvas_gain_ = 1.0;
    rosbag::Bag bag_;
    ros::Time publishTime_;

    std::vector<dEvent> eventBuffer_, eventStabilized_;
    std::vector<ros::Time> eventTimeBuffer_;
    std::vector<geometry_msgs::Quaternion> rot_list_;
    std::vector<std_msgs::Header> rot_time_list_;

    // Classes
    cameraCalibration cameraCalib_;
    eventWindowing eventWindower_;
    cameraRotationStabilizer camOrientation_;
    visionStabilizer stabilizer_;

    // Methods
    void initializeParameters(double tStart);
    void publishAndRender();
    void applyStabilization();
    bool matchEventOrientation();
    void updateTimeMatchOrientation(geometry_msgs::Quaternion q, std_msgs::Header header);

    // Callbacks
    void eventCallback(const dvs_msgs::EventArray::ConstPtr& msg);
    void externalImuCallback(const sensor_msgs::Imu::ConstPtr& msg);
    void externalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);

  public:
    eventStabilization(ros::NodeHandle &n, ros::NodeHandle &npublic);
    ~eventStabilization(void);
};
#endif //EVENT_STABILIZATION_H
