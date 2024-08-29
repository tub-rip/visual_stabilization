#ifndef FRAME_STABILIZATION_H
#define FRAME_STABILIZATION_H

#include "ros/ros.h"
#include <rosbag/bag.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <vector>

#include "visual_stabilization/vision_stabilizer.hpp"
#include "visual_stabilization/camera_calibration.hpp"
#include "visual_stabilization/camera_rotation_stabilizer.hpp"

class frameStabilization{
  private:
    ros::Publisher warpImagePub_, warpImageRoiPub_, warpExtendedImagePub_;
    ros::Subscriber externalPoseSub_, imageSub_, externalImuSub_;

    int cameraWidth_ = 240, cameraHeight_ = 180, sample_counter_ = 0, saccade_area_ = 40,
    camW2_, camH2_, extendedW_, extendedH_;
    bool poseDataReady_ = false, enableSaccades_ = false, saccade_ = false, 
    undistortFrame_ = false, record_bag_ = false;
    double canvas_gain_ = 1.0;
    rosbag::Bag bag_;

    sensor_msgs::ImagePtr imMsg_;
    ros::Time publishTime_;
    tf2::Matrix3x3 H_;
    std::vector<geometry_msgs::Quaternion> rot_list_;
    std::vector<std_msgs::Header> rot_time_list_;

    // Objects
    cameraCalibration cameraCalib_;
    cameraRotationStabilizer camOrientation_;
    visionStabilizer stabilizer_;

    // Callbacks
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
    void externalPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void externalImuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    // Methods
    void initializeParameters();
    cv::Mat getPerspectiveTransformationMatrix();
    void publish(cv::Mat &im1, cv::Mat &im2, cv::Mat &im3);
    bool matchImageOrientation();
    void updateTimeMatchOrientation(geometry_msgs::Quaternion q, std_msgs::Header header);

  public:
    frameStabilization(ros::NodeHandle &n, ros::NodeHandle &npublic);
    ~frameStabilization(void);
};
#endif //FRAME_STABILIZATION_H
