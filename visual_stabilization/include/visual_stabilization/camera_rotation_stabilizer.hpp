#ifndef CAMERA_ROTATION_STABILIZER_H
#define CAMERA_ROTATION_STABILIZER_H

#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <geometry_msgs/Quaternion.h>

class cameraRotationStabilizer{
  private:
    tf2::Quaternion q_init_, q_init_inv_, q_gt_s_, q_gt_s_prev_, q_ext_, q_ext_conjugate_;
    ros::Time stamp_init_;
    bool activate_external_rotation_ = false;
    void rotateOrientation(tf2::Quaternion &q);
  public:
    cameraRotationStabilizer();
    ~cameraRotationStabilizer(void);

    void update(const geometry_msgs::Quaternion msg);
    bool updateAdaptativeReferenceByPixelLocation(const geometry_msgs::Quaternion msg, const ros::Time stamp, tf2::Matrix3x3 K, tf2::Matrix3x3 K_inv, int pRef_x, int pRef_y, int delta_d, int camWidth, int camHeight);
    
    // Get
    tf2::Quaternion getCameraOrientation();
    tf2::Quaternion getPreviousCameraOrientation();

    // Set
    void setExternalRotation(tf2::Quaternion q);
};
#endif //CAMERA_ROTATION_STABILIZER_H
