#include "visual_stabilization/camera_rotation_stabilizer.hpp"

cameraRotationStabilizer::cameraRotationStabilizer(){}
cameraRotationStabilizer::~cameraRotationStabilizer(){}

void cameraRotationStabilizer::update(const geometry_msgs::Quaternion msg){
  static bool firstIteration = true;
  tf2::Quaternion q_k = tf2::Quaternion(msg.x, msg.y, msg.z, msg.w);
  if(activate_external_rotation_)
    rotateOrientation(q_k);
  ROS_DEBUG("Current quaternion, qx: %f qy: %f  qz: %f  qw: %f",q_k.x(),q_k.y(),q_k.z(),q_k.w());

  if(firstIteration){
    // Keep the first quaterion to set it as the initial orientation.
    q_init_ = q_k;
    q_init_inv_ = q_init_.inverse();

    q_k = q_init_inv_*q_k;
    q_k.normalize();
    // Update global quaterion
    q_gt_s_ = q_k;

    ROS_INFO("Saving orientation reference");
    ROS_INFO("Reference quaternion, qx: %f qy: %f  qz: %f  qw: %f",q_init_.x(),q_init_.y(),q_init_.z(),q_init_.w());
    firstIteration = false;
  }
  else{
    q_k = q_init_inv_*q_k;
    q_k.normalize();
    // Update global quaterion
    q_gt_s_ = q_k;
  }
}

bool cameraRotationStabilizer::updateAdaptativeReferenceByPixelLocation(const geometry_msgs::Quaternion msg, const ros::Time stamp, tf2::Matrix3x3 K, tf2::Matrix3x3 K_inv, int pRef_x, int pRef_y, int delta_d, int camWidth, int camHeight){
  
  static bool firstIteration = true;
  static tf2::Quaternion q_k_prev_init;

  bool output = false;
  tf2::Quaternion q_k = tf2::Quaternion(msg.x, msg.y, msg.z, msg.w);
  if(activate_external_rotation_)
    rotateOrientation(q_k);

  ROS_DEBUG("Current quaternion, qx: %f qy: %f  qz: %f  qw: %f",q_k.x(),q_k.y(),q_k.z(),q_k.w());

  if(firstIteration){
    // Save the first quaterion to set it as the initial orientation.
    q_init_ = q_k;
    q_init_inv_ = q_init_.inverse();
    // Keep prev reference
    q_k_prev_init = q_k;

    q_k = q_init_inv_*q_k;
    q_k.normalize();
    // Update global quaterion
    q_gt_s_ = q_k;
    // Update previous global quaternion
    q_gt_s_prev_ = q_gt_s_;

    ROS_DEBUG("First Reference quaternion, qx: %f qy: %f  qz: %f  qw: %f",q_init_.x(),q_init_.y(),q_init_.z(),q_init_.w());

    firstIteration = false;
  }
  else{
    tf2::Quaternion q_aux = q_init_inv_*q_k;
    q_aux.normalize();
    // Apply current transformation to the reference point
    tf2::Matrix3x3 R(q_aux);
    tf2::Vector3 v = K*R*K_inv*tf2::Vector3(pRef_x,pRef_y,1.0);
    v = v/v.z();
    // Check if the point lies inside the camera frame
    if(v.x() < delta_d || v.y() < delta_d || v.x() >= camWidth-delta_d || v.y() >= camHeight-delta_d){
      ROS_DEBUG("Changing reference orientation");
      // Keep latest global rotation
      q_gt_s_prev_ = q_gt_s_;
      q_init_ = q_k_prev_init;
      q_init_inv_ = q_init_.inverse();
      output = true;
    }

    // update prev init
    q_k_prev_init = q_k;

    q_k = q_init_inv_*q_k;
    q_k.normalize();
    // Update global quaterion
    q_gt_s_ = q_k;

  }

  return output;
}

void cameraRotationStabilizer::rotateOrientation(tf2::Quaternion &q){
  q = q_ext_*q*q_ext_conjugate_;
  q.normalize();
}

tf2::Quaternion cameraRotationStabilizer::getCameraOrientation(){
  return q_gt_s_;
}

tf2::Quaternion cameraRotationStabilizer::getPreviousCameraOrientation(){
  return q_gt_s_prev_;
}

void cameraRotationStabilizer::setExternalRotation(tf2::Quaternion q){
  ROS_INFO("External extrinsic rotation activated");
  q_ext_ = q;
  q_ext_conjugate_ = tf2::Quaternion(-q_ext_.x(), -q_ext_.y(), -q_ext_.z(), q_ext_.w());
  activate_external_rotation_ = true;
}
