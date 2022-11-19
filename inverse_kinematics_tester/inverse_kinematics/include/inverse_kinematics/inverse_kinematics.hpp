#pragma once
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/QR>
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Dense>
#include <vector>
#include "rclcpp/rclcpp.hpp"
using std::vector;

class InverseKinematics
{
private:
    // Robot's dimensions
    // For more clarification have a look at the README.md file
    
    // // Body dimensions
    float body_length_;
    float body_width_;

    // // height from bottom of body to the plane spanned by the hip yaw motors
    float base_frame_height_;

    // // Leg Dimensions
    float upper_leg_x_;
    float upper_leg_y_;
    float upper_leg_z_;
    float lower_leg_x_;
    float lower_leg_y_;
    float lower_leg_z_;

    // // Joint frames relative position to each other
    float distance_hip_joints_;

    // // distance between hip yaw joint and hip pitch joint
    float distance_between_hip_joints_;

    // // Hip yaw location in B frame
    Eigen::Matrix4f hip_yaw_locations_;

    // // Relative position joint vector
    Eigen::Matrix3f relative_joint_vectors_;

    //Location of body IMU relative to the geometric 
    //center of the body
    vector<double> pos_imu_rel_to_b_frame_;

    // transformation matrices for magnecko
    // 1 = hip_yaw, 2 = hip_pitch, 3 = knee_pitch, 4 = foot
    Eigen::Matrix4f joint_to_transform_B1(Eigen::Vector3f q, Eigen::Vector3f B_r_B1);
    Eigen::Matrix4f joint_to_transform_12(Eigen::Vector3f q, Eigen::Vector3f B_r_12);
    Eigen::Matrix4f joint_to_transform_23(Eigen::Vector3f q, Eigen::Vector3f B_r_23);
    Eigen::Matrix4f joint_to_transform_34(Eigen::Vector3f q, Eigen::Vector3f B_r_34);

    // find base to foot vector
    Eigen::Vector3f find_base_to_foot_vector(Eigen::Vector3f q_one_leg, Eigen::Vector3f B_r_B1);

    // find transformation matrix from I fram to B frame
    Eigen::Matrix4f get_transform_IB(Eigen::VectorXf q, const Eigen::Vector3f body_orientation, const vector<int> stationary_feet);

    // Calculate position jacobian from B frame to End effector
    Eigen::Matrix3Xf joint_to_position_jacobian(Eigen::Vector3f q, Eigen::Vector3f B_r_B1);

public:
    // InverseKinematics class constructor - set robot's dimensions
    InverseKinematics(const float body_dimensions[], const float leg_dimensions[][3], const float base_frame_height, const float distance_between_hip_joints);

    // compute inverse kinematics for a given leg position
    Eigen::VectorXf inverse_kinematics(Eigen::VectorXf q_0, Eigen::Vector3f I_r_IE_des, const vector<int> stationary_feet, const Eigen::Vector3f body_orientation, Eigen::Matrix3f I_C_IE_des);
};