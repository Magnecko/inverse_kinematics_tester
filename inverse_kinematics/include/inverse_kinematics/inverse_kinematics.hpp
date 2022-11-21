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
    double body_length_;
    double body_width_;

    // // height from bottom of body to the plane spanned by the hip yaw motors
    double base_frame_height_;

    // // Leg Dimensions
    double upper_leg_x_;
    double upper_leg_y_;
    double upper_leg_z_;
    double lower_leg_x_;
    double lower_leg_y_;
    double lower_leg_z_;

    // // Joint frames relative position to each other
    double distance_hip_joints_;

    // // distance between hip yaw joint and hip pitch joint
    double distance_between_hip_joints_;

    // // Hip yaw location in B frame
    Eigen::Matrix4d hip_yaw_locations_;

    // // Relative position joint vector
    Eigen::Matrix3d relative_joint_vectors_;

    //Location of body IMU relative to the geometric 
    //center of the body
    vector<double> pos_imu_rel_to_b_frame_;

    // transformation matrices for magnecko
    // 1 = hip_yaw, 2 = hip_pitch, 3 = knee_pitch, 4 = foot
    Eigen::Matrix4d joint_to_transform_B1(Eigen::Vector3d q, Eigen::Vector3d B_r_B1);
    Eigen::Matrix4d joint_to_transform_12(Eigen::Vector3d q, Eigen::Vector3d B_r_12);
    Eigen::Matrix4d joint_to_transform_23(Eigen::Vector3d q, Eigen::Vector3d B_r_23);
    Eigen::Matrix4d joint_to_transform_34(Eigen::Vector3d q, Eigen::Vector3d B_r_34);

    // find base to foot vector
    Eigen::Vector3d find_base_to_foot_vector(Eigen::Vector3d q_one_leg, Eigen::Vector3d B_r_B1);

    // find transformation matrix from I fram to B frame
    Eigen::Matrix4d get_transform_IB(Eigen::VectorXd q, const Eigen::Vector3d body_orientation, const vector<int> stationary_feet);

    // Calculate position jacobian from B frame to End effector
    Eigen::Matrix3Xd joint_to_position_jacobian(Eigen::Vector3d q, Eigen::Vector3d B_r_B1);

public:
    // InverseKinematics class constructor - set robot's dimensions
    InverseKinematics(const double body_dimensions[], const double leg_dimensions[][3], const double base_frame_height, const double distance_between_hip_joints);

    // compute inverse kinematics for a given leg position
    Eigen::VectorXd inverse_kinematics(Eigen::VectorXd q_0, Eigen::Vector3d I_r_IE_des, const vector<int> stationary_feet, const Eigen::Vector3d body_orientation, Eigen::Matrix3d I_C_IE_des);
};