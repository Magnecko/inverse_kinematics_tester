/*
 *  Transformations.cpp
 *  Author: lnotspotl
 */

#include <math.h>
#include "inverse_kinematics/Transformations.hpp"
#include "inverse_kinematics/inverse_kinematics.hpp"

InverseKinematics::InverseKinematics(const float body_dimensions[], const float leg_dimensions[][3], const float base_frame_height, const float distance_between_hip_joints)
{
    // assigning body dimensions
    body_length_ = body_dimensions[0];
    body_width_ = body_dimensions[1];

    // base frame height
    base_frame_height_ = base_frame_height;

    // leg dimensions
    upper_leg_x_ = leg_dimensions[0][0];
    upper_leg_y_ = leg_dimensions[0][1];
    upper_leg_z_ = leg_dimensions[0][2];
    lower_leg_x_ = leg_dimensions[1][0];
    lower_leg_y_ = leg_dimensions[1][1];
    lower_leg_z_ = leg_dimensions[1][2];

    // distance between hip yaw joint and hip pitch joint
    distance_between_hip_joints_ = distance_between_hip_joints;

    // hip yaw locations
    // FL
    hip_yaw_locations_(0, 0) = body_length_/2;
    hip_yaw_locations_(1, 0) = body_width_/2;
    hip_yaw_locations_(2, 0) = 0;
    // FR
    hip_yaw_locations_(0, 1) = body_length_/2;
    hip_yaw_locations_(1, 1) = -body_width_/2;
    hip_yaw_locations_(2, 1) = 0;
    // BR
    hip_yaw_locations_(0, 2) = -body_length_/2;
    hip_yaw_locations_(1, 2) = -body_width_/2;
    hip_yaw_locations_(2, 2) = 0;
    // BL
    hip_yaw_locations_(0, 3) = -body_length_/2;
    hip_yaw_locations_(1, 3) = body_width_/2;
    hip_yaw_locations_(2, 3) = 0;

    // relative position joint vectors
    // for the moment I define these vectors based on the leg dimension 
    // later I will adjust that so that we can use the information from 
    // TF2
    relative_joint_vectors_ << distance_between_hip_joints, 0, 0,
                               upper_leg_x_, upper_leg_y_, upper_leg_z_,
                               lower_leg_x_, lower_leg_y_, lower_leg_z_;
}

Eigen::Matrix4f InverseKinematics::joint_to_transform_B1(Eigen::Vector3f q, Eigen::Vector3f B_r_B1)
{
    float hip_yaw = q(0);
    float fix_angle = atan2(B_r_B1(1), B_r_B1(0));
    
    float angle = hip_yaw + fix_angle;

    return homog_transform(B_r_B1(0), B_r_B1(1), B_r_B1(2), 0, 0, angle);

}

// transformation matrices for magnecko
// 1 = hip_yaw, 2 = hip_pitch, 3 = knee_pitch, 4 = foot
Eigen::Matrix4f InverseKinematics::joint_to_transform_12(Eigen::Vector3f q, Eigen::Vector3f B_r_12)
{
    float hip_pitch = q(1);
    
    return homog_transform(B_r_12(0), B_r_12(1), B_r_12(2), 0, hip_pitch, 0);
}

Eigen::Matrix4f InverseKinematics::joint_to_transform_23(Eigen::Vector3f q, Eigen::Vector3f B_r_23)
{
    float knee_pitch = q(2);

    return homog_transform(B_r_23(0), B_r_23(1), B_r_23(2), 0, knee_pitch, 0);
}

Eigen::Matrix4f InverseKinematics::joint_to_transform_34(Eigen::Vector3f foot_orientation, Eigen::Vector3f B_r_34)
{
    return homog_transform(B_r_34(0), B_r_34(1), B_r_34(2), foot_orientation(0), foot_orientation(1), foot_orientation(2));
}

Eigen::Vector3f InverseKinematics::find_base_to_foot_vector(Eigen::Vector3f q_one_leg, Eigen::Vector3f B_r_B1)
{
    
    // Homogenous Transformation from B frame to Hip yaw joint frame
    Eigen::Matrix4f T_B1 = InverseKinematics::joint_to_transform_B1(q_one_leg, B_r_B1);

    // Homogeneous transfomation from hip yaw joint frame to hip pitch frame
    Eigen::Vector3f B_r_12(distance_between_hip_joints_, 0, 0);
    Eigen::Matrix4f T_12 = InverseKinematics::joint_to_transform_12(q_one_leg, B_r_12);

    // Homogeneous tranformation from hip pitch joint frame to knee pitch frame
    Eigen::Vector3f B_r_23(upper_leg_x_, upper_leg_y_, upper_leg_z_);
    Eigen::Matrix4f T_23 = InverseKinematics::joint_to_transform_12(q_one_leg, B_r_23);

    // homogeneous transformation from knee pitch joint frame to foot frame
    Eigen::Vector3f B_r_34(lower_leg_x_, lower_leg_y_, lower_leg_z_);
    Eigen::Matrix4f T_34 = InverseKinematics::joint_to_transform_34(q_one_leg, B_r_34);

    // Homogeneous transformation from B to Q = [FL, BL, FR, BR]
    Eigen::Matrix4f T_B4 = T_B1 * T_12 * T_23 * T_34;
    return T_B4(Eigen::seq(0,2), 3);
}

Eigen::Matrix4f InverseKinematics::get_transform_IB(Eigen::VectorXf q, const Eigen::Vector3f body_orientation, const vector<int> stationary_feet)
{
    Eigen::Matrix3f foot_positions = Eigen::Matrix3f::Zero();
    int counter = 0;
    int vector_array_position[4][2] = {
        {0,2},
        {3,5},
        {6,8},
        {9,11},
    };

    // Calculate the position vectors of the feet w.r.t. the base frame
    for (int i = 0; i < 4; i++) 
    {
        if (stationary_feet.at(i) == 0) 
        {
            foot_positions(Eigen::seq(0,2), counter) = find_base_to_foot_vector(q(Eigen::seq(vector_array_position[i][0],vector_array_position[i][1])), hip_yaw_locations_(Eigen::seq(0,2), i));
            counter++;
        }
    }

    // relative vector from B to I frame in B frame coordinates
    Eigen::Vector3f test = foot_positions.block<3,1>(0,0) + foot_positions.block<3,1>(0,1) + foot_positions.block<3,1>(0,2);
    Eigen::Vector3f B_r_BI = 0.333333333 * (foot_positions.block<3,1>(0,0) + foot_positions.block<3,1>(0,1) + foot_positions.block<3,1>(0,2));

    // calculate homogeneous transformation from B to I frame
    Eigen::Matrix4f T_BI = homog_transform(B_r_BI(0), B_r_BI(1), B_r_BI(2), body_orientation(0), body_orientation(1), body_orientation(2));

    // calculate homogeneous transformation from I to B frame
    Eigen::Matrix4f T_IB = T_BI.inverse();

    return T_IB;
}

Eigen::Matrix3Xf InverseKinematics::joint_to_position_jacobian(Eigen::Vector3f q, Eigen::Vector3f B_r_B1)
{
    // relative joint vector positions
    Eigen::Vector3f r_12 = relative_joint_vectors_(Eigen::seq(0,2),0);
    Eigen::Vector3f r_23 = relative_joint_vectors_(Eigen::seq(0,2),1);
    Eigen::Vector3f r_34 = relative_joint_vectors_(Eigen::seq(0,2),2);

    // compute homogeneous transformation matrices
    Eigen::Matrix4f T_B1 = InverseKinematics::joint_to_transform_B1(q, B_r_B1);
    Eigen::Matrix4f T_12 = InverseKinematics::joint_to_transform_12(q, r_12);
    Eigen::Matrix4f T_23 = InverseKinematics::joint_to_transform_23(q, r_23);
    Eigen::Matrix4f T_34 = InverseKinematics::joint_to_transform_34(q, r_34);

    // transformation matrices from B frame to joint
    Eigen::Matrix4f T_B2 = T_B1 * T_12;
    Eigen::Matrix4f T_B3 = T_B2 * T_23;
    Eigen::Matrix4f T_B4 = T_B3 * T_34;

    // Rotation matrices 
    Eigen::Matrix3f R_B1 = T_B1(Eigen::seq(0,2), Eigen::seq(0,2));
    Eigen::Matrix3f R_B2 = T_B2(Eigen::seq(0,2), Eigen::seq(0,2));
    Eigen::Matrix3f R_B3 = T_B3(Eigen::seq(0,2), Eigen::seq(0,2));

    // positions of joint n w.r.t. frame B
    Eigen::Vector3f B_r_B2 = T_B2(Eigen::seq(0,2), 3);
    Eigen::Vector3f B_r_B3 = T_B3(Eigen::seq(0,2), 3);
    Eigen::Vector3f B_r_B4 = T_B4(Eigen::seq(0,2), 3);

    // directions of the rotation axis of joints n w.r.t joint n-1 
    Eigen::Vector3f n_1(0, 0, 1);
    Eigen::Vector3f n_2(0, 1, 0);
    Eigen::Vector3f n_3(0, 1, 0);

    // calculate positional jacobian
    Eigen::Matrix3f B_Jp_Q;
    Eigen::Vector3f v1 = R_B1 * n_1;
    Eigen::Vector3f v2 = B_r_B4 - B_r_B1;
    B_Jp_Q.block<3,1>(0,0) = v1.cross(v2);

    v1 = R_B2 * n_2;
    v2 = B_r_B4 - B_r_B2;
    B_Jp_Q.block<3,1>(0,1) = v1.cross(v2);

    v1 = R_B3 * n_3;
    v2 = B_r_B4 - B_r_B3;
    B_Jp_Q.block<3,1>(0,2) = v1.cross(v2);

    return B_Jp_Q;
}

Eigen::VectorXf InverseKinematics::inverse_kinematics(Eigen::VectorXf q_0, Eigen::Vector3f I_r_IE_des, const vector<int> stationary_feet, const Eigen::Vector3f body_orientation, Eigen::Matrix3f I_C_IE_des = Eigen::Matrix3f::Zero())
{
    // To silence unused parameter error
    I_C_IE_des = Eigen::Matrix3f::Zero();
    
    // Setup
    int max_it = 1; // maximum of iteration

    Eigen::VectorXf q_dot;

    // start configuration
    Eigen::VectorXf q = q_0;
    
    // position error of moving foot
    Eigen::Vector3f dr(0,0,0);
    for (int it = 0; it < max_it; it++)
    {
        // Transformation matrix from I to B
        Eigen::Matrix4f T_IB = InverseKinematics::get_transform_IB(q, body_orientation, stationary_feet);
        Eigen::Matrix3f C_IB = T_IB.block<3,3>(0,0);

        // Calculate all the jacobian needed
        // // FL
        Eigen::Vector3f B_r_BFL = InverseKinematics::find_base_to_foot_vector(q(Eigen::seq(0,2)), hip_yaw_locations_(Eigen::seq(0,2), 0));
        Eigen::Matrix3Xf B_Jp_FL = InverseKinematics::joint_to_position_jacobian(q(Eigen::seq(0,2)), hip_yaw_locations_(Eigen::seq(0,2), 0));
        Eigen::Matrix3Xf B_Jp_FL_resized = Eigen::Matrix<float, 3, 12>::Zero();
        B_Jp_FL_resized.block<3,3>(0,0) = B_Jp_FL;
        B_Jp_FL_resized.block<3,9>(0,3) = Eigen::Matrix<float, 3, 9>::Zero();

        // // FR
        Eigen::Vector3f B_r_BFR = InverseKinematics::find_base_to_foot_vector(q(Eigen::seq(3,5)), hip_yaw_locations_(Eigen::seq(0,2), 1));
        Eigen::Matrix3Xf B_Jp_FR = InverseKinematics::joint_to_position_jacobian(q(Eigen::seq(3,5)), hip_yaw_locations_(Eigen::seq(0,2), 1));
        Eigen::Matrix3Xf B_Jp_FR_resized = Eigen::Matrix<float, 3, 12>::Zero();
        B_Jp_FR_resized.block<3,3>(0,0) = Eigen::Matrix3f::Zero();
        B_Jp_FR_resized.block<3,3>(0,3) = B_Jp_FR;
        B_Jp_FR_resized.block<3,6>(0,6) = Eigen::Matrix<float, 3, 6>::Zero();

        // // BR
        Eigen::Vector3f B_r_BBR = InverseKinematics::find_base_to_foot_vector(q(Eigen::seq(6,8)), hip_yaw_locations_(Eigen::seq(0,2), 2));
        Eigen::Matrix3Xf B_Jp_BR = InverseKinematics::joint_to_position_jacobian(q(Eigen::seq(6,8)), hip_yaw_locations_(Eigen::seq(0,2), 2));
        Eigen::Matrix3Xf B_Jp_BR_resized = Eigen::Matrix<float, 3, 12>::Zero();
        B_Jp_BR_resized.block<3,6>(0,0) = Eigen::Matrix<float, 3, 6>::Zero();
        B_Jp_BR_resized.block<3,3>(0,6) = B_Jp_BR;
        B_Jp_BR_resized.block<3,3>(0,9) = Eigen::Matrix3f::Zero();

        // // BL
        Eigen::Vector3f B_r_BBL = InverseKinematics::find_base_to_foot_vector(q(Eigen::seq(9,11)), hip_yaw_locations_(Eigen::seq(0,2), 3));
        Eigen::Matrix3Xf B_Jp_BL = InverseKinematics::joint_to_position_jacobian(q(Eigen::seq(9,11)), hip_yaw_locations_(Eigen::seq(0,2), 3));
        Eigen::Matrix3Xf B_Jp_BL_resized = Eigen::Matrix<float, 3, 12>::Zero();
        B_Jp_BL_resized.block<3,9>(0,0) = Eigen::Matrix<float, 3, 9>::Zero();
        B_Jp_BL_resized.block<3,3>(0,9) = B_Jp_BL;

        // Find I_Jp
        // // For FL
        Eigen::Matrix3Xf I_Jp_FL = Eigen::Matrix<float, 3, 18>::Zero();
        I_Jp_FL.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
        I_Jp_FL.block<3,3>(0,3) = -C_IB*skew_matrix(B_r_BFL);
        I_Jp_FL.block<3,12>(0,6) = C_IB*B_Jp_FL_resized;

        // // For FR
        Eigen::Matrix3Xf I_Jp_FR = Eigen::Matrix<float, 3, 18>::Zero();
        I_Jp_FR.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
        I_Jp_FR.block<3,3>(0,3) = -C_IB*skew_matrix(B_r_BFR);
        I_Jp_FR.block<3,12>(0,6) = C_IB*B_Jp_FR_resized;
        
        // // For BR
        Eigen::Matrix3Xf I_Jp_BR = Eigen::Matrix<float, 3, 18>::Zero();
        I_Jp_BR.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
        I_Jp_BR.block<3,3>(0,3) = -C_IB*skew_matrix(B_r_BBR);
        I_Jp_BR.block<3,12>(0,6) = C_IB*B_Jp_BR_resized;

        // // For BL
        Eigen::Matrix3Xf I_Jp_BL = Eigen::Matrix<float, 3, 18>::Zero();
        I_Jp_BL.block<3,3>(0,0) = Eigen::Matrix3f::Identity();
        I_Jp_BL.block<3,3>(0,3) = -C_IB*skew_matrix(B_r_BBL);
        I_Jp_BL.block<3,12>(0,6) = C_IB*B_Jp_BL_resized;

        // // For the base
        Eigen::Matrix3Xf I_Jp_B = Eigen::Matrix<float, 3, 18>::Zero();
        I_Jp_B.block<3,15>(0,0) = Eigen::Matrix<float, 3, 15>::Zero();
        I_Jp_B.block<3,3>(0,15) = Eigen::Matrix3f::Identity();
        
        Eigen::MatrixXf I_Jp = Eigen::Matrix<float, 12, 18>::Zero();
        I_Jp.block<3,18>(0,0) = I_Jp_FL;
        I_Jp.block<3,18>(3,0) = I_Jp_FR;
        I_Jp.block<3,18>(6,0) = I_Jp_BR;
        I_Jp.block<3,18>(9,0) = I_Jp_BL;

        // Position error 
        Eigen::Vector3f I_r_IE;
        Eigen::VectorXf dr = Eigen::Vector<float, 12>::Zero();

        if (stationary_feet.at(0) == 1)
        {
            I_r_IE = find_base_to_foot_vector(q(Eigen::seq(0,2)), hip_yaw_locations_.block<3,1>(0,0));
            dr.head(3) = I_r_IE_des - I_r_IE;
        }
        else if (stationary_feet.at(1) == 1)
        {
            I_r_IE = find_base_to_foot_vector(q.segment(3,5), hip_yaw_locations_.block<3,1>(0,1));
            dr.segment(3,5) = I_r_IE_des - I_r_IE;
        }
        else if (stationary_feet.at(2) == 1)
        {
            I_r_IE = find_base_to_foot_vector(q(Eigen::seq(5,8)), hip_yaw_locations_.block<3,1>(0,2));
            dr.segment(6,8) = I_r_IE_des - I_r_IE;
        }
        else if (stationary_feet.at(3) == 1)
        {
            I_r_IE = find_base_to_foot_vector(q(Eigen::seq(9,11)), hip_yaw_locations_.block<3,1>(0,3));
            dr.tail(3) = I_r_IE_des - I_r_IE;
        }
        std::cout << dr << std::endl;
        // Kinematic controller
        // // Controller gain
        float kp = 5;

        // // Desired end effector velocity of moving leg
        Eigen::VectorXf I_v_IE_des = Eigen::Matrix<float, 12, 1>::Zero();

        // // Pseudo Inverse of I_Jp
        Eigen::MatrixXf I_Jp_pinv = I_Jp.completeOrthogonalDecomposition().pseudoInverse();
        // // Null space projection 
        Eigen::MatrixXf N = Eigen::Matrix<float, 18, 18>::Identity() - I_Jp_pinv*I_Jp;

        // // Controlled End effector velocity
        Eigen::VectorXf I_v_command = Eigen::Vector<float, 12>::Zero();
        I_v_command = I_v_IE_des + kp*dr;
        Eigen::MatrixXf temp = I_Jp_B * N;

        q_dot = I_Jp_pinv * I_v_command + N * temp.completeOrthogonalDecomposition().pseudoInverse() * (-1*I_Jp_B*I_Jp_pinv * I_v_command);
    }

    return q_dot;
}