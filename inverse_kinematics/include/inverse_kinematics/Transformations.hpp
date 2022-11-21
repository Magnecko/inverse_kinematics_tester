/*
 *  Transformations.hpp
 *  Author: Frigi
 */

#pragma once
#include <eigen3/Eigen/Core>

#define PI 3.1415926535897

// Quaternion struct 
struct Quaternion 
{
        double w, x, y, z;
};

// Euler angles struct
struct EulerAngles
{
        double roll, pitch, yaw;
};

// Rotation around the x axis
Eigen::Matrix3d rotx(double alpha);

// Rotation around the y axis
Eigen::Matrix3d roty(double beta);

// Rotation around the z axis
Eigen::Matrix3d rotz(double gamma);

// Rotation around the x axis -> y axis -> z axis
Eigen::Matrix3d rotxyz(double alpha, double beta, double gamma);

// Transformation along the x, y and z axis
Eigen::Matrix4d homog_transxyz(double dx, double dy, double dz);

// 4x4 general transformation matrix
Eigen::Matrix4d homog_transform(double dx, double dy, double dz, 
        double alpha, double beta, double gamma);

// Inverse od a general 4x4 transdormation matrix
Eigen::Matrix4d homog_transform_inverse(Eigen::Matrix4d matrix);

// Implement quaternion to euler angle function
EulerAngles toEulerAngles(Quaternion q);

// Euler angles to quaternion
Quaternion toQuaternion(EulerAngles e);

// Calculate skew matrix
Eigen::Matrix3d skew_matrix(Eigen::Vector3d B_r_BQ);