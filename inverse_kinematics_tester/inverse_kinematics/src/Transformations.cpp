/*
 *  Transformations.cpp
 *  Author: lnotspotl
 */


#include <eigen3/Eigen/Geometry>
#include "inverse_kinematics/Transformations.hpp"

///////////////////////////////////////////////////////////////////////////////
// X-Euler rotation
Eigen::Matrix3f rotx(float alpha)
{
    Eigen::Matrix3f x_rotation_matrix;
    x_rotation_matrix = Eigen::AngleAxisf(alpha, Eigen::Vector3f::UnitX());
    return x_rotation_matrix;
}

///////////////////////////////////////////////////////////////////////////////
// Y-Euler rotation
Eigen::Matrix3f roty(float beta)
{
    Eigen::Matrix3f y_rotation_matrix;
    y_rotation_matrix = Eigen::AngleAxisf(beta, Eigen::Vector3f::UnitY());
    return y_rotation_matrix;
}

///////////////////////////////////////////////////////////////////////////////
// Z-Euler rotation
Eigen::Matrix3f rotz(float gamma)
{
    Eigen::Matrix3f z_rotation_matrix;
    z_rotation_matrix = Eigen::AngleAxisf(gamma, Eigen::Vector3f::UnitZ());
    return z_rotation_matrix;
}

///////////////////////////////////////////////////////////////////////////////
// XYZ-Euler Rotation
Eigen::Matrix3f rotxyz(float alpha, float beta, float gamma)
{
    Eigen::Matrix3f xyz_rotation_matrix;
    xyz_rotation_matrix = rotx(alpha) * roty(beta) * rotz(gamma);
    return xyz_rotation_matrix;
}

///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f homog_transxyz(float dx, float dy, float dz)
{
    Eigen::Matrix4f translation_matrix = Eigen::Matrix<float, 4, 4>::Identity();
    translation_matrix(0,3) = dx;
    translation_matrix(1,3) = dy;
    translation_matrix(2,3) = dz;
    return translation_matrix;
}

///////////////////////////////////////////////////////////////////////////////
///////////// Function to get the Current Transformationmatrixes //////////////
///////////////////////////////////////////////////////////////////////////////
Eigen::Matrix4f homog_transform(float dx, float dy, float dz, 
        float alpha, float beta, float gamma)
{
    Eigen::Matrix4f transformation_matrix = homog_transxyz(dx, dy, dz);
    transformation_matrix.block<3,3>(0,0) = rotxyz(alpha, beta, gamma);
    return transformation_matrix;
}

// Turn Quaternion to euler angles 
EulerAngles toEulerAngles(Quaternion q)
{
    EulerAngles angles;

    // roll (x-axis rotation)
    double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
    double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    angles.roll = atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2 * (q.w * q.y - q.z * q.x);
    if (std::abs(sinp) >= 1)
        angles.pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        angles.pitch = std::asin(sinp);

    // yaw (z-axis rotation)
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    angles.yaw = atan2(siny_cosp, cosy_cosp);

    return angles;
}

// Turn euler angles to euler
Quaternion toQuaternion(EulerAngles e)
{
    double cr = cos(e.roll * 0.5);
    double sr = sin(e.roll * 0.5);
    double cp = cos(e.pitch * 0.5);
    double sp = sin(e.pitch * 0.5);
    double cy = cos(e.yaw * 0.5);
    double sy = sin(e.yaw * 0.5);

    Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;

    return q;
}

// calculate skew matrix
Eigen::Matrix3f skew_matrix(Eigen::Vector3f B_r_BQ)
{
    float wx = B_r_BQ(0);
    float wy = B_r_BQ(1);
    float wz = B_r_BQ(2);

    Eigen::Matrix3f skew;
    skew << 0, -wz, wy,
            wz, 0, -wx,
            -wy, wx, 0;

    return skew;
}