#include <iostream>
#include <cmath>
#include <Eigen/Dense>

// Define a macro for converting degrees to radians
#define DEG2RAD(x) ((x) * M_PI / 180.0)

using namespace std;
using namespace Eigen;

// Function to construct a rotation matrix from roll, pitch, and yaw
Matrix3d constructRotationMatrix(double roll, double pitch, double yaw)
{
    Matrix3d R;
    // double c_r = cos(roll), s_r = sin(roll);
    // double c_p = cos(pitch), s_p = sin(pitch);
    // double c_y = cos(yaw), s_y = sin(yaw);

    // R(0, 0) = c_y * c_p;
    // R(0, 1) = c_y * s_p * s_r - s_y * c_r;
    // R(0, 2) = c_y * s_p * c_r + s_y * s_r;

    // R(1, 0) = s_y * c_p;
    // R(1, 1) = s_y * s_p * s_r + c_y * c_r;
    // R(1, 2) = s_y * s_p * c_r - c_y * s_r;

    // R(2, 0) = -s_p;
    // R(2, 1) = c_p * s_r;
    // R(2, 2) = c_p * c_r;

    Eigen::Matrix3d Rx = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d Ry = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d Rz = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    R = Rz * Ry * Rx;

    return R;
}

// Function to extract roll, pitch, and yaw from a rotation matrix
void extractEulerAngles(const Matrix3d &R, double &roll, double &pitch, double &yaw)
{
    pitch = asin(-R(2, 0));

    if (cos(pitch) > 1e-6)
    { // cos(pitch) != 0
        roll = atan2(R(2, 1), R(2, 2));
        yaw = atan2(R(1, 0), R(0, 0));
    }
    else
    {
        roll = 0; // Gimbal lock: roll and yaw cannot be distinguished
        yaw = atan2(-R(0, 1), R(1, 1));
        std::cout << "Gimbal lock" << std::endl;
        std::cout << "R(2, 1), R(2, 2): " << R(2, 1) << ", " << R(2, 2) << " " << atan2(3.06162, 5.30288)/M_PI*180 << std::endl;
    }

    // roll = atan2(R(2, 1), R(2, 2));
    // pitch = asin(-R(2, 0));
    // yaw = atan2(R(1, 0), R(0, 0));
}

int main()
{
    // Example for pitch = 90 degrees (gimbal lock)
    double roll = DEG2RAD(30);  // 30 degrees
    double pitch = DEG2RAD(90); // 90 degrees
    double yaw = DEG2RAD(45);   // 45 degrees

    // Construct rotation matrix
    Matrix3d R = constructRotationMatrix(roll, pitch, yaw);

    // Extract Euler angles from rotation matrix
    double extractedRoll, extractedPitch, extractedYaw;
    extractEulerAngles(R, extractedRoll, extractedPitch, extractedYaw);

    // Convert extracted angles back to degrees for display
    extractedRoll = extractedRoll * 180.0 / M_PI;
    extractedPitch = extractedPitch * 180.0 / M_PI;
    extractedYaw = extractedYaw * 180.0 / M_PI;

    // Output the results
    cout << "Original Roll: " << roll * 180.0 / M_PI << " degrees" << endl;
    cout << "Original Pitch: " << pitch * 180.0 / M_PI << " degrees" << endl;
    cout << "Original Yaw: " << yaw * 180.0 / M_PI << " degrees" << endl;
    cout << "Extracted Roll: " << extractedRoll << " degrees" << endl;
    cout << "Extracted Pitch: " << extractedPitch << " degrees" << endl;
    cout << "Extracted Yaw: " << extractedYaw << " degrees" << endl;

    return 0;
}
