//
// Created by hyj on 18-1-19.
//

#include <random>
#include "imu.h"
#include "utilities.h"

// euler2Rotation:   body frame to target frame
Eigen::Matrix3d euler2Rotation( Eigen::Vector3d  eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);
    double yaw = eulerAngles(2);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
    double cy = cos(yaw); double sy = sin(yaw);

    Eigen::Matrix3d Rtb;
    Rtb<< cy*cp ,   cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,
            sy*cp,    cy *cr + sy*sr*sp,  sp*sy*cr - cy*sr,
            -sp,         cp*sr,           cp*cr;
    return Rtb;
}

Eigen::Matrix3d eulerRates2bodyRates(Eigen::Vector3d eulerAngles)
{
    double roll = eulerAngles(0);
    double pitch = eulerAngles(1);

    double cr = cos(roll); double sr = sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);

    Eigen::Matrix3d R;
    R<<  1,   0,    -sp,
            0,   cr,   sr*cp,
            0,   -sr,  cr*cp;

    return R;
}


IMU::IMU(Param p): param_(p)
{
    gyro_bias_ = Eigen::Vector3d::Zero();
    acc_bias_ = Eigen::Vector3d::Zero();
}

void IMU::addIMUnoise(MotionData& data)
{
    std::random_device rd;
    std::default_random_engine generator_(rd());
    std::normal_distribution<double> noise(0.0, 1.0);

    Eigen::Vector3d noise_gyro(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d gyro_sqrt_cov = param_.gyro_noise_sigma * Eigen::Matrix3d::Identity();
    data.imu_gyro = data.imu_gyro + gyro_sqrt_cov * noise_gyro / sqrt( param_.imu_timestep ) + gyro_bias_;

    Eigen::Vector3d noise_acc(noise(generator_),noise(generator_),noise(generator_));
    Eigen::Matrix3d acc_sqrt_cov = param_.acc_noise_sigma * Eigen::Matrix3d::Identity();
    data.imu_acc = data.imu_acc + acc_sqrt_cov * noise_acc / sqrt( param_.imu_timestep ) + acc_bias_;

    // gyro_bias update
    Eigen::Vector3d noise_gyro_bias(noise(generator_),noise(generator_),noise(generator_));
    gyro_bias_ += param_.gyro_bias_sigma * sqrt(param_.imu_timestep ) * noise_gyro_bias;
    data.imu_gyro_bias = gyro_bias_;

    // acc_bias update
    Eigen::Vector3d noise_acc_bias(noise(generator_),noise(generator_),noise(generator_));
    acc_bias_ += param_.acc_bias_sigma * sqrt(param_.imu_timestep ) * noise_acc_bias;
    data.imu_acc_bias = acc_bias_;

}

MotionData IMU::MotionModelSinShape(double t, double time_offset)
{

    MotionData data;
    // param
    double ellipse_x = 15.;
    double ellipse_y = 20.;
    double z = 1.;         // z轴做sin运动 Amplitude of the sinusoidal motion in the z direction.
    double K1 = 10.;       // z轴的正弦频率是x，y的k1倍
    double K = M_PI / 10.; // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周
    double K2 = K * K;

    // translation
    // twb:  body frame in world frame
    // Eigen::Vector3d position(ellipse_x * cos(K * t) + 5, ellipse_y * sin(K * t) + 5, z * sin(K1 * K * t) + 5);
    // Eigen::Vector3d dp(-K * ellipse_x * sin(K * t), K * ellipse_y * cos(K * t), z * K1 * K * cos(K1 * K * t)); // position导数　in world frame
    // Eigen::Vector3d ddp(-K2 * ellipse_x * cos(K * t), -K2 * ellipse_y * sin(K * t), -z * K1 * K1 * K2 * sin(K1 * K * t)); // position二阶导数
    Eigen::Vector3d position(ellipse_x * cos(K * t) + 5, ellipse_y * cos(K * t) + 5, z * cos(K1 * K * t) + 5);
    Eigen::Vector3d dp(-K * ellipse_x * sin(K * t), -K * ellipse_y * sin(K * t), -z * K1 * K * sin(K1 * K * t));          // position导数　in world frame
    Eigen::Vector3d ddp(-K2 * ellipse_x * cos(K * t), -K2 * ellipse_y * cos(K * t), -z * K1 * K1 * K2 * cos(K1 * K * t)); // position二阶导数

    // Rotation
    double k_roll = 0.1;
    double k_pitch = 0.2;
    double k_yaw = 0.3;
    // k_roll = 0;
    // k_pitch = 0; // this will make the integration without noise more accurate.
    // K = 0;
    // Eigen::Vector3d eulerAngles(k_roll * cos(t), k_pitch * sin(t), K * t);          // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    // Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t) , k_pitch * cos(t) , K);      // euler angles 的导数
    Eigen::Vector3d eulerAngles(k_roll * sin(t), k_pitch * sin(t), K * t);  // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    Eigen::Vector3d eulerAnglesRates(k_roll * cos(t), k_pitch * cos(t), K); // euler angles 的导数

    // Create individual rotation matrices
    double roll = eulerAngles(0);  // phi
    double pitch = eulerAngles(1); // theta
    double yaw = eulerAngles(2);   // psi
    Eigen::Matrix3d R_roll = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
    Eigen::Matrix3d R_pitch = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
    Eigen::Matrix3d R_yaw = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();

    Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles); // body frame to world frame
    // Rwb = R_yaw * R_pitch * R_roll;

    // Combine the rotation matrices (roll * pitch * yaw)
    Eigen::Matrix3d R = R_yaw * R_pitch * R_roll;

    Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates; //  euler rates trans to body gyro

    Eigen::Vector3d gn(0, 0, -9.81);                                 //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    Eigen::Vector3d imu_acc = Rwb.matrix().transpose() * (ddp - gn); //  Rbw * Rwn * gn = gs

    data.imu_gyro = imu_gyro;
    data.imu_acc = imu_acc;
    // data.Rwb = Rwb.matrix();
    data.Rwb = Rwb;
    data.twb = position;
    data.imu_velocity = dp;
    data.timestamp = t + time_offset;
    // Rwb = Rwb * SO3::exp(imu_gyro * param_.imu_timestep);

    return data;
}

MotionData IMU::MotionModel(double t, double time_offset)
{

    MotionData data;
    // param
    double ellipse_x = 15.;
    double ellipse_y = 20.;
    double z = 1.;           // z轴做sin运动 Amplitude of the sinusoidal motion in the z direction.
    double K1 = 10.;         // z轴的正弦频率是x，y的k1倍
    double K = M_PI / 10.;   // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周
    double K2 = K * K;

    // translation
    // twb:  body frame in world frame
    // Eigen::Vector3d position(ellipse_x * cos(K * t) + 5, ellipse_y * sin(K * t) + 5, z * sin(K1 * K * t) + 5);
    // Eigen::Vector3d dp(-K * ellipse_x * sin(K * t), K * ellipse_y * cos(K * t), z * K1 * K * cos(K1 * K * t)); // position导数　in world frame
    // Eigen::Vector3d ddp(-K2 * ellipse_x * cos(K * t), -K2 * ellipse_y * sin(K * t), -z * K1 * K1 * K2 * sin(K1 * K * t)); // position二阶导数
    // Eigen::Vector3d position(ellipse_x * cos(K * t) + 5, ellipse_y * cos(K * t) + 5, z * cos(K1 * K * t) + 5);
    // Eigen::Vector3d dp(-K * ellipse_x * sin(K * t), -K * ellipse_y * sin(K * t), -z * K1 * K * sin(K1 * K * t)); // position导数　in world frame
    // Eigen::Vector3d ddp(-K2 * ellipse_x * cos(K * t), -K2 * ellipse_y * cos(K * t), -z * K1 * K1 * K2 * cos(K1 * K * t)); // position二阶导数
    Eigen::Vector3d position(ellipse_x * (K * t) + 5, ellipse_y * cos(K * t) + 5, z * cos(K1 * K * t) + 5);
    Eigen::Vector3d dp(ellipse_x * K, -K * ellipse_y * sin(K * t), -z * K1 * K * sin(K1 * K * t));          // position导数　in world frame
    Eigen::Vector3d ddp(0, -K2 * ellipse_y * cos(K * t), -z * K1 * K1 * K2 * cos(K1 * K * t)); // position二阶导数

    // Rotation
    double k_roll = 0.1;
    double k_pitch = 0.2;
    double k_yaw = 0.3;
    // k_roll = 0;
    // k_pitch = 0; // this will make the integration without noise more accurate.
    // K = 0;
    // Eigen::Vector3d eulerAngles(k_roll * cos(t), k_pitch * sin(t), K * t);          // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    // Eigen::Vector3d eulerAnglesRates(-k_roll * sin(t) , k_pitch * cos(t) , K);      // euler angles 的导数
    Eigen::Vector3d eulerAngles(k_roll * sin(t), k_pitch * sin(t), K * t);         // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    Eigen::Vector3d eulerAnglesRates(k_roll * cos(t), k_pitch * cos(t), K);        // euler angles 的导数

    Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles);         // body frame to world frame
    // Rwb = R_yaw * R_pitch * R_roll;

    Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates; //  euler rates trans to body gyro

    Eigen::Vector3d gn (0,0,-9.81);                                   //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    Eigen::Vector3d imu_acc = Rwb.matrix().transpose() * ( ddp -  gn );  //  Rbw * Rwn * gn = gs

    data.imu_gyro = imu_gyro;
    data.imu_acc = imu_acc;
    // data.Rwb = Rwb.matrix();
    data.Rwb = Rwb;
    data.twb = position;
    data.imu_velocity = dp;
    data.timestamp = t + time_offset;
    // Rwb = Rwb * SO3::exp(imu_gyro * param_.imu_timestep);

    return data;

}

MotionData IMU::MotionModelSO3(double t, double time_offset)
{
    MotionData data;
    // param
    double ellipse_x = 15.;
    double ellipse_y = 20.;
    double z = 1.;         // z轴做sin运动 Amplitude of the sinusoidal motion in the z direction.
    double K1 = 10.;       // z轴的正弦频率是x，y的k1倍
    double K = M_PI / 10.; // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周
    double K2 = K * K;

    // translation
    // twb:  body frame in world frame
    // Eigen::Vector3d position(ellipse_x * cos(K * t) + 5, ellipse_y * sin(K * t) + 5, z * sin(K1 * K * t) + 5);
    // Eigen::Vector3d dp(-K * ellipse_x * sin(K * t), K * ellipse_y * cos(K * t), z * K1 * K * cos(K1 * K * t)); // position导数　in world frame
    // Eigen::Vector3d ddp(-K2 * ellipse_x * cos(K * t), -K2 * ellipse_y * sin(K * t), -z * K1 * K1 * K2 * sin(K1 * K * t)); // position二阶导数
    // Eigen::Vector3d position(ellipse_x * cos(K * t) + 5, ellipse_y * cos(K * t) + 5, z * cos(K1 * K * t) + 5);
    // Eigen::Vector3d dp(-K * ellipse_x * sin(K * t), -K * ellipse_y * sin(K * t), -z * K1 * K * sin(K1 * K * t)); // position导数　in world frame
    // Eigen::Vector3d ddp(-K2 * ellipse_x * cos(K * t), -K2 * ellipse_y * cos(K * t), -z * K1 * K1 * K2 * cos(K1 * K * t)); // position二阶导数
    Eigen::Vector3d position(ellipse_x * (K * t) + 5, ellipse_y * cos(K * t) + 5, z * cos(K1 * K * t) + 5);
    Eigen::Vector3d dp(ellipse_x * K, -K * ellipse_y * sin(K * t), -z * K1 * K * sin(K1 * K * t)); // position导数　in world frame
    Eigen::Vector3d ddp(0, -K2 * ellipse_y * cos(K * t), -z * K1 * K1 * K2 * cos(K1 * K * t));     // position二阶导数

    // Rotation
    double k_roll = 0.1;
    double k_pitch = 0.2;
    double k_yaw = 0.3;

    static SO3 Rwb;

    Eigen::Vector3d imu_gyro(k_roll, k_pitch, k_yaw); // euler rates
    
    Eigen::Vector3d gn(0, 0, -9.81);                                 //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    Eigen::Vector3d imu_acc = Rwb.matrix().transpose() * (ddp - gn); //  Rbw * Rwn * gn = gs

    data.imu_gyro = imu_gyro;
    data.imu_acc = imu_acc;
    data.Rwb = Rwb.matrix();
    // data.Rwb = Rwb;
    data.twb = position;
    data.imu_velocity = dp;
    data.timestamp = t + time_offset;
    Rwb = Rwb * SO3::exp(imu_gyro * param_.imu_timestep);

    return data;
}

MotionData IMU::MotionModelStraightLine(double t, double dt, double time_offset)
{
    MotionData data;
    // param
    Eigen::Vector3d acc_world(0.05, 0.04, 0.03);
    Eigen::Vector3d gyro_body(0.1, 0.3, 0.4);

    Eigen::Vector3d gn(0, 0, -9.81);                                 //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    Eigen::Vector3d imu_acc = Rwb_.matrix().transpose() * (acc_world - gn); //  Rbw * Rwn * gn = gs

    // posi = posi + vel * param_.imu_timestep + 0.5 * acc_world * param_.imu_timestep * param_.imu_timestep;
    // vel = vel + acc_world * param_.imu_timestep;
    // posi = posi + vel * param_.imu_timestep + 0.5 * Rwb.matrix() * imu_acc * param_.imu_timestep * param_.imu_timestep;
    Eigen::Vector3d acc_w = Rwb_.matrix() * imu_acc + gn; // aw = Rwb * ( acc_body - acc_bias ) + gw
    position_ += velocity_ * dt + 0.5 * acc_w * dt * dt;
    std::cout << "data: " << position_.transpose() << " " << velocity_.transpose() << " " << acc_w.transpose() << std::endl;
    velocity_ += acc_w * dt;

    // Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw; // aw = Rwb * ( acc_body - acc_bias ) + gw
    // Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
    // Vw = Vw + acc_w * dt;

    data.imu_gyro = gyro_body;
    data.imu_acc = imu_acc;
    data.Rwb = Rwb_.matrix();
    data.twb = position_;
    std::cout << "position_1 = " << data.twb.transpose() << std::endl;
    std::cout << "position_2 = " << data.twb.transpose() << std::endl;
    data.imu_velocity = velocity_;
    data.timestamp = t + time_offset;
    Rwb_ = Rwb_ * SO3::exp(gyro_body * dt);

    return data;
}

MotionData IMU::StaticMotionModel(double t, double time_offset)
{
    MotionData data;
    // param
    double ellipse_x = 0.;
    double ellipse_y = 0.;
    double z = 0;         // z轴做sin运动 Amplitude of the sinusoidal motion in the z direction.
    // double ellipse_y = 20.;
    // double z = 1.;
    double K1 = 10.;       // z轴的正弦频率是x，y的k1倍
    double K = M_PI / 10.; // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周
    double K2 = K * K;

    // translation
    // twb:  body frame in world frame
    // Eigen::Vector3d position(ellipse_x + 5, ellipse_y + 5, z + 5);
    Eigen::Vector3d position(ellipse_x, ellipse_y, z);
    Eigen::Vector3d dp(0, 0, 0);            // position导数　in world frame
    Eigen::Vector3d ddp(0, 0, 0); // position二阶导数


    // Rotation
    double k_roll = 0.1;
    double k_pitch = 0.2;
    // k_roll = 0;
    // k_pitch = 0; // this will make the integration without noise more accurate.
    // K = 0;
    Eigen::Vector3d eulerAngles(0, 0, 0); // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    Eigen::Vector3d eulerAnglesRates(0, 0, 0); // euler angles 的导数

    Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles); // body frame to world frame
    // Rwb = Eigen::AngleAxisd(eulerAngles[2], Eigen::Vector3d::UnitZ()) *
    //       Eigen::AngleAxisd(eulerAngles[1], Eigen::Vector3d::UnitY()) *
    //       Eigen::AngleAxisd(eulerAngles[0], Eigen::Vector3d::UnitX());
    Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates; //  euler rates trans to body gyro

    Eigen::Vector3d gn(0, 0, -9.81);                        //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    Eigen::Vector3d imu_acc = Rwb.transpose() * (ddp - gn); //  Rbw * Rwn * gn = gs

    data.imu_gyro = imu_gyro;
    data.imu_acc = imu_acc;
    data.Rwb = Rwb;
    data.twb = position;
    data.imu_velocity = dp;
    data.timestamp = t + time_offset;
    return data;
}

MotionData IMU::StaticMotionModelSinShape(double t, double time_offset)
{
    MotionData data;
    // param
    double ellipse_x = 15.;
    double ellipse_y = 20.;
    double z = 1; // z轴做sin运动 Amplitude of the sinusoidal motion in the z direction.
    // double ellipse_y = 20.;
    // double z = 1.;
    double K1 = 10.;       // z轴的正弦频率是x，y的k1倍
    double K = M_PI / 10.; // 20 * K = 2pi 　　由于我们采取的是时间是20s, 系数K控制yaw正好旋转一圈，运动一周
    double K2 = K * K;

    // translation
    // twb:  body frame in world frame
    // Eigen::Vector3d position(ellipse_x + 5, ellipse_y + 5, z + 5);
    Eigen::Vector3d position(ellipse_x + 5, ellipse_y + 5, z + 5);
    Eigen::Vector3d dp(0, 0, 0);  // position导数　in world frame
    Eigen::Vector3d ddp(0, 0, 0); // position二阶导数

    // Rotation
    double k_roll = 0.1;
    double k_pitch = 0.2;
    // k_roll = 0;
    // k_pitch = 0; // this will make the integration without noise more accurate.
    // K = 0;
    Eigen::Vector3d eulerAngles(0, 0, 0);      // roll ~ [-0.2, 0.2], pitch ~ [-0.3, 0.3], yaw ~ [0,2pi]
    Eigen::Vector3d eulerAnglesRates(0, 0, 0); // euler angles 的导数

    Eigen::Matrix3d Rwb = euler2Rotation(eulerAngles); // body frame to world frame
    // Rwb = Eigen::AngleAxisd(eulerAngles[2], Eigen::Vector3d::UnitZ()) *
    //       Eigen::AngleAxisd(eulerAngles[1], Eigen::Vector3d::UnitY()) *
    //       Eigen::AngleAxisd(eulerAngles[0], Eigen::Vector3d::UnitX());
    Eigen::Vector3d imu_gyro = eulerRates2bodyRates(eulerAngles) * eulerAnglesRates; //  euler rates trans to body gyro

    Eigen::Vector3d gn(0, 0, -9.81);                        //  gravity in navigation frame(ENU)   ENU (0,0,-9.81)  NED(0,0,9,81)
    Eigen::Vector3d imu_acc = Rwb.transpose() * (ddp - gn); //  Rbw * Rwn * gn = gs

    data.imu_gyro = imu_gyro;
    data.imu_acc = imu_acc;
    data.Rwb = Rwb;
    data.twb = position;
    data.imu_velocity = dp;
    data.timestamp = t + time_offset;
    return data;
}

//读取生成的imu数据并用imu动力学模型对数据进行计算，最后保存imu积分以后的轨迹，
//用来验证数据以及模型的有效性。
void IMU::testImu(std::string src, std::string dist)
{
    std::vector<MotionData>imudata;
    LoadPose(src,imudata); // load IMU data

    std::ofstream save_points;
    save_points.open(dist);

    double dt = param_.imu_timestep;
    Eigen::Vector3d Pwb = init_twb_;              // position :    from  imu measurements
    Eigen::Quaterniond Qwb(init_Rwb_);            // quaterniond:  from imu measurements
    Eigen::Vector3d Vw = init_velocity_;          // velocity  :   from imu measurements
    Eigen::Vector3d gw(0,0,-9.81);    // ENU frame
    Eigen::Vector3d temp_a;
    Eigen::Vector3d theta;
    for (int i = 1; i < imudata.size(); ++i) {

        MotionData imupose = imudata[i];

        //delta_q = [1 , 1/2 * thetax , 1/2 * theta_y, 1/2 * theta_z]
        Eigen::Quaterniond dq;
        Eigen::Vector3d dtheta_half =  imupose.imu_gyro * dt /2.0;
        dq.w() = 1;
        dq.x() = dtheta_half.x();
        dq.y() = dtheta_half.y();
        dq.z() = dtheta_half.z();
        dq.normalize();

        // // Compute the angle of rotation
        // double theta = imupose.imu_gyro.norm() * dt;

        // // Normalize the rotation axis
        // Eigen::Vector3d u = imupose.imu_gyro.normalized();

        // // Compute half angle trigonometric functions
        // double half_theta = theta / 2.0;
        // double cos_half_theta = std::cos(half_theta);
        // double sin_half_theta = std::sin(half_theta);

        // // Construct the quaternion
        // Eigen::Quaterniond dq;
        // dq.w() = cos_half_theta;
        // dq.x() = u.x() * sin_half_theta;
        // dq.y() = u.y() * sin_half_theta;
        // dq.z() = u.z() * sin_half_theta;
        // dq.normalize();

        /// imu 动力学模型 欧拉积分
        Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;  // aw = Rwb * ( acc_body - acc_bias ) + gw

        // // Midpoint integration for better accuracy
        // Eigen::Vector3d acc_w1 = Qwb * (imupose.imu_acc) + gw;
        // Eigen::Quaterniond Qwb_mid = Qwb * dq;
        // Eigen::Vector3d acc_w2 = Qwb_mid * (imupose.imu_acc) + gw;
        // Eigen::Vector3d acc_w = (acc_w1 + acc_w2) / 2.0;

        // // Endpoint integration for better accuracy
        // Eigen::Vector3d acc_w_prev = Qwb * (imudata[i - 1].imu_acc) + gw;
        // Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw;

        // // Average acceleration
        // Eigen::Vector3d acc_w_avg = (acc_w_prev + acc_w) / 2.0;

        Qwb = Qwb * dq;
        Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
        Vw = Vw + acc_w * dt;
        
        /// 中值积分

        //　按着imu postion, imu quaternion , cam postion, cam quaternion 的格式存储，由于没有cam，所以imu存了两次
        save_points << std::setprecision(5) 
                    << imupose.timestamp << " "
                    << std::setprecision(10)
                    << Qwb.w() << " "
                    << Qwb.x() << " "
                    << Qwb.y() << " "
                    << Qwb.z() << " "
                    << Pwb(0) << " "
                    << Pwb(1) << " "
                    << Pwb(2) << " "
                    << Qwb.w() << " "
                    << Qwb.x() << " "
                    << Qwb.y() << " "
                    << Qwb.z() << " "
                    << Pwb(0) << " "
                    << Pwb(1) << " "
                    << Pwb(2) << " "
                    << std::endl;
    }

    std::cout<<"test end"<<std::endl;

}

// void IMU::testImu(std::string src, std::string dist)
// {
//     std::vector<MotionData> imudata;
//     LoadPose(src, imudata); // load IMU data

//     std::ofstream save_points;
//     save_points.open(dist);

//     double dt = param_.imu_timestep;
//     Eigen::Vector3d Pwb = init_twb_;     // position :    from  imu measurements
//     Eigen::Quaterniond Qwb(init_Rwb_);   // quaterniond:  from imu measurements
//     Eigen::Vector3d Vw = init_velocity_; // velocity  :   from imu measurements
//     Eigen::Vector3d gw(0, 0, -9.81);     // ENU frame

//     for (int i = 1; i < imudata.size(); ++i)
//     {
//         MotionData imupose = imudata[i];

//         // Compute the angle of rotation
//         double theta = imupose.imu_gyro.norm() * dt;

//         // Normalize the rotation axis
//         Eigen::Vector3d u = imupose.imu_gyro.normalized();

//         // Compute half angle trigonometric functions
//         double half_theta = theta / 2.0;
//         double cos_half_theta = std::cos(half_theta);
//         double sin_half_theta = std::sin(half_theta);

//         // Construct the quaternion
//         Eigen::Quaterniond dq;
//         dq.w() = cos_half_theta;
//         dq.x() = u.x() * sin_half_theta;
//         dq.y() = u.y() * sin_half_theta;
//         dq.z() = u.z() * sin_half_theta;
//         dq.normalize();

//         // IMU dynamics model Euler integration
//         Eigen::Vector3d acc_w = Qwb * (imupose.imu_acc) + gw; // aw = Rwb * ( acc_body - acc_bias ) + gw
//         Qwb = Qwb * dq;
//         Pwb = Pwb + Vw * dt + 0.5 * dt * dt * acc_w;
//         Vw = Vw + acc_w * dt;

//         // Save results
//         save_points << std::setprecision(5)
//                     << imupose.timestamp << " "
//                     << std::setprecision(10)
//                     << Qwb.w() << " "
//                     << Qwb.x() << " "
//                     << Qwb.y() << " "
//                     << Qwb.z() << " "
//                     << Pwb(0) << " "
//                     << Pwb(1) << " "
//                     << Pwb(2) << " "
//                     << Qwb.w() << " "
//                     << Qwb.x() << " "
//                     << Qwb.y() << " "
//                     << Qwb.z() << " "
//                     << Pwb(0) << " "
//                     << Pwb(1) << " "
//                     << Pwb(2) << " "
//                     << std::endl;
//     }

//     std::cout << "test end" << std::endl;
// }
