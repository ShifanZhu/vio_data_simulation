//
// Created by hyj on 17-6-22.
//

#include <fstream>
#include <sys/stat.h>
#include "../src/imu.h"
#include "../src/utilities.h"

using Point = Eigen::Vector4d;
using Points = std::vector<Point, Eigen::aligned_allocator<Point> >;
using Line = std::pair<Eigen::Vector4d, Eigen::Vector4d>;
using Lines = std::vector<Line, Eigen::aligned_allocator<Line> >;

void CreatePointsLines(Points& points, Lines& lines)
{
    std::ifstream f;
    f.open("../models/house_model/house.txt");

    while(!f.eof())
    {
        std::string s;
        std::getline(f,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double x,y,z;
            ss >> x;
            ss >> y;
            ss >> z;
            Eigen::Vector4d pt0( x, y, z, 1 );
            ss >> x;
            ss >> y;
            ss >> z;
            Eigen::Vector4d pt1( x, y, z, 1 );

            bool isHistoryPoint = false;
            for (int i = 0; i < points.size(); ++i) {
                Eigen::Vector4d pt = points[i];
                if(pt == pt0)
                {
                    isHistoryPoint = true;
                }
            }
            if(!isHistoryPoint)
                points.push_back(pt0);

            isHistoryPoint = false;
            for (int i = 0; i < points.size(); ++i) {
                Eigen::Vector4d pt = points[i];
                if(pt == pt1)
                {
                    isHistoryPoint = true;
                }
            }
            if(!isHistoryPoint)
                points.push_back(pt1);

            // pt0 = Twl * pt0;
            // pt1 = Twl * pt1;
            lines.emplace_back(pt0, pt1);   // lines
        }
    }

    // // create more 3d points, you can comment this code
    // int n = points.size();
    // for (int j = 0; j < n; ++j) {
    //     Eigen::Vector4d p = points[j] + Eigen::Vector4d(0.05,0.05,-0.05,0);
    //     points.push_back(p);
    // }

    // save points
    save_points("all_points.txt", points);
}

int main(){

    // Eigen::Quaterniond Qwb;
    // Qwb.setIdentity();
    // Eigen::Vector3d omega (0,0,M_PI/10);
    // double dt_tmp = 0.005;
    // for (double i = 0; i < 20.; i += dt_tmp) {
    //     Eigen::Quaterniond dq;
    //     Eigen::Vector3d dtheta_half =  omega * dt_tmp /2.0;
    //     dq.w() = 1;
    //     dq.x() = dtheta_half.x();
    //     dq.y() = dtheta_half.y();
    //     dq.z() = dtheta_half.z();
    //     Qwb = Qwb * dq;
    // }
    // std::cout << Qwb.coeffs().transpose() <<"\n"<<Qwb.toRotationMatrix() << std::endl;

    // 建立keyframe文件夹
    mkdir("keyframe", 0777);

    // 生成3d points
    Points points;
    Lines lines;
    CreatePointsLines(points, lines);

    // IMU model
    Param params;
    IMU imuGen(params);

    // create imu data
    // imu pose gyro acc
    std::vector< MotionData > imudata;
    std::vector< MotionData > imudata_noise;

    // static motion
    for (float t = params.t_start; t<params.t_start + params.t_static-1e-5;) {
        // MotionData data = imuGen.StaticMotionModel(t, 0);
        MotionData data = imuGen.StaticMotionModelSinShape(t, 0);
        imudata.push_back(data);

        // add imu noise
        MotionData data_noise = data;
        imuGen.addIMUnoise(data_noise);
        imudata_noise.push_back(data_noise);

        t += 1.0/params.imu_frequency;
    }

    Eigen::Vector3d imu_velocity;
    bool get_vel = false;
    // specific motion
    for (float t = params.t_start; t < params.t_end;)
    {
        // MotionData data = imuGen.MotionModel(t, params.t_static);
        // MotionData data = imuGen.MotionModelSO3(t, params.t_static);
        // MotionData data = imuGen.MotionModelStraightLine(t, params.imu_timestep, params.t_static);
        MotionData data = imuGen.MotionModelSinShape(t, params.t_static); // imu body frame to world frame motion
        if (!get_vel) {
            imu_velocity = data.imu_velocity;
            get_vel = true;
        }
        imudata.push_back(data);

        // add imu noise
        MotionData data_noise = data;
        imuGen.addIMUnoise(data_noise);
        imudata_noise.push_back(data_noise);

        t += 1.0/params.imu_frequency;
    }
    imuGen.init_velocity_ = imudata[0].imu_velocity;
    // imuGen.init_velocity_ = imu_velocity; // imudata[0].imu_velocity;
    imuGen.init_twb_ = imudata.at(0).twb;
    imuGen.init_Rwb_ = imudata.at(0).Rwb;
    save_Pose("imu_pose.txt", imudata);
    save_Pose("imu_pose_noise.txt", imudata_noise);
    save_Pose_for_SAD("imu_data_sad.txt", imudata);
    save_Pose_for_SAD("imu_data_sad_noise.txt", imudata_noise);

    imuGen.testImu("imu_pose.txt", "imu_int_pose.txt"); // test the imu data, integrate the imu data to generate the imu trajecotry
    imuGen.testImu("imu_pose_noise.txt", "imu_int_pose_noise.txt");

    // cam pose
    std::vector< MotionData > camdata;
    imuGen.position_ = Eigen::Vector3d(0,0,0);
    imuGen.velocity_ = Eigen::Vector3d(0,0,0);
    imuGen.Rwb_ = Sophus::SO3<double>();

    for (float t = params.t_start; t<params.t_start + params.t_static-1e-5;) {

        // MotionData imu = imuGen.StaticMotionModel(t, 0); // imu body frame to world frame motion
        MotionData imu = imuGen.StaticMotionModelSinShape(t, 0);
        MotionData cam;

        cam.timestamp = imu.timestamp;
        cam.Rwb = imu.Rwb * params.R_bc;           // cam frame in world frame
        cam.twb = imu.twb + imu.Rwb * params.t_bc; //  Tcw = Twb * Tbc ,  t = Rwb * tbc + twb
        std::cout << std::endl
                  << "first imu pose: " << t << std::endl
                  << imu.Rwb << std::endl
                  << imu.twb << std::endl;

        camdata.push_back(cam);
        t += 1.0 / params.cam_frequency;
    }
    for (float t = params.t_start; t<params.t_end;) {

        // MotionData imu = imuGen.MotionModel(t, params.t_static); // imu body frame to world frame motion
        // MotionData imu = imuGen.MotionModelSO3(t, params.t_static); // imu body frame to world frame motion
        // MotionData imu = imuGen.MotionModelStraightLine(t, 1.0/params.cam_frequency, params.t_static); // imu body frame to world frame motion
        MotionData imu = imuGen.MotionModelSinShape(t, params.t_static); // imu body frame to world frame motion
        MotionData cam;

        cam.timestamp = imu.timestamp;
        cam.Rwb = imu.Rwb * params.R_bc;    // cam frame in world frame
        cam.twb = imu.twb + imu.Rwb * params.t_bc; //  Tcw = Twb * Tbc ,  t = Rwb * tbc + twb

        camdata.push_back(cam);
        t += 1.0/params.cam_frequency;
    }
    save_Pose("cam_pose.txt",camdata);
    save_Pose_asTUM("cam_pose_tum.txt",camdata);

    // print all points
    for (int i = 0; i < points.size(); ++i) {
        auto p = points[i];
        std::cout << "{" << p[0] << ", " << p[1] << ", " << p[2] << "}, ";
    }

    std::cout << std::endl << "first cam pose: " << camdata[0].timestamp << std::endl << camdata[0].Rwb << std::endl << camdata[0].twb << std::endl;

    double t(0);
    // points obs in image
    for(int n = 0; n < camdata.size(); ++n)
    {
        MotionData data = camdata[n];
        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        Twc.block(0, 0, 3, 3) = data.Rwb;
        Twc.block(0, 3, 3, 1) = data.twb;
        Eigen::Matrix4d Tbc = Eigen::Matrix4d::Identity();
        Eigen::Matrix4d Twb = Eigen::Matrix4d::Identity();
        Tbc.block(0, 0, 3, 3) = params.R_bc;
        Tbc.block(0, 3, 3, 1) = params.t_bc;
        Twb = Twc * Tbc.inverse();

        // 遍历所有的特征点，看哪些特征点在视野里
        std::vector<Vector5d, Eigen::aligned_allocator<Vector5d>> stamped_points_cam;             // ３维点在当前cam视野里
        std::vector<Vector5d, Eigen::aligned_allocator<Vector5d>> stamped_points_imu;             // ３维点在当前IMU系里
        std::vector<Vector5d, Eigen::aligned_allocator<Vector5d>> stamped_points_world;           // ３维点在世界坐标系里
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> points_cam;     // ３维点在当前cam视野里
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features_cam;  // 对应的２维图像坐标
        for (int i = 0; i < points.size(); ++i) {
            Eigen::Vector4d pw = points[i];          // 最后一位存着feature id
            pw[3] = 1;                               //改成齐次坐标最后一位
            Eigen::Vector4d pc1 = Twc.inverse() * pw; // T_wc.inverse() * Pw  -- > point in cam frame
            Eigen::Vector3d p_b = params.R_bc * pc1.head<3>() + params.t_bc; // T_bc * Pw  -- > point in imu frame

            // if(pc1(2) < 0) continue; // z必须大于０,在摄像机坐标系前方

            Eigen::Vector2d obs(pc1(0)/pc1(2), pc1(1)/pc1(2));
            // if( (obs(0)*460 + 255) < params.image_h && ( obs(0) * 460 + 255) > 0 &&
                   // (obs(1)*460 + 255) > 0 && ( obs(1)* 460 + 255) < params.image_w )
            {
                points_cam.push_back(points[i]); // points in world frame
                features_cam.push_back(obs); // points in camera frame (normalized plane)
                stamped_points_cam.push_back(Vector5d(t, i, pc1(0), pc1(1), pc1(2)));
                stamped_points_imu.push_back(Vector5d(t, i, p_b(0), p_b(1), p_b(2)));
                stamped_points_world.push_back(Vector5d(t, i, pw(0), pw(1), pw(2)));
                // std::cout << "points_world: " << pw.transpose() << std::endl;
            }
        }

        // save points
        std::stringstream filename1;
        filename1<<"keyframe/all_points_"<<n<<".txt";
        save_features(filename1.str(),points_cam,features_cam);
        save_points_with_time("points_cam.txt", stamped_points_cam);
        save_points_with_time("points_imu.txt", stamped_points_imu);
        save_points_with_time("points_world.txt", stamped_points_world);
        t += 1.0/params.cam_frequency;
    }

    // lines obs in image
    for(int n = 0; n < camdata.size(); ++n)
    {
        MotionData data = camdata[n];
        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        Twc.block(0, 0, 3, 3) = data.Rwb;
        Twc.block(0, 3, 3, 1) = data.twb;

        // 遍历所有的特征点，看哪些特征点在视野里
        // std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points_cam;    // ３维点在当前cam视野里
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > features_cam;  // 对应的２维图像坐标
        for (int i = 0; i < lines.size(); ++i) {
            Line linept = lines[i];

            Eigen::Vector4d pc1 = Twc.inverse() * linept.first; // T_wc.inverse() * Pw  -- > point in cam frame
            Eigen::Vector4d pc2 = Twc.inverse() * linept.second; // T_wc.inverse() * Pw  -- > point in cam frame

            if(pc1(2) < 0 || pc2(2) < 0) continue; // z必须大于０,在摄像机坐标系前方

            Eigen::Vector4d obs(pc1(0)/pc1(2), pc1(1)/pc1(2),
                                pc2(0)/pc2(2), pc2(1)/pc2(2));
            //if(obs(0) < params.image_h && obs(0) > 0 && obs(1)> 0 && obs(1) < params.image_w)
            {
                features_cam.push_back(obs);
            }
        }

        // save points
        std::stringstream filename1;
        filename1<<"keyframe/all_lines_"<<n<<".txt";
        save_lines(filename1.str(),features_cam);
    }

    return 0;
}
