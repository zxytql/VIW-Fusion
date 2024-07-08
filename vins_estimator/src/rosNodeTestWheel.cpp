/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "ros_things.h"
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
// #include <ros/ros.h>
// #include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
#include "utility/visualization.h"

#define TIME_SHIFT_TORLERANCE (5.0f)
Estimator estimator;

queue<sensor_msgs::ImuConstPtr> imu_buf;
queue<sensor_msgs::PointCloudConstPtr> feature_buf;
queue<pair<cv::Mat, double>> img0_buf; 
queue<pair<cv::Mat, double>> img1_buf; 
bool is_img_time_shift;
bool is_imu_time_shift;
std::mutex m_buf;


void img0_callback(const cv::Mat &img_msg, const double &t)
{
    static double last_img_t = -1;
    
    if(last_img_t < 0)
    {
        last_img_t = t;
    }
    // 检查最新传入的图像数据的时间戳是否突变
    // @todo 这里的判断有点随意 
    // 可能还需要判断后续的时间戳是否都是正常的再做重启
    if( t - last_img_t >= TIME_SHIFT_TORLERANCE)
    {
        printf("*****************is_img_time_shift set 1 \n");
        printf("last_img_t: %lf, now t: %lf \n",last_img_t, t);
        is_img_time_shift = true;
    }

    m_buf.lock();
    pair<cv::Mat, double> tmp;
    tmp.first=img_msg;
    tmp.second=t;
    img0_buf.push(tmp);
    last_img_t = t;
    m_buf.unlock();
}

// 没有用到
void img1_callback(const cv::Mat &img_msg, const double &t)
{
    m_buf.lock();
    pair<cv::Mat, double> tmp;
    tmp.first=img_msg;
    tmp.second=t;
    img1_buf.push(tmp);
    m_buf.unlock();
}

void wheel_callback(const nav_msgs::OdometryConstPtr &odom_msg)
{
    double t = odom_msg->header.stamp.toSec();
    double dx = odom_msg->twist.twist.linear.x;
    double dy = odom_msg->twist.twist.linear.y;
    double dz = odom_msg->twist.twist.linear.z;
    double rx = odom_msg->twist.twist.angular.x;
    double ry = odom_msg->twist.twist.angular.y;
    double rz = odom_msg->twist.twist.angular.z;
    Vector3d vel(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputWheel(t, vel, gyr);
    return;
}
// 从msg中获取图片，返回值cv::Mat，输入是当前图像msg的指针
// cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
// {
//     cv_bridge::CvImageConstPtr ptr;
//     if (img_msg->encoding == "8UC1")
//     {
//         sensor_msgs::Image img;
//         img.header = img_msg->header;
//         img.height = img_msg->height;
//         img.width = img_msg->width;
//         img.is_bigendian = img_msg->is_bigendian;
//         img.step = img_msg->step;
//         img.data = img_msg->data;
//         img.encoding = "mono8";
//         ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
//     }
//     else
//         ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

//     cv::Mat img = ptr->image.clone();
//     return img;
// }

// extract images with same timestamp from two topics
void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cerr << "You have enter STEREO !!!" << endl;
            assert(0);
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front().second;
                image = img0_buf.front().first;
                //printf("pop img data: %f \n", img0_buf.front().second);
                img0_buf.pop();
            }
            m_buf.unlock();
            if(!image.empty())
                estimator.inputImage(time, image);
            
        }

        // 如果imu和图像数据都发生了时间戳突变，执行算法重启
        // @todo 这里要不要单独开一个线程进行操作呢 1000HZ的判断有点高了
        //cout << "Now imu and img bool is " << is_imu_time_shift << "," << is_img_time_shift << endl;
        if (is_imu_time_shift && is_img_time_shift) 
        {
            printf("*******imu & img shift leads restart \n");
            restart_callback();
        }
        
        std::chrono::milliseconds dura(1);
        std::this_thread::sleep_for(dura);
    }
}


void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    Vector3d acc(dx, dy, dz);
    Vector3d gyr(rx, ry, rz);
    estimator.inputIMU(t, acc, gyr);
    return;
}


// void feature_callback(const sensor_msgs::PointCloudConstPtr &feature_msg)
// {
//     map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
//     for (unsigned int i = 0; i < feature_msg->points.size(); i++)
//     {
//         int feature_id = feature_msg->channels[0].values[i];
//         int camera_id = feature_msg->channels[1].values[i];
//         double x = feature_msg->points[i].x;
//         double y = feature_msg->points[i].y;
//         double z = feature_msg->points[i].z;
//         double p_u = feature_msg->channels[2].values[i];
//         double p_v = feature_msg->channels[3].values[i];
//         double velocity_x = feature_msg->channels[4].values[i];
//         double velocity_y = feature_msg->channels[5].values[i];
//         if(feature_msg->channels.size() > 5)
//         {
//             double gx = feature_msg->channels[6].values[i];
//             double gy = feature_msg->channels[7].values[i];
//             double gz = feature_msg->channels[8].values[i];
//             pts_gt[feature_id] = Eigen::Vector3d(gx, gy, gz);
//             //printf("receive pts gt %d %f %f %f\n", feature_id, gx, gy, gz);
//         }
//         ROS_ASSERT(z == 1);
//         Eigen::Matrix<double, 7, 1> xyz_uv_velocity;
//         xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
//         featureFrame[feature_id].emplace_back(camera_id,  xyz_uv_velocity);
//     }
//     double t = feature_msg->header.stamp.toSec();
//     estimator.inputFeature(t, featureFrame);
//     return;
// }

void restart_callback()
{
    printf("********************************************************\n");
    printf("**             Encounter time shift                   **\n");
    printf("**           VIO system is restarting!!               **\n");
    printf("**                                                    **\n");
    printf("********************************************************\n");

    estimator.clearState();
    estimator.setParameter();

    is_img_time_shift = false;
    is_imu_time_shift = false;
    //is_initialized = false;
    return ;
}

int main(int argc, char **argv)
{
    // ros::init(argc, argv, "vins_estimator");
    // ros::NodeHandle n("~");
    // ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif

    // ROS_WARN("waiting for image and imu...");

    // registerPub(n);
//TODO transport hints 怎么用
    // ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_wheel = n.subscribe(WHEEL_TOPIC, 2000, wheel_callback, ros::TransportHints().tcpNoDelay());
    // ros::Subscriber sub_feature = n.subscribe("/feature_tracker/feature", 2000, feature_callback);
    // ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    // ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    // ros::Subscriber sub_restart = n.subscribe("/vins_restart", 100, restart_callback);
    // ros::Subscriber sub_imu_switch = n.subscribe("/vins_imu_switch", 100, imu_switch_callback);
    // ros::Subscriber sub_cam_switch = n.subscribe("/vins_cam_switch", 100, cam_switch_callback);

    std::thread sync_thread{sync_process};
    sync_thread.detach();

    // 如果你的程序写了相关的消息订阅函数，那么程序在执行过程中，除了主程序以外，ROS还会自动在后台按照你规定的格式，接受订阅的消息，但是所接到的消息并不是
    // 立刻就被处理，而是必须要等到ros::spin()或ros::spinOnce()执行的时候才被调用，这就是消息回到函数的原理
    return 0;
}
