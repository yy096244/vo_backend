#include <iostream>
#include <map>
#include "imu_buf.hpp"
#include "imu_vo_initializer.hpp"
#include "imu_vo_interface.hpp"
// #include <glog/logging.h>
#include <mutex>
#include <condition_variable>
#include <thread>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

// 参考vins_mono中的数据同步，同时去订阅imu以及svo中的pose的部分
// imu和vo的pose的数据缓存
vobackend::ImuBuf imu_buf;
std::map<double, Eigen::Matrix4d> vo_pose_buf;
vobackend::ImuData convert_imudata(const sensor_msgs::ImuConstPtr &p_ros_imu);

std::condition_variable condition;
std::mutex mutex_buf;

void data_sync() {
    while (ros::ok()) {
        std::unique_lock<std::mutex> lk(mutex_buf);

        // 需要确保vo中的数据大于2，而且
        condition.wait(lk, [&] {
            return vo_pose_buf.size() >= 2 &&
                vo_pose_buf.rbegin()->first <= imu_buf.get_buf_timebound().second;
        });

        // TODO : 如果此时需要插入第一帧的数据，则插入第一帧的数据
        // 取出其中对齐的imu和vo的pose的数据
        auto it1 = vo_pose_buf.begin();
        auto it2 = std::next(it1);
        for (; it2 != vo_pose_buf.end(); it1++, it2++) {
            std::deque<vobackend::ImuData> imus;
            if (imu_buf.get_imus_between(it1->first, it2->first, imus)) {
                // TODO : push back the imu data and the vo data [*it2, imus]
            }
        }
        lk.unlock();
    }
}

void imu_callback(const sensor_msgs::ImuConstPtr& pimu) {
    vobackend::ImuData imu_i = convert_imudata(pimu);
    mutex_buf.lock();
    bool insert_ok = imu_buf.insert_imu(imu_i);
    mutex_buf.unlock();
    if (insert_ok) {
        condition.notify_one();
    }
}

void vopose_callback(const geometry_msgs::PoseWithCovarianceStampedConstPtr& pvopose) {
    mutex_buf.lock();
    if (pvopose->header.stamp.toSec() < imu_buf.get_buf_timebound().first) {
        mutex_buf.unlock();
        return;
    }
    if (!vo_pose_buf.empty()) {
        if (pvopose->header.stamp.toSec() <= vo_pose_buf.rbegin()->first) {
            mutex_buf.unlock();
            return;
        }
    }
    
    // convert the pose to Eigen::Matrix4d
    Eigen::Quaterniond q(pvopose->pose.pose.orientation.w, pvopose->pose.pose.orientation.x,
        pvopose->pose.pose.orientation.y, pvopose->pose.pose.orientation.z);
    Eigen::Vector3d t{pvopose->pose.pose.position.x, pvopose->pose.pose.position.y,
        pvopose->pose.pose.position.z};
    Eigen::Matrix4d posei = Eigen::Matrix4d::Identity();
    posei.block<3, 3>(0, 0) = Eigen::Matrix3d(q);
    posei.block<3, 1>(0, 3) = t;
    double vo_time = pvopose->header.stamp.toSec();
    vo_pose_buf[vo_time] = posei;
    mutex_buf.unlock();
    condition.notify_one();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "vo_backend");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");
    
    ROS_INFO_STREAM("vo_backend_interface start");

    // ros params
    std::string imu_topic("/imu/data");
    std::string vo_topic("/vo/data");
    nh_private.param("imu_topic", imu_topic, imu_topic);
    nh_private.param("vo_topic", vo_topic, vo_topic);
    
    ros::Subscriber imu_sub = nh.subscribe(imu_topic, 100, imu_callback);
    ros::Subscriber vo_sub = nh.subscribe(vo_topic, 100, vopose_callback);
    
    // start thread
    std::thread data_sync_thread{data_sync};
    
    ros::spin();
    
    // data_sync_thread.join();
    // glogInit();
    // google::InitGoogleLogging(argv[0]);
    // google::SetLogDestination(google::INFO, "/tmp");
    // LOG(INFO) << "vo_backend run";

    // sub vo pose and wait for imu measurement(multithread)

    // insert imus data and vo_pose to vo_backend

    // get between data to preint and save the pose

    // initialization to solve the bias

    // initalization to solve the gravity and the scale and the pose

    return 0;
}

vobackend::ImuData convert_imudata(const sensor_msgs::ImuConstPtr &p_ros_imu) {
    vobackend::ImuData imu_data;
    imu_data.timestamp = p_ros_imu->header.stamp.toSec();
    imu_data.am.x() = p_ros_imu->linear_acceleration.x;
    imu_data.am.y() = p_ros_imu->linear_acceleration.y;
    imu_data.am.z() = p_ros_imu->linear_acceleration.z;
    imu_data.wm.x() = p_ros_imu->angular_velocity.x;
    imu_data.wm.y() = p_ros_imu->angular_velocity.y;
    imu_data.wm.z() = p_ros_imu->angular_velocity.z;
    return imu_data;
}