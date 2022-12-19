#pragma once

#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/PreintegrationBase.h>
#include <map>
#include "imu_buf.hpp"
#include "imu_vo_initializer.hpp"

// gtsam related
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>

// config
// gtsam中相关的内容的集成
// 1 首先会调用gtsam中的imu的预积分的相关内容，预积分的数据接口，雅可比矩阵，得到相对的旋转
// 2 需要找到imu是如何来进行预积分的？



class ImuVoInterface {
public:

    ImuVoInterface() {
        // 创建预积分的默认参数，并且得到一个对应的对应的imu的预积分类(创建一个Combined类)
        boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> p =
            gtsam::PreintegratedCombinedMeasurements::Params::MakeSharedU(9.80125);
        double accel_noise_sigma = 0.0003924;
        double gyro_noise_sigma = 0.000205689024915;
        double accel_bias_rw_sigma = 0.004905;
        double gyro_bias_rw_sigma = 0.000001454441043;
        gtsam::Matrix33 measured_acc_cov = gtsam::Matrix33::Identity(3,3) * pow(accel_noise_sigma,2);
        gtsam::Matrix33 measured_omega_cov = gtsam::Matrix33::Identity(3,3) * pow(gyro_noise_sigma,2);
        gtsam::Matrix33 integration_error_cov = gtsam::Matrix33::Identity(3,3)*1e-8; // error committed in integrating position from velocities
        gtsam::Matrix33 bias_acc_cov = gtsam::Matrix33::Identity(3,3) * pow(accel_bias_rw_sigma,2);
        gtsam::Matrix33 bias_omega_cov = gtsam::Matrix33::Identity(3,3) * pow(gyro_bias_rw_sigma,2);
        gtsam::Matrix66 bias_acc_omega_int = gtsam::Matrix::Identity(6,6)*1e-5; // error in the bias used for preintegration
        // PreintegrationBase params:
        p->accelerometerCovariance = measured_acc_cov; // acc white noise in continuous
        p->integrationCovariance = integration_error_cov; // integration uncertainty continuous
        // should be using 2nd order integration
        // PreintegratedRotation params:
        p->gyroscopeCovariance = measured_omega_cov; // gyro white noise in continuous
        // PreintegrationCombinedMeasurements params:
        p->biasAccCovariance = bias_acc_cov; // acc bias in continuous
        p->biasOmegaCovariance = bias_omega_cov; // gyro bias in continuous
        p->biasAccOmegaInt = bias_acc_omega_int;

        // assume the initial imu bias zero
        prior_imu_bias_ = gtsam::imuBias::ConstantBias();
        pimu_preintegrated_ = new gtsam::PreintegratedCombinedMeasurements(p, prior_imu_bias_);
    }

    bool vo_empty() {
        return vo_poses_.empty();
    }
    void insert_first_vo(double time, const Eigen::Matrix4d& pose) {
        vo_poses_.clear();
        vo_poses_[time] = pose;
        last_imu_time_ = time;
    }
    void insert_data(double time, const Eigen::Matrix4d& pose, const std::deque<vobackend::ImuData>& imus) {
        if (last_imu_time_ < 0) {
            std::cout << "please input the first vo pose" << std::endl;
            return;
        }

        // reset the gtsam imu preint
        pimu_preintegrated_->resetIntegrationAndSetBias(prior_imu_bias_);
        for (auto &imui : imus) {
            double dt = imui.timestamp - last_imu_time_;
            last_imu_time_ = imui.timestamp;
            pimu_preintegrated_->integrateMeasurement(imui.am, imui.wm, dt);
        }

        // push vo pose
        vo_poses_[time] = pose;

        // preintgrate the imus data with gtsam

        // try to initilize

        // optimize slide window
    }
private:
    std::map<double, Eigen::Matrix4d> vo_poses_;
    // TODO : gtsam : the imus_pre_integrate data between the vo
    // std::shared_ptr<gtsam>
    void try_to_initialize();
    void optimize();

    // gtsam related

    // gtsam中实现了两种不同的imu的预积分
    // #ifdef GTSAM_TANGENT_PREINTEGRATION
    // typedef TangentPreintegration PreintegrationType;
    // #else
    // typedef ManifoldPreintegration PreintegrationType;
    // #endif
    gtsam::PreintegrationType* pimu_preintegrated_;
    gtsam::imuBias::ConstantBias prior_imu_bias_;

    double last_imu_time_{-1};
    bool gyro_bias_has_solved_{false};
    bool vo_imu_has_aligned_{false};

    bool is_imu_vo_aligned_ = false;

    // TODO: set the ex
    Eigen::Matrix4d ex_camera_to_imu_ = Eigen::Matrix4d::Identity();


    // for solving the gyro bias
    std::vector<Eigen::Matrix4d> imu_pre_delta_rov_;
    std::vector<Eigen::Matrix4d> vo_delta_rov_;
    std::vector<Eigen::Matrix3d>& imu_jacobian_bias_;
};


void ImuVoInterface::try_to_initialize() {
    // 估计陀螺仪的零偏
    if (!gyro_bias_has_solved_) {
        Eigen::Matrix3d imu_rov = pimu_preintegrated_->deltaRij().matrix();
        Eigen::Matrix4d delta_vo = vo_poses_[vo_poses_.size()-2].inverse() * vo_poses_[vo_poses_.size()-1];
        Eigen::Matrix4d delta_vo_imu = ex_camera_to_imu_ * delta_vo * ex_camera_to_imu_.inverse();
        Eigen::Matrix3d delta_vo_imu_rov = delta_vo_imu.block<3, 3>(0, 0);
        Eigen::Matrix<double, 9, 3> j_gyro_bias = pimu_preintegrated_->preintegrated_H_biasOmega();
        // 从其中抠出来旋转部分关于bias的雅可比
        // Vector3 theta() const     { return preintegrated_.head<3>(); }
        // 排序是 ： 旋转， 位置， 速度
        Eigen::Matrix<double, 3, 3> ji = j_gyro_bias.block<3, 3>(0, 0);

        imu_pre_delta_rov_.push_back(imu_rov);
        vo_delta_rov_.push_back(delta_vo_imu_rov);
        imu_jacobian_bias_.push_back(ji);

        Eigen::Vector3d gyro_bias;
        if (imu_pre_delta_rov_.size() > 100) {
            gyro_bias_has_solved_ = vobackend::GyroBiasSolver::solve_gyroscope_bias(imu_pre_delta_rov_, vo_delta_rov_, imu_jacobian_bias_, gyro_bias);
        }

        if (gyro_bias_has_solved_) {
            imu_pre_delta_rov_.clear();
            vo_delta_rov_.clear();
            imu_jacobian_bias_.clear();
            prior_imu_bias_ = gtsam::imuBias::ConstantBias(Eigen::Vector3d{0, 0, 0}, gyro_bias);
            std::cout << "solve gyro bias : " << gyro_bias.transpose() << std::endl;
        }

        if (gyro_bias_has_solved_) {
            return;
        }
    }

    // 视觉imu的对齐


}
