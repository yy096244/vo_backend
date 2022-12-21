#pragma once

#include <boost/core/checked_delete.hpp>
#include <deque>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/PreintegrationBase.h>
#include <map>
#include "Eigen/src/Core/Matrix.h"
#include "imu_buf.hpp"
#include "imu_vo_initializer.hpp"

// gtsam related
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/inference/Symbol.h>
#include <memory>

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

        // TODO : 从索引为1开始进行预积分
        for (auto &imui : imus) {
            double dt = imui.timestamp - last_imu_time_;
            last_imu_time_ = imui.timestamp;
            pimu_preintegrated_->integrateMeasurement(imui.am, imui.wm, dt);
        }

        // push vo pose
        vo_poses_[time] = pose;

        // for initialization, save the original imu data
        if (!vo_imu_has_aligned_) {
            imu_datas_.push_back(imus);
        }

        // preintgrate the imus data with gtsam
        // sliding window optimize


        // try to initilize

        // optimize slide window
    }

    // 获取vo的一个整体的旋转以及整体的尺度，对整个单目vo的前端来进行设置
    bool get_vo_imu_align(Eigen::Matrix4d& delta_mat, double& scale) {
        return true;
    }



private:
    // core data
    std::map<double, Eigen::Matrix4d> vo_poses_;
    std::vector<std::deque<vobackend::ImuData>> imu_datas_;
    std::vector<std::shared_ptr<gtsam::PreintegrationType>> pints_;



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
    vobackend::ImuVoInitializer imu_vo_aligner_;

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
    if (!is_imu_vo_aligned_) {
        // 首先将用于解算陀螺仪零偏的数据用来重新进行预积分(这部分由于更新了陀螺仪的零偏，因此需要重新的进行预积分)
        if (!imu_datas_.empty()) {
            double lt = imu_datas_[0][0].timestamp;
            // check the imu_datas_时间戳与vo_pose的时间戳是一样的
            for (int i = 0; i < imu_datas_.size(); i++) {
                Eigen::Matrix4d delta_vo = vo_poses_[i].inverse() * vo_poses_[i+1];
                Eigen::Matrix4d delta_vo_imu = ex_camera_to_imu_ * delta_vo * ex_camera_to_imu_.inverse();

                std::shared_ptr<gtsam::PreintegrationType> pint(new gtsam::PreintegrationType(*pimu_preintegrated_));
                pint->resetIntegrationAndSetBias(prior_imu_bias_);

                // TODO : 忽略起始开头的那一帧数据
                for (int j = 1; j < imu_datas_[i].size(); j++) {
                    double dt = imu_datas_[i][j].timestamp - lt;
                    lt = imu_datas_[i][j].timestamp;
                    pint->integrateMeasurement(imu_datas_[i][j].am, imu_datas_[i][j].wm, dt);
                }

                // put the data into vo imu align
                // Eigen::Matrix3d imu_rov = pint->deltaRij().matrix();
                // Eigen::Vector3d imu_trans = pint->deltaPij();
                // Eigen::Matrix4d imu_T = Eigen::Matrix4d::Identity();
                // imu_T.block<3, 3>(0, 0) = imu_rov;
                // imu_T.block<3, 1>(0, 3) = imu_trans;

                double imu_delta_dt = pint->deltaTij();
                Eigen::Matrix3d imu_delta_rov = pint->deltaRij().matrix();
                Eigen::Vector3d imu_delta_p = pint->deltaPij();
                Eigen::Vector3d imu_delta_v = pint->deltaVij();

                // put the data into imu vo aligner
                imu_vo_aligner_.insert_data(imu_delta_dt, imu_delta_rov, imu_delta_p, imu_delta_v, delta_vo_imu);

                pints_.push_back(pint);
            }
            imu_datas_.clear();
        } else { // 此时用最新的imu预先积分数据来进行数据的对齐
            Eigen::Matrix4d delta_vo = vo_poses_[vo_poses_.size()-2].inverse() * vo_poses_[vo_poses_.size()-1];
            Eigen::Matrix4d delta_vo_imu = ex_camera_to_imu_ * delta_vo * ex_camera_to_imu_.inverse();

            double imu_delta_dt = pimu_preintegrated_->deltaTij();
            Eigen::Matrix3d imu_delta_rov = pimu_preintegrated_->deltaRij().matrix();
            Eigen::Vector3d imu_delta_p = pimu_preintegrated_->deltaPij();
            Eigen::Vector3d imu_delta_v = pimu_preintegrated_->deltaVij();
            imu_vo_aligner_.insert_data(imu_delta_dt, imu_delta_rov, imu_delta_p, imu_delta_v, delta_vo_imu);

            std::shared_ptr<gtsam::PreintegrationType> pint(new gtsam::PreintegrationType(*pimu_preintegrated_));
            pints_.push_back(pint);
        }

        // try to align
        Eigen::Vector3d gravity_direction;
        double scale;
        std::vector<Eigen::Vector3d> vs;
        bool align_ok = imu_vo_aligner_.try_to_align(gravity_direction, scale, vs);

        // vo imu align(视觉惯性的对齐的逻辑)
        if (align_ok) {
            // 1 根据重力的方向以及对应的尺度对整个vo进行调整(需要根据整个ex来进行调整)
            // 首先计算得到第一帧imu相对重力坐标系下的pose
            // 根据第一帧imu的pose以及对应的外参得到第一帧camera的(旋转，平移)
            // 对所有的位姿的pose来进行变换


            {




            }

            // 2 将用于初始化过程中的imu以及vo的pose用来进行构建预积分的约束
            // 对所有的imu根据估计的零偏进行重新的预积分
            // graph中构建预积分的约束

            // 3 设置对应的标志位
            is_imu_vo_aligned_ = true;
        }
    }
}

void ImuVoInterface::optimize() {
    if (!gyro_bias_has_solved_ || !is_imu_vo_aligned_) {
        return;
    }

    // add the current imu_preint to graph


    // sliding window

}
