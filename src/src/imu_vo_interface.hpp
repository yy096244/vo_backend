#pragma once

#include <deque>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/PreintegrationBase.h>
#include <gtsam/nonlinear/Values.h>
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
#include <memory>

// copy from dmvio for marginlization for sliding window optimize
#include "Marginalization.h"

// config
// gtsam中相关的内容的集成
// 1 首先会调用gtsam中的imu的预积分的相关内容，预积分的数据接口，雅可比矩阵，得到相对的旋转
// 2 需要找到imu是如何来进行预积分的？

using gtsam::symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using gtsam::symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using gtsam::symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)

class ImuVoInterface {
public:

    ImuVoInterface() :
     graph_(new gtsam::NonlinearFactorGraph())
     {
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
        if (!is_imu_vo_aligned_) {
            imu_datas_.push_back(imus);
        }

        // try to initilize
        try_to_initialize();

        // optimize slide window
        optimize();
    }

    // 获取vo的一个整体的旋转以及整体的尺度，对整个单目vo的前端来进行设置
    bool get_vo_imu_align(Eigen::Matrix4d& delta_mat, double& scale) {

        if (!is_imu_vo_aligned_) {
            return false;
        }

        T_vo_delta_ = delta_mat;
        vo_scale_ = scale;
        return true;
    }

    bool get_predict_pose(double time, Eigen::Matrix4d& pre_pos) {
        // TODO : 获取预测的位置，vo主要是需要用这个数据
        return true;
    }

    bool get_delta_pose(double time1, double time2, const std::deque<vobackend::ImuData>& imus, Eigen::Matrix4d& delta_pos) {
        // TODO : 根据[time1, time2]之间的imu数据来计算相邻之间的相机的pose
        return true;
    }

    static gtsam::Pose3 mat2pos3(const Eigen::Matrix4d& mat) {
        gtsam::Pose3 gtsam_pos3(gtsam::Rot3(mat.block<3, 3>(0, 0)),
            gtsam::Point3(mat.block<3, 1>(0, 3)));
        return gtsam_pos3;
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

    // for vo imu bias and align
    double last_imu_time_{-1};
    bool gyro_bias_has_solved_{false};
    bool is_imu_vo_aligned_ = false;
    vobackend::ImuVoInitializer imu_vo_aligner_;

    // TODO: set the ex
    Eigen::Matrix4d ex_camera_to_imu_ = Eigen::Matrix4d::Identity();


    // for solving the gyro bias
    std::vector<Eigen::Matrix3d> imu_pre_delta_rov_;
    std::vector<Eigen::Matrix3d> vo_delta_rov_;
    std::vector<Eigen::Matrix3d> imu_jacobian_bias_;

    // 经过vo与imu的pose对齐之后，得到的vo的相对旋转
    Eigen::Matrix4d T_vo_delta_;
    double vo_scale_;

    // for gtsam_optimization
    std::shared_ptr<gtsam::NonlinearFactorGraph> graph_{nullptr};
    gtsam::Values initial_values_;
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
        static bool has_imu_used = false;
        // 首先将用于解算陀螺仪零偏的数据用来重新进行预积分(这部分由于更新了陀螺仪的零偏，因此需要重新的进行预积分)
        if (!imu_datas_.empty() && !has_imu_used) {
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
            // imu_datas_.clear();
            has_imu_used = true;
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
                // 根据 gravity_direction 来计算起始的imu的旋转
                gravity_direction.normalize();
                std::cout << "gravity_direction : "  << gravity_direction.transpose() << std::endl;
                Eigen::Quaterniond q_w_b = Eigen::Quaterniond::FromTwoVectors(gravity_direction, Eigen::Vector3d{0, 0, 1});

                // 将yaw角第一帧设置为0，
                Eigen::Vector3d ypr = Eigen::Matrix3d(q_w_b).eulerAngles(2, 1, 0);
                ypr(0) = 0;
                q_w_b = Eigen::AngleAxisd(ypr(0), Eigen::Vector3d{0, 0, 1})
                      * Eigen::AngleAxisd(ypr(1), Eigen::Vector3d{0, 1, 0})
                      * Eigen::AngleAxisd(ypr(2), Eigen::Vector3d{1, 0, 0});
                std::cout << "q_w_b : " << q_w_b.coeffs().transpose() << std::endl;

                // 将第一帧的imu位置设置为0
                Eigen::Vector3d pos_0 = Eigen::Vector3d{0, 0, 0};

                Eigen::Matrix4d T_imu_0 = Eigen::Matrix4d::Zero();
                T_imu_0.block<3, 3>(0, 0) = Eigen::Matrix3d(q_w_b);
                T_imu_0.block<3, 1>(0, 3) = pos_0;

                // 根据当前imu的位姿以及相机的外参，预测第一帧相机的pose，计算出当前的vo的pose的偏移
                Eigen::Matrix4d T_camera_0_imu = T_imu_0 * ex_camera_to_imu_;
                Eigen::Matrix4d T_camera_0_vo = vo_poses_[0];

                T_vo_delta_ = T_camera_0_imu.inverse() * T_camera_0_vo;
                vo_scale_ = scale;
            }

            // 2 将用于初始化过程中的imu以及vo的pose用来进行构建预积分的约束
            // 对所有的imu根据估计的零偏进行重新的预积分
            // graph中构建预积分的约束,并且进行起始的一个对应的预积分

            // 根据尺度以及相邻的pose得到对应的vo的pose
            std::vector<Eigen::Matrix4d> vo_align_scale;
            {
                Eigen::Matrix4d voi = Eigen::Matrix4d::Identity();
                vo_align_scale.push_back(voi);
                for (int i = 1; i < vo_poses_.size(); i++) {
                    Eigen::Matrix4d delta_vo = vo_poses_[i-1].inverse() * vo_poses_[i];
                    Eigen::Matrix4d delta_vo_imu = ex_camera_to_imu_ * delta_vo * ex_camera_to_imu_.inverse();
                    delta_vo_imu.block<3, 1>(0, 3) *= scale;
                    voi = voi * delta_vo_imu;
                    vo_align_scale.push_back(voi);
                }
            }

            // 对所有的imu的数据进行重新的预积分
            std::vector<std::shared_ptr<gtsam::PreintegrationType>> pints;
            {
                double lt = imu_datas_[0][0].timestamp;
                for (int i = 0; i < imu_datas_.size(); i++) {
                    std::shared_ptr<gtsam::PreintegrationType> pint(new gtsam::PreintegrationType(*pimu_preintegrated_));
                    pint->resetIntegrationAndSetBias(prior_imu_bias_);

                    // 预积分对应的vo之间的pose
                    for (int j = 1; j < imu_datas_[i].size(); j++) {
                        double dt = imu_datas_[i][j].timestamp - lt;
                        lt = imu_datas_[i][j].timestamp;
                        pint->integrateMeasurement(imu_datas_[i][j].am, imu_datas_[i][j].wm, dt);
                    }

                    pints.push_back(pint);
                }
            }

            // 构建预积分的约束并进行优化
            gtsam::Values result;
            {
                // 设置第一帧对应的pose
                gtsam::noiseModel::Diagonal::shared_ptr first_pose_noise_model 
                    = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) 
                      << 0.001, 0.001, 0.00001, 0.0000005, 0.0000005, 0.0000005).finished()); // rad,rad,rad,m, m, m
                gtsam::noiseModel::Diagonal::shared_ptr velocity_noise_model = gtsam::noiseModel::Isotropic::Sigma(3,0.05); // m/s
                gtsam::noiseModel::Diagonal::shared_ptr bias_noise_model = gtsam::noiseModel::Isotropic::Sigma(6,1e-3);

                gtsam::noiseModel::Diagonal::shared_ptr pose_noise_model 
                    = gtsam::noiseModel::Diagonal::Sigmas((gtsam::Vector(6) 
                      << 0.01, 0.01, 0.01, 0.05, 0.05, 0.05).finished()); // rad,rad,rad,m, m, m

                // 设置对应的优化的起始值
                gtsam::Pose3 first_pose = mat2pos3(vo_align_scale[0]);
                gtsam::Vector3 prior_velocity(vs[0]);
                gtsam::imuBias::ConstantBias prior_imu_bias = prior_imu_bias_;

                // 将第一帧的pose添加到graph中
                graph_->add(gtsam::PriorFactor<gtsam::Pose3>(X(0), first_pose, pose_noise_model));
                graph_->add(gtsam::PriorFactor<gtsam::Vector3>(V(0), prior_velocity, velocity_noise_model));
                graph_->add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(0), prior_imu_bias_,bias_noise_model));

                initial_values_.insert(X(0), first_pose);
                initial_values_.insert(V(0), prior_velocity);
                initial_values_.insert(B(0), prior_imu_bias_);

                for (int i = 0; i < pints.size(); i++) {
                    // 添加每帧的起始量以及对应的初始值
                    gtsam::Pose3 posei = mat2pos3(vo_align_scale[i]);
                    gtsam::Vector3 velocityi(vs[i]);
                    graph_->add(gtsam::PriorFactor<gtsam::Pose3>(X(i), posei, first_pose_noise_model));
                    graph_->add(gtsam::PriorFactor<gtsam::Vector3>(V(i), velocityi, velocity_noise_model));
                    graph_->add(gtsam::PriorFactor<gtsam::imuBias::ConstantBias>(B(i), prior_imu_bias_,bias_noise_model));

                    initial_values_.insert(X(i), posei);
                    initial_values_.insert(V(i), velocityi);
                    initial_values_.insert(B(i), prior_imu_bias_);

                    // add imu factor
                    gtsam::PreintegratedCombinedMeasurements *preint_imu_combined = dynamic_cast<gtsam::PreintegratedCombinedMeasurements*>(pints[i].get());
                    gtsam::CombinedImuFactor imu_factor(X(i-1), V(i-1),
                        X(i  ), V(i  ),
                        B(i-1), B(i ),
                        *preint_imu_combined);
                    graph_->add(imu_factor);

                    // optimize
                    gtsam::LevenbergMarquardtOptimizer optimizer(*graph_, initial_values_);
                    result = optimizer.optimize();
                }
            }

            // TODO : 利用result来进行设置

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
    // 1 根据最新的pose以及传入的相对误差预测当前最新的pose
        // 或者根据当前的imu的预积分得到当前的最新的预测的pose
    
    // 2 添加最新的factor

    // TODO: sliding window
}
