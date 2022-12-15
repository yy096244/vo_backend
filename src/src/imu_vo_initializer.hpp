# pragma once

// initial from vo and imu data
// copy from vins_mono
#include <Eigen/Core>
#include <Eigen/Dense>
#include <vector>

namespace vobackend
{

// ---------------------------------------------------------------------------------
class GyroBiasSolver {
  static bool solve_gyroscope_bias(const std::vector<Eigen::Matrix4d>& imu_pre_delta_rov,
    const std::vector<Eigen::Matrix4d>& vo_delta_rov, const std::vector<Eigen::Matrix3d>& imu_jacobian_bias,
    Eigen::Vector3d& gyro_bias) {
      Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
      Eigen::Vector3d b = Eigen::Vector3d::Zero();
      Eigen::Vector3d delta_bg;

      // TODO ： imu的角度预积分关于零偏的偏导

      // the vo robotion to imu and imu rotation should be same
      for (int i = 0; i < imu_pre_delta_rov.size(); i++) {
        Eigen::Matrix3d A_temp = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b_temp = Eigen::Vector3d::Zero();
        Eigen::Quaterniond q_ij_imu(imu_pre_delta_rov[i].block<3, 3>(0, 0));
        Eigen::Quaterniond q_ij_vo(vo_delta_rov[i].block<3, 3>(0, 0));
        A_temp = imu_jacobian_bias[i];
        b_temp = 2 * (q_ij_imu.inverse() * q_ij_vo).vec();
        A += A_temp.transpose() * A_temp;
        b += A_temp.transpose() * b_temp;
      }

      // solve delta gyro bias
      delta_bg = A.ldlt().solve(b);

      // todo : determin wheather the result is ok

      gyro_bias = gyro_bias + delta_bg;

      return true;
    }
};
// ---------------------------------------------------------------------------------



// ---------------------------------------------------------------------------------
// get the imu pose with regard to the gravity and the vo scale
// modified from vins_mono
class ImuVoInitializer {
public:
  void insert_data(const Eigen::Matrix4d& imu_preint_delta_pose, const Eigen::Matrix4d& vo_delta_pose);
  void reset();

  // imu和vo的视觉惯性的对齐,需要估计的量有重力矢量，尺度，各个时刻的速度值
  bool get_result(Eigen::Vector3d& gyro_bias, Eigen::Vector3d& g_dir, Eigen::VectorXd& v_s);
private:
  std::vector<Eigen::Matrix4d> imu_preint_delta_poses_;
  std::vector<Eigen::Matrix4d> vo_delta_poses_;
};

void ImuVoInitializer::insert_data(const Eigen::Matrix4d& imu_preint_delta_pose, const Eigen::Matrix4d& vo_delta_pose) {
  imu_preint_delta_poses_.emplace_back(imu_preint_delta_pose);
  vo_delta_poses_.emplace_back(vo_delta_pose);
}

void ImuVoInitializer::reset() {
  imu_preint_delta_poses_.clear();
  vo_delta_poses_.clear();
}

// 视觉和惯性的对齐.(速度,重力和尺度因子)， 是传入vo的pose还是vo的相对的增量？
bool ImuVoInitializer::get_result(Eigen::Vector3d &gyro_bias, Eigen::Vector3d &g_dir, Eigen::VectorXd &v_s) {
  // 1 首先判断是否有足够多的平移(原来的vins中是需要进行关键帧的判断吗)
  // 2 构建线性求解方程(借助于gtsam来进行求解)
  // 3 返回最终的结果
}

}
// ---------------------------------------------------------------------------------