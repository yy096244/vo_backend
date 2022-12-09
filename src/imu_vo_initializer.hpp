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

      // the vo robotion to imu and imu rotation should be same
      for (int i = 0; i < imu_pre_delta_rov.size(); i++) {
        Eigen::Matrix3d A_temp = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b_temp = Eigen::Vector3d::Zero();
        Eigen::Quaterniond q_ij_imu(imu_pre_delta_rov[i].block<3, 3>(0, 0));
        Eigen::Quaterniond q_ij_vo(vo_delta_rov[i].block<3, 3>(0, 0));
        A_temp = imu_jacobian_bias[i];
        b_temp = 2 * (q_ij_imu.inverse() * q_ij_vo).vec();

        A += A_temp.transpose() * A_temp;
        b += b_temp.transpose() * b_temp;
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

}
// ---------------------------------------------------------------------------------