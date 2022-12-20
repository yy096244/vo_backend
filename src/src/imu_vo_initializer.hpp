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
public :
  static bool solve_gyroscope_bias(const std::vector<Eigen::Matrix3d>& imu_pre_delta_rov,
    const std::vector<Eigen::Matrix3d>& vo_delta_rov, const std::vector<Eigen::Matrix3d>& imu_jacobian_bias,
    Eigen::Vector3d& gyro_bias) {
      Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
      Eigen::Vector3d b = Eigen::Vector3d::Zero();
      Eigen::Vector3d delta_bg;

      // TODO ： imu的角度预积分关于零偏的偏导,通过外部的参数传递进来

      // the vo robotion to imu and imu rotation should be same
      for (int i = 0; i < imu_pre_delta_rov.size(); i++) {
        Eigen::Matrix3d A_temp = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b_temp = Eigen::Vector3d::Zero();
        Eigen::Quaterniond q_ij_imu(imu_pre_delta_rov[i]);
        Eigen::Quaterniond q_ij_vo(vo_delta_rov[i]);
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
  void insert_data(double dt, const Eigen::Matrix3d& r,
    const Eigen::Vector3d& t, const Eigen::Vector3d& v, const Eigen::Matrix4d& vo_delta_pose);
  void reset();

  // imu和vo的视觉惯性的对齐,需要估计的量有重力矢量，尺度，各个时刻的速度值
  bool try_to_align(Eigen::Vector3d& g_dir, double& s, std::vector<Eigen::Vector3d>& vs);
private:
  std::vector<double> imu_preint_dts_;
  std::vector<Eigen::Matrix3d> imu_preint_delta_rovs_;
  std::vector<Eigen::Vector3d> imu_preint_delta_trans_;
  std::vector<Eigen::Vector3d> imu_preint_delta_vels_;
  std::vector<Eigen::Matrix4d> vo_delta_poses_;
};
void ImuVoInitializer::insert_data(double dt, const Eigen::Matrix3d& r,
  const Eigen::Vector3d& t,
  const Eigen::Vector3d& v,
  const Eigen::Matrix4d& vo_delta_pose) {
  imu_preint_dts_.push_back(dt);
  imu_preint_delta_rovs_.push_back(r);
  imu_preint_delta_trans_.push_back(t);
  imu_preint_delta_vels_.push_back(v);
  vo_delta_poses_.emplace_back(vo_delta_pose);
}

void ImuVoInitializer::reset() {
  imu_preint_dts_.clear();
  imu_preint_delta_rovs_.clear();
  imu_preint_delta_trans_.clear();
  imu_preint_delta_vels_.clear();
  vo_delta_poses_.clear();
}

// 视觉和惯性的对齐.(速度,重力和尺度因子)， 是传入vo的pose还是vo的相对的增量？
bool ImuVoInitializer::try_to_align(Eigen::Vector3d& g_dir, double& s, std::vector<Eigen::Vector3d>& vs) {
  // 1 首先判断是否有足够多的平移(原来的vins中是需要进行关键帧的判断吗)
  // TODO : 逻辑需要进一步来优化，这里首先只通过一定数量的, 后面加上在用方程组的可解性来判断是否可以得到最终的结果
  if (vo_delta_poses_.size() < 100) {  
    return false;
  }

  // 2 构建线性求解方程(不考虑外参的情况下，这个只是利用相邻两帧的速度预积分以及位置预先积分)
  // TODO ：test the result
  // 误差的排布 ：重力矢量，尺度，其余对应的速度
  // 与积分对应的尺寸为 n
  // 估计量为 ： 3 + 1 + 3*(n+1) = 3*n + 7
  // 对应的矩阵 A 的尺寸为 (3*n + 7) * (3*n + 7)
  Eigen::VectorXd result;
  {
    int total_size = vo_delta_poses_.size();

    // check imu_preint_delta_rovs_ imu_preint_delta_trans_ imu_preint_delta_vels_ vo_delta_poses_ the same size
    Eigen::Matrix3d R_c0_c = Eigen::Matrix3d::Identity();
    Eigen::MatrixXd A(6*total_size, 3*total_size + 7);
    Eigen::VectorXd b(6*total_size);
    A.setZero();
    b.setZero();

    for (int i = 0; i < imu_preint_delta_rovs_.size(); i++) {
    
      // 构建位置的预积分方程约束方程
      double dti = imu_preint_dts_[i];
      A.block<3, 3>(6*i, 0) = 0.5 * dti * dti * Eigen::Matrix3d::Identity(); // 0.5 * g * dt^2
      Eigen::Vector3d pij_c0 = R_c0_c * vo_delta_poses_[i].block<3, 1>(0, 3);
      A.block<3, 1>(6*i, 3) = pij_c0; // for scale
      A.block<3, 3>(6*i, 3*i + 4) = -1 * dti * Eigen::Matrix3d::Identity();
      b.segment<3>(6*i) = R_c0_c * imu_preint_delta_trans_[i];

      // 根据速度的预积分创建约束方程
      A.block<3, 3>(6*i+3, 0) = dti * Eigen::Matrix3d::Identity(); // 0.5 * g * dt^2
      A.block<3, 3>(6*i+3, 3*i + 4) = -1 * Eigen::Matrix3d::Identity();
      A.block<3, 3>(6*i+3, 3*i + 7) = Eigen::Matrix3d::Identity();
      b.segment<3>(6*i + 3) = R_c0_c * imu_preint_delta_vels_[i];

      R_c0_c = R_c0_c * imu_preint_delta_rovs_[i];
    }

    result = A.ldlt().solve(b);
  }

  // 3 求解线性方程组，返回最终的结果
  g_dir = result.segment<3>(0);
  s = result(3);
  vs.clear();
  int vel_size = (result.size() - 4) / 3;
  for (int i = 0; i < vel_size; i++) {
    vs.push_back(result.segment<3>(3*i + 4));
  }
  return true;
}

}
// ---------------------------------------------------------------------------------