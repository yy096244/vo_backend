#pragma once

#include <Eigen/Core>
#include <deque>
#include <mutex>
#include <memory>
#include <algorithm>

namespace vobackend
{

// copy from openvins
struct ImuData {

  /// Timestamp of the reading
  double timestamp;

  /// Gyroscope reading, angular velocity (rad/s)
  Eigen::Matrix<double, 3, 1> wm;

  /// Accelerometer reading, linear acceleration (m/s^2)
  Eigen::Matrix<double, 3, 1> am;

  /// Sort function to allow for using of STL containers
  bool operator<(const ImuData &other) const { return timestamp < other.timestamp; }

  static ImuData interpolate_data(const ImuData &imu_1, const ImuData &imu_2, double timestamp) {
    double lambda = (timestamp - imu_1.timestamp) / (imu_2.timestamp - imu_1.timestamp);
    ImuData data;
    data.timestamp = timestamp;
    data.am = (1 - lambda) * imu_1.am + lambda * imu_2.am;
    data.wm = (1 - lambda) * imu_1.wm + lambda * imu_2.wm;
    return data;
  }
};


// ---------------------------------------------------------------------------------
// imudata insert, and get the imudata between [t0, t1]
// thread safe
class ImuBuf {
public:
  ImuBuf(double buf_time = 100) : buf_time_(buf_time) {}
  bool insert_imu(const ImuData& imui);
  std::pair<double, double> get_buf_timebound() const;
  bool get_imus_between(double t1, double t2, std::deque<ImuData>& imu_between);
private:
  double buf_time_;
  mutable std::mutex mutex_;
  std::deque<ImuData> imus_;
};

bool ImuBuf::insert_imu(const ImuData& imui) {
  std::lock_guard<std::mutex> locker(mutex_);

  if (imus_.empty()) {
    imus_.emplace_back(imui);
    return true;
  }

  if (imui.timestamp <= imus_.back().timestamp) {
    return false;
  }

  imus_.emplace_back(imui);

  // auto delete oldest imu data to keep the buf a constant time length
  if (buf_time_ > 0) {
    while (imus_.back().timestamp - imus_.front().timestamp > buf_time_) {
      imus_.pop_front();
    }
  }

  return true;
}

std::pair<double, double> ImuBuf::get_buf_timebound() const {
  std::lock_guard<std::mutex> locker(mutex_);

  if (imus_.empty()) {
    return std::make_pair<double, double>(-1, -1);
  }

  return std::make_pair(imus_.front().timestamp, imus_.back().timestamp);
}

bool ImuBuf::get_imus_between(double t1, double t2, std::deque<ImuData>& imu_between) {
  std::lock_guard<std::mutex> locker(mutex_);

  if (t1 < imus_.front().timestamp || t2 >= imus_.back().timestamp || t1 < t2) {
    return false;
  }

  // binary search to speedup
  ImuData temp_comp;
  temp_comp.timestamp = t1;
  auto it1_upper = std::upper_bound(imus_.begin(), imus_.end(), temp_comp);
  temp_comp.timestamp = t2;
  auto it2_upper = std::upper_bound(imus_.begin(), imus_.end(), temp_comp);

  imu_between.clear();

  // insert first data
  ImuData first_imu_data = ImuData::interpolate_data(*std::prev(it1_upper, 1), *it1_upper, t1);
  imu_between.emplace_back(first_imu_data);

  // insert middle data
  for (auto it = it1_upper; it != it2_upper; it++) {
    imu_between.emplace_back(*it);
  }

  // insert last data
  ImuData last_imu_data = ImuData::interpolate_data(*std::prev(it2_upper, 1), *it2_upper, t2);
  imu_between.emplace_back(last_imu_data);

  return true;
}
// ---------------------------------------------------------------------------------
}