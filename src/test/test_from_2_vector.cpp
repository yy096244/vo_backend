#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>

int main(int argc, char** argv) {
    std::cout << "test eigen form 2 vector" << std::endl;
    Eigen::Quaterniond q_w_i = Eigen::AngleAxisd(10/57.3, Eigen::Vector3d{0, 0, 1})
        * Eigen::AngleAxisd(20/57.3, Eigen::Vector3d{0, 1, 0})
        * Eigen::AngleAxisd(50/57.3, Eigen::Vector3d{1, 0, 0});

    // 给定重力坐标系，经过q_w_i来进行旋转，得到在imu坐标下的ector，然后根据eigen的库提取对应的结果
    Eigen::Vector3d g_w{0, 0, 1};
    Eigen::Vector3d g_i = q_w_i.inverse() * g_w;

    Eigen::Quaterniond q_compute = Eigen::Quaterniond::FromTwoVectors(g_i, g_w);
    Eigen::Vector3d g_i_compute = Eigen::Matrix3d(q_compute.inverse()) * Eigen::Vector3d{0,0,1};

    std::cout << "q_w_i : \t\t" << q_w_i.coeffs().transpose() << std::endl;
    std::cout << "q_compute : \t\t" << q_compute.coeffs().transpose() << std::endl;

    std::cout << "g_i : " << g_i.transpose() << std::endl;
    std::cout << "g_i_compute : " << g_i_compute.transpose() << std::endl;

    // 不能保证q完全相同，但是可以保证旋转后的矢量是一致的
    Eigen::Vector3d xyz = Eigen::Matrix3d(q_w_i).eulerAngles(2, 1, 0);
    std::cout << "xyz : " << xyz.transpose() * 57.3 << std::endl;
    Eigen::Vector3d xyz_compute = Eigen::Matrix3d(q_compute).eulerAngles(2, 1, 0);
    std::cout << "xyz_compute : " << xyz_compute.transpose() * 57.3 << std::endl;

    // 测试将Z轴角度置为0
    Eigen::Quaterniond q_w_i_yaw0 = Eigen::AngleAxisd(0, Eigen::Vector3d{0, 0, 1})
        * Eigen::AngleAxisd(xyz_compute(1), Eigen::Vector3d{0, 1, 0})
        * Eigen::AngleAxisd(xyz_compute(2), Eigen::Vector3d{1, 0, 0});
    std::cout << "q_w_i_yaw0 : " << q_w_i_yaw0.coeffs().transpose() << std::endl;
    Eigen::Vector3d g_i_yaw0 = Eigen::Matrix3d(q_w_i_yaw0.inverse()) * Eigen::Vector3d{0,0,1};
    std::cout << "g_i_yaw0 : " << g_i_yaw0.transpose() << std::endl;

    // 测试结果 ： 绕zyx旋转，z轴旋转的角度不保证，但是yx的角度可以保证

    return 0;
}