#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <vector>

namespace lidar_preprocessing_plugins
{
namespace plugins_context_data
{

struct ImuSample
{
    double stamp{0.0};
    double dt{0.0};
    Eigen::Vector3f ang_vel{Eigen::Vector3f::Zero()};
    Eigen::Vector3f lin_accel{Eigen::Vector3f::Zero()};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

using ImuSampleBuffer = std::vector<ImuSample, Eigen::aligned_allocator<ImuSample>>;

struct PreprocessContext
{
    double sweep_ref_time{0.0};
    ImuSampleBuffer imu_samples;
    bool external_velocity_valid{false};
    double external_velocity_stamp{0.0};
    double external_speed_mps{0.0};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace plugins_context_data
}  // namespace lidar_preprocessing_plugins
