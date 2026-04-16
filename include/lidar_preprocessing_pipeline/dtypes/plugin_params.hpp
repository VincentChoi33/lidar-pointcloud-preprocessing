#pragma once

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <cstdint>

#include "lidar_preprocessing_pipeline/utils/imu_integrator.hpp"

namespace lidar_preprocessing_plugins
{

struct PointCloudDeskewParams
{
    bool enabled{false};
    Eigen::Matrix4f imu_to_lidar_transform{Eigen::Matrix4f::Identity()};
};

struct VoxelGridFilterParams
{
    bool enabled{false};
    double leaf_size{0.1};
};

struct RadiusOutlierRemovalParams
{
    bool enabled{false};
    double radius{0.5};
    int min_neighbors{5};
};

struct LivoxTagFilterParams
{
    bool enabled{false};
    bool remove_high_confidence_noise{false};
    bool remove_moderate_confidence_noise{false};
    bool remove_low_confidence_noise{false};
    bool remove_intensity_noise{false};
    bool output_per_return{false};
};

struct GravityAlignParams
{
    bool enabled{false};
};

struct PreprocessingPluginParams
{
    lidar_preprocessing_utils::imu_integrator::ImuIntegratorParams imu_integrator_params;
    PointCloudDeskewParams pointcloud_deskew_params;
    GravityAlignParams gravity_align_params;
    VoxelGridFilterParams voxel_grid_filter_params;
    RadiusOutlierRemovalParams ror_params;
    LivoxTagFilterParams livox_tag_filter_params;
};

}  // namespace lidar_preprocessing_plugins
