#pragma once

#include <pcl/point_cloud.h>

#include "lidar_preprocessing_pipeline/dtypes/plugin_params.hpp"
#include "lidar_preprocessing_pipeline/plugins/plugin_interface.hpp"

namespace lidar_preprocessing_plugins
{
template<typename PointT>
class VoxelGridFilterPlugin : public ILidarPreProcessingPlugin<PointT>
{
public:
    using PointCloud = pcl::PointCloud<PointT>;

    VoxelGridFilterPlugin() = default;
    ~VoxelGridFilterPlugin() = default;

    void initialize(const PreprocessingPluginParams &params) override;

    void process(const PointCloud &input, PointCloud &output) const override;

private:
    bool m_enabled{false};
    float m_leaf_size{0.1f};
};
}  // namespace lidar_preprocessing_plugins