#pragma once

#include <pcl/point_cloud.h>

#include "lidar_preprocessing_pipeline/dtypes/plugin_params.hpp"
#include "lidar_preprocessing_pipeline/plugins/plugin_interface.hpp"

namespace lidar_preprocessing_plugins
{
template<typename PointT>
class RadiusOutlierRemovalPlugin : public ILidarPreProcessingPlugin<PointT>
{
public:
    using PointCloud = pcl::PointCloud<PointT>;

    RadiusOutlierRemovalPlugin() = default;
    ~RadiusOutlierRemovalPlugin() = default;

    void initialize(const PreprocessingPluginParams &params) override;

    void process(const PointCloud &input, PointCloud &output) const override;

private:
    bool m_enabled{false};
    double m_radius{0.5};
    int m_min_neighbors{5};
};
}  // namespace lidar_preprocessing_plugins