
#include "lidar_preprocessing_pipeline/plugins/radius_outlier_removal_plugin.hpp"

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_preprocessing_pipeline/dtypes/plugin_params.hpp"
#include "lidar_preprocessing_pipeline/dtypes/point_types.hpp"

namespace lidar_preprocessing_plugins
{

template<typename PointT>
void RadiusOutlierRemovalPlugin<PointT>::initialize(const PreprocessingPluginParams &params)
{
    m_enabled = params.ror_params.enabled;
    m_radius = params.ror_params.radius;
    m_min_neighbors = params.ror_params.min_neighbors;
}

template<typename PointT>
void RadiusOutlierRemovalPlugin<PointT>::process(const PointCloud &input, PointCloud &output) const
{
    if (!m_enabled)
    {
        output = input;
        return;
    }
    if (input.empty())
    {
        output = input;
        return;
    }

    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(input.makeShared());
    ror.setRadiusSearch(m_radius);
    ror.setMinNeighborsInRadius(m_min_neighbors);
    ror.filter(output);
    output.header = input.header;
    output.is_dense = input.is_dense;
}

}  // namespace lidar_preprocessing_plugins

// Explicit template instantiations
template class lidar_preprocessing_plugins::RadiusOutlierRemovalPlugin<lidar_point_types::OusterPoint>;
template class lidar_preprocessing_plugins::RadiusOutlierRemovalPlugin<lidar_point_types::VelodynePoint>;
template class lidar_preprocessing_plugins::RadiusOutlierRemovalPlugin<lidar_point_types::LivoxPoint>;
