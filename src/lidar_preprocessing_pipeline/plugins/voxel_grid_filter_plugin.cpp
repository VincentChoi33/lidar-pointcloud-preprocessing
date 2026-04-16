
#include "lidar_preprocessing_pipeline/plugins/voxel_grid_filter_plugin.hpp"

#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_preprocessing_pipeline/dtypes/plugin_params.hpp"
#include "lidar_preprocessing_pipeline/dtypes/point_types.hpp"

namespace lidar_preprocessing_plugins
{

template<typename PointT>
void VoxelGridFilterPlugin<PointT>::initialize(const PreprocessingPluginParams &params)
{
    m_enabled = params.voxel_grid_filter_params.enabled;
    m_leaf_size = static_cast<float>(params.voxel_grid_filter_params.leaf_size);
}

template<typename PointT>
void VoxelGridFilterPlugin<PointT>::process(const PointCloud &input, PointCloud &output) const
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

    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(input.makeShared());
    voxel.setLeafSize(m_leaf_size, m_leaf_size, m_leaf_size);
    voxel.filter(output);
    output.header = input.header;
    output.is_dense = input.is_dense;
}

}  // namespace lidar_preprocessing_plugins

// Explicit template instantiations
template class lidar_preprocessing_plugins::VoxelGridFilterPlugin<lidar_point_types::OusterPoint>;
template class lidar_preprocessing_plugins::VoxelGridFilterPlugin<lidar_point_types::VelodynePoint>;
template class lidar_preprocessing_plugins::VoxelGridFilterPlugin<lidar_point_types::LivoxPoint>;
