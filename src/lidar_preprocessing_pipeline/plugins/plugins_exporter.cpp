#include <pcl/point_types.h>
#include <pluginlib/class_list_macros.h>

#include "lidar_preprocessing_pipeline/dtypes/point_types.hpp"
#include "lidar_preprocessing_pipeline/plugins/gravity_align_plugin.hpp"
#include "lidar_preprocessing_pipeline/plugins/livox_tag_filter_plugin.hpp"
#include "lidar_preprocessing_pipeline/plugins/plugin_interface.hpp"
#include "lidar_preprocessing_pipeline/plugins/pointcloud_deskew_plugin.hpp"
#include "lidar_preprocessing_pipeline/plugins/radius_outlier_removal_plugin.hpp"
#include "lidar_preprocessing_pipeline/plugins/voxel_grid_filter_plugin.hpp"

namespace lidar_preprocessing_plugins
{
using OusterPointType = lidar_point_types::OusterPoint;
using OusterPointCloudDeskewPlugin = PointCloudDeskewPlugin<OusterPointType>;
using OusterGravityAlignPlugin = GravityAlignPlugin<OusterPointType>;
using OusterVoxelGridFilterPlugin = VoxelGridFilterPlugin<OusterPointType>;
using OusterRadiusOutlierRemovalPlugin = RadiusOutlierRemovalPlugin<OusterPointType>;
using OusterPreprocessingPlugins = ILidarPreProcessingPlugin<OusterPointType>;

using VelodynePointType = lidar_point_types::VelodynePoint;
using VelodynePointCloudDeskewPlugin = PointCloudDeskewPlugin<VelodynePointType>;
using VelodyneGravityAlignPlugin = GravityAlignPlugin<VelodynePointType>;
using VelodyneVoxelGridFilterPlugin = VoxelGridFilterPlugin<VelodynePointType>;
using VelodyneRadiusOutlierRemovalPlugin = RadiusOutlierRemovalPlugin<VelodynePointType>;
using VelodynePreprocessingPlugins = ILidarPreProcessingPlugin<VelodynePointType>;

using LivoxPointType = lidar_point_types::LivoxPoint;
using LivoxPointCloudDeskewPlugin = PointCloudDeskewPlugin<LivoxPointType>;
using LivoxGravityAlignPlugin = GravityAlignPlugin<LivoxPointType>;
using LivoxVoxelGridFilterPlugin = VoxelGridFilterPlugin<LivoxPointType>;
using LivoxRadiusOutlierRemovalPlugin = RadiusOutlierRemovalPlugin<LivoxPointType>;
using LivoxTagFilterPluginType = LivoxTagFilterPlugin<LivoxPointType>;
using LivoxPreprocessingPlugins = ILidarPreProcessingPlugin<LivoxPointType>;
}  // namespace lidar_preprocessing_plugins

// Ouster Pre-processing Plugins
PLUGINLIB_EXPORT_CLASS(lidar_preprocessing_plugins::OusterPointCloudDeskewPlugin,
                       lidar_preprocessing_plugins::OusterPreprocessingPlugins)
PLUGINLIB_EXPORT_CLASS(lidar_preprocessing_plugins::OusterGravityAlignPlugin,
                       lidar_preprocessing_plugins::OusterPreprocessingPlugins)
PLUGINLIB_EXPORT_CLASS(lidar_preprocessing_plugins::OusterVoxelGridFilterPlugin,
                       lidar_preprocessing_plugins::OusterPreprocessingPlugins)
PLUGINLIB_EXPORT_CLASS(lidar_preprocessing_plugins::OusterRadiusOutlierRemovalPlugin,
                       lidar_preprocessing_plugins::OusterPreprocessingPlugins)

// Velodyne pre-processing plugins
PLUGINLIB_EXPORT_CLASS(lidar_preprocessing_plugins::VelodynePointCloudDeskewPlugin,
                       lidar_preprocessing_plugins::VelodynePreprocessingPlugins)
PLUGINLIB_EXPORT_CLASS(lidar_preprocessing_plugins::VelodyneGravityAlignPlugin,
                       lidar_preprocessing_plugins::VelodynePreprocessingPlugins)
PLUGINLIB_EXPORT_CLASS(lidar_preprocessing_plugins::VelodyneVoxelGridFilterPlugin,
                       lidar_preprocessing_plugins::VelodynePreprocessingPlugins)
PLUGINLIB_EXPORT_CLASS(lidar_preprocessing_plugins::VelodyneRadiusOutlierRemovalPlugin,
                       lidar_preprocessing_plugins::VelodynePreprocessingPlugins)

// Livox pre-processing plugins
PLUGINLIB_EXPORT_CLASS(lidar_preprocessing_plugins::LivoxPointCloudDeskewPlugin,
                       lidar_preprocessing_plugins::LivoxPreprocessingPlugins)
PLUGINLIB_EXPORT_CLASS(lidar_preprocessing_plugins::LivoxGravityAlignPlugin,
                       lidar_preprocessing_plugins::LivoxPreprocessingPlugins)
PLUGINLIB_EXPORT_CLASS(lidar_preprocessing_plugins::LivoxVoxelGridFilterPlugin,
                       lidar_preprocessing_plugins::LivoxPreprocessingPlugins)
PLUGINLIB_EXPORT_CLASS(lidar_preprocessing_plugins::LivoxRadiusOutlierRemovalPlugin,
                       lidar_preprocessing_plugins::LivoxPreprocessingPlugins)
PLUGINLIB_EXPORT_CLASS(lidar_preprocessing_plugins::LivoxTagFilterPluginType,
                       lidar_preprocessing_plugins::LivoxPreprocessingPlugins)
