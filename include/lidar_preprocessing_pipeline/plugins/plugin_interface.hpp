#pragma once

#include <pcl/point_cloud.h>

#include "lidar_preprocessing_pipeline/dtypes/plugin_params.hpp"
#include "lidar_preprocessing_pipeline/dtypes/preprocess_context.hpp"

namespace lidar_preprocessing_plugins
{

template<typename PointT>
class ILidarPreProcessingPlugin
{
public:
    using PointCloud = pcl::PointCloud<PointT>;

    virtual ~ILidarPreProcessingPlugin() = default;

    /**
     * @brief Initialize the preprocessor with the latest parameters.
     *
     * @param params Parameter bundle shared across preprocessors.
     */
    virtual void initialize(const PreprocessingPluginParams &params) = 0;

    /**
     * @brief Process an input pointcloud into an output pointcloud.
     *
     * @param input Input pointcloud.
     * @param output Output pointcloud.
     */
    virtual void process(const PointCloud &input, PointCloud &output) const = 0;

    /**
     * @brief Process an input pointcloud into an output pointcloud with shared context.
     *
     * @param context Shared preprocessing context (IMU, extrinsics, timing).
     * @param input Input pointcloud.
     * @param output Output pointcloud.
     */
    virtual void process(const plugins_context_data::PreprocessContext &context, const PointCloud &input,
                         PointCloud &output) const
    {
        process(input, output);
    }
};

}  // namespace lidar_preprocessing_plugins
