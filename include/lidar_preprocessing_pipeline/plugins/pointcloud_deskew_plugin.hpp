#pragma once

#include <memory>

#include <pcl/point_cloud.h>

#include "lidar_preprocessing_pipeline/dtypes/plugin_params.hpp"
#include "lidar_preprocessing_pipeline/plugins/plugin_interface.hpp"
#include "lidar_preprocessing_pipeline/utils/imu_integrator.hpp"

namespace lidar_preprocessing_plugins
{
template<typename PointT>
class PointCloudDeskewPlugin : public ILidarPreProcessingPlugin<PointT>
{
public:
    using PointCloud = pcl::PointCloud<PointT>;

    PointCloudDeskewPlugin() = default;
    ~PointCloudDeskewPlugin() = default;

    void initialize(const PreprocessingPluginParams &params) override;

    void process(const PointCloud &input, PointCloud &output) const override;
    void process(const plugins_context_data::PreprocessContext &context, const PointCloud &input,
                 PointCloud &output) const override;
    lidar_preprocessing_utils::imu_integrator::CalibrationGateStatus calibration_gate_status() const;

private:
    static double point_time_seconds(const PointT &point, double sweep_ref_time);

private:
    bool m_enabled{false};
    Eigen::Matrix4f m_imu_to_lidar_transform{Eigen::Matrix4f::Identity()};
    mutable double m_prev_scan_stamp{0.0};

    std::unique_ptr<lidar_preprocessing_utils::ImuIntegrator> m_imu_integrator;
};
}  // namespace lidar_preprocessing_plugins
