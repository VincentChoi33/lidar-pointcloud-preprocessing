#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>

#include <pcl/point_cloud.h>

#include "lidar_preprocessing_pipeline/dtypes/plugin_params.hpp"
#include "lidar_preprocessing_pipeline/plugins/plugin_interface.hpp"
#include "lidar_preprocessing_pipeline/utils/imu_integrator.hpp"

namespace lidar_preprocessing_plugins
{

template<typename PointT>
class GravityAlignPlugin : public ILidarPreProcessingPlugin<PointT>
{
public:
    using PointCloud = pcl::PointCloud<PointT>;

    GravityAlignPlugin() = default;
    ~GravityAlignPlugin() = default;

    void initialize(const PreprocessingPluginParams &params) override;

    void process(const PointCloud &input, PointCloud &output) const override;
    void process(const plugins_context_data::PreprocessContext &context, const PointCloud &input,
                 PointCloud &output) const override;
    bool last_alignment(Eigen::Quaternionf &alignment) const;
    lidar_preprocessing_utils::imu_integrator::CalibrationGateStatus calibration_gate_status() const;

private:
    static double point_time_seconds(const PointT &point, double sweep_ref_time);

    bool m_enabled{false};
    Eigen::Matrix3f m_imu_to_lidar_rotation{Eigen::Matrix3f::Identity()};
    mutable bool m_has_alignment{false};
    mutable Eigen::Quaternionf m_last_alignment{Eigen::Quaternionf::Identity()};
    mutable double m_prev_scan_stamp{0.0};

    std::unique_ptr<lidar_preprocessing_utils::ImuIntegrator> m_imu_integrator;
};

}  // namespace lidar_preprocessing_plugins
