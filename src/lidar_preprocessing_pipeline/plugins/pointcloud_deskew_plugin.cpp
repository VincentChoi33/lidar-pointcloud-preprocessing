#include "lidar_preprocessing_pipeline/plugins/pointcloud_deskew_plugin.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cstdint>
#include <limits>
#include <type_traits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_preprocessing_pipeline/dtypes/plugin_params.hpp"
#include "lidar_preprocessing_pipeline/dtypes/point_types.hpp"
#include "lidar_preprocessing_pipeline/utils/imu_integrator.hpp"

namespace lidar_preprocessing_plugins
{

template<typename PointT>
void PointCloudDeskewPlugin<PointT>::initialize(const PreprocessingPluginParams &params)
{
    m_enabled = params.pointcloud_deskew_params.enabled;
    m_imu_to_lidar_transform = params.pointcloud_deskew_params.imu_to_lidar_transform;
    m_imu_integrator = std::make_unique<lidar_preprocessing_utils::ImuIntegrator>(params.imu_integrator_params);
}

template<typename PointT>
double PointCloudDeskewPlugin<PointT>::point_time_seconds(const PointT &point, double sweep_ref_time)
{
    return lidar_point_types::PointTimeAccessor<PointT>::time_seconds(point, sweep_ref_time);
}


template<typename PointT>
void PointCloudDeskewPlugin<PointT>::process(const PointCloud &input, PointCloud &output) const
{
    plugins_context_data::PreprocessContext context;
    process(context, input, output);
}

template<typename PointT>
void PointCloudDeskewPlugin<PointT>::process(const plugins_context_data::PreprocessContext &context,
                                             const PointCloud &input, PointCloud &output) const
{
    if (!m_enabled || input.empty())
    {
        output = input;
        return;
    }

    if (m_imu_integrator)
    {
        m_imu_integrator->set_external_velocity(context.external_speed_mps, context.external_velocity_stamp,
                                                context.external_velocity_valid);
    }

    if (context.imu_samples.size() < 2)
    {
        output.clear();
        return;
    }

    // Find the time offset b/w the first point and the sweep reference (pointcloud stamped) time
    double last_raw_time = point_time_seconds(input.points.back(), context.sweep_ref_time);
    double time_offset = context.sweep_ref_time - last_raw_time;

    // Extract unique timestamps and their corresponding point indices taking time offset into account
    std::vector<double> timestamps;
    std::vector<std::size_t> unique_time_indices;
    timestamps.reserve(input.points.size());
    unique_time_indices.reserve(input.points.size());
    double last_time = std::numeric_limits<double>::quiet_NaN();
    for (std::size_t i = 0; i < input.points.size(); ++i)
    {
        double stamp = point_time_seconds(input.points[i], context.sweep_ref_time) + time_offset;
        if (timestamps.empty() || stamp != last_time)
        {
            timestamps.push_back(stamp);
            unique_time_indices.push_back(i);
            last_time = stamp;
        }
    }
    unique_time_indices.push_back(input.points.size());

    if (timestamps.empty())
    {
        output.clear();
        return;
    }

    // Use the middle timestamp as the reference pose for relative deskewing.
    std::size_t reference_index = timestamps.size() / 2;

    // Always start integrating from the FIRST timestamp of the CURRENT scan.
    double imu_integration_start_time = timestamps.front();

    // Integrate
    auto result =
            m_imu_integrator->integrate(imu_integration_start_time, timestamps, reference_index, context.imu_samples);
    if (!result.success || result.poses.size() != timestamps.size())
    {
        output.clear();
        return;
    }

    output = input;

    Eigen::Matrix4f T_ref = result.poses[reference_index] * m_imu_to_lidar_transform;
    Eigen::Matrix4f T_ref_inv = T_ref.inverse();

    const auto timestamp_count = static_cast<std::int64_t>(timestamps.size());
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (std::int64_t i = 0; i < timestamp_count; ++i)
    {
        const auto idx = static_cast<std::size_t>(i);
        Eigen::Matrix4f T = result.poses[idx] * m_imu_to_lidar_transform;
        Eigen::Matrix4f T_rel = T_ref_inv * T;

        for (std::size_t k = unique_time_indices[idx]; k < unique_time_indices[idx + 1]; ++k)
        {
            auto &pt = output.points[k];
            // Ensure homogeneous coordinate is 1.0 so rotation/translation applies correctly
            pt.getVector4fMap()[3] = 1.0f;
            pt.getVector4fMap() = T_rel * pt.getVector4fMap();
        }
    }

    output.header = input.header;
    output.header.stamp = timestamps[reference_index];
    output.is_dense = input.is_dense;

    // Preserve the reference IMU state so the next sweep starts from a consistent estimate.
    if (result.reference_state_valid)
    {
        // We must push the calculated state (velocity/bias) back into the
        // integrator so it is preserved for the next scan.
        m_imu_integrator->set_state(result.reference_state);
    }
}

template<typename PointT>
lidar_preprocessing_utils::imu_integrator::CalibrationGateStatus
PointCloudDeskewPlugin<PointT>::calibration_gate_status() const
{
    if (!m_imu_integrator)
        return {};
    return m_imu_integrator->calibration_gate_status();
}

}  // namespace lidar_preprocessing_plugins

// Explicit template instantiations
template class lidar_preprocessing_plugins::PointCloudDeskewPlugin<lidar_point_types::OusterPoint>;
template class lidar_preprocessing_plugins::PointCloudDeskewPlugin<lidar_point_types::VelodynePoint>;
template class lidar_preprocessing_plugins::PointCloudDeskewPlugin<lidar_point_types::LivoxPoint>;
