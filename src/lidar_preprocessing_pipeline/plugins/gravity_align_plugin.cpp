#include "lidar_preprocessing_pipeline/plugins/gravity_align_plugin.hpp"

#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_preprocessing_pipeline/dtypes/plugin_params.hpp"
#include "lidar_preprocessing_pipeline/dtypes/point_types.hpp"
#include "lidar_preprocessing_pipeline/utils/imu_integrator.hpp"

namespace lidar_preprocessing_plugins
{

template<typename PointT>
void GravityAlignPlugin<PointT>::initialize(const PreprocessingPluginParams &params)
{
    m_enabled = params.gravity_align_params.enabled;
    m_imu_to_lidar_rotation = params.pointcloud_deskew_params.imu_to_lidar_transform.block<3, 3>(0, 0);
    m_imu_integrator = std::make_unique<lidar_preprocessing_utils::ImuIntegrator>(params.imu_integrator_params);
}

template<typename PointT>
double GravityAlignPlugin<PointT>::point_time_seconds(const PointT &point, double sweep_ref_time)
{
    return lidar_point_types::PointTimeAccessor<PointT>::time_seconds(point, sweep_ref_time);
}

template<typename PointT>
void GravityAlignPlugin<PointT>::process(const PointCloud &input, PointCloud &output) const
{
    plugins_context_data::PreprocessContext context;
    process(context, input, output);
}

template<typename PointT>
void GravityAlignPlugin<PointT>::process(const plugins_context_data::PreprocessContext &context,
                                         const PointCloud &input, PointCloud &output) const
{
    if (!m_enabled || input.empty())
    {
        output = input;
        m_has_alignment = false;
        return;
    }

    m_has_alignment = false;
    Eigen::Vector3f gravity_lidar = Eigen::Vector3f::Zero();
    bool have_gravity = false;

    if (m_imu_integrator)
    {
        m_imu_integrator->set_external_velocity(context.external_speed_mps, context.external_velocity_stamp,
                                                context.external_velocity_valid);
    }

    if (m_imu_integrator && context.imu_samples.size() >= 2)
    {
        // Find the time offset b/w the first point and the sweep reference (pointcloud stamped) time
        double first_raw_time = point_time_seconds(input.points.front(), context.sweep_ref_time);
        double time_offset = 0.0;
        time_offset = context.sweep_ref_time - first_raw_time;

        std::vector<double> timestamps;
        timestamps.reserve(input.points.size());
        double last_time = std::numeric_limits<double>::quiet_NaN();
        for (std::size_t i = 0; i < input.points.size(); ++i)
        {
            double stamp = point_time_seconds(input.points[i], context.sweep_ref_time) + time_offset;
            if (timestamps.empty() || stamp != last_time)
            {
                timestamps.push_back(stamp);
                last_time = stamp;
            }
        }

        if (timestamps.empty())
        {
            output.clear();
            return;
        }

        std::size_t reference_index = timestamps.size() / 2;
        if (reference_index + 1 < timestamps.size())
            timestamps.resize(reference_index + 1);

        // Calculate orientation at the median timestamp.
        auto result =
                m_imu_integrator->integrate(context.sweep_ref_time, timestamps, reference_index, context.imu_samples);

        if (result.success && result.poses.size() == timestamps.size())
        {
            const Eigen::Matrix3f imu_to_world = result.poses[reference_index].block<3, 3>(0, 0);

            // Extract Gravity Vector (Z-axis of world in IMU frame)
            const Eigen::Vector3f gravity_imu = imu_to_world.transpose() * Eigen::Vector3f::UnitZ();

            // Transform to Lidar Frame
            gravity_lidar = m_imu_to_lidar_rotation * gravity_imu;

            have_gravity = gravity_lidar.norm() > std::numeric_limits<float>::epsilon();
        }
    }

    if (!have_gravity)
    {
        output.clear();
        return;
    }

    // Compute Alignment Rotation
    Eigen::Vector3f gravity_dir = gravity_lidar.normalized();

    // Ensure we are aligning the "Down" vector to -Z (or Up to +Z)
    // If the vector points generally down, flip it to point up so we can align to UnitZ
    if (gravity_dir.dot(Eigen::Vector3f::UnitZ()) < 0.0f)
        gravity_dir = -gravity_dir;

    const Eigen::Quaternionf align_q = Eigen::Quaternionf::FromTwoVectors(gravity_dir, Eigen::Vector3f::UnitZ());

    m_last_alignment = align_q;
    m_has_alignment = true;

    Eigen::Matrix4f T_align = Eigen::Matrix4f::Identity();
    T_align.block<3, 3>(0, 0) = align_q.toRotationMatrix();

    // Apply Rotation
    output = input;
#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
    for (std::int64_t i = 0; i < static_cast<std::int64_t>(output.points.size()); ++i)
    {
        auto &pt = output.points[static_cast<std::size_t>(i)];
        // Ensure homogeneous coordinate is 1.0 so rotation/translation applies correctly
        pt.getVector4fMap()[3] = 1.0f;
        pt.getVector4fMap() = T_align * pt.getVector4fMap();
    }

    output.header = input.header;
    output.is_dense = input.is_dense;
}

template<typename PointT>
bool GravityAlignPlugin<PointT>::last_alignment(Eigen::Quaternionf &alignment) const
{
    if (!m_has_alignment)
        return false;
    alignment = m_last_alignment;
    return true;
}

template<typename PointT>
lidar_preprocessing_utils::imu_integrator::CalibrationGateStatus
GravityAlignPlugin<PointT>::calibration_gate_status() const
{
    if (!m_imu_integrator)
        return {};
    return m_imu_integrator->calibration_gate_status();
}

}  // namespace lidar_preprocessing_plugins

// Explicit template instantiations
template class lidar_preprocessing_plugins::GravityAlignPlugin<lidar_point_types::OusterPoint>;
template class lidar_preprocessing_plugins::GravityAlignPlugin<lidar_point_types::VelodynePoint>;
template class lidar_preprocessing_plugins::GravityAlignPlugin<lidar_point_types::LivoxPoint>;
