#include "lidar_preprocessing_nodelet.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <exception>
#include <iterator>
#include <limits>
#include <sstream>
#include <type_traits>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pluginlib/class_list_macros.h>
#include <pluginlib/class_loader.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <boost/bind.hpp>

#include "ros_utils.h"

#include "lidar_preprocessing_pipeline/dtypes/point_types.hpp"
#include "lidar_preprocessing_pipeline/plugins/gravity_align_plugin.hpp"
#include "lidar_preprocessing_pipeline/plugins/livox_tag_filter_plugin.hpp"
#include "lidar_preprocessing_pipeline/plugins/pointcloud_deskew_plugin.hpp"

namespace lidar_preprocessing_pipeline
{
template<>
const char *LidarPreProcessingNodelet<lidar_point_types::OusterPoint>::preprocessor_base_class_name()
{
    return "lidar_preprocessing_plugins::OusterPreprocessingPlugins";
}

template<>
const char *LidarPreProcessingNodelet<lidar_point_types::VelodynePoint>::preprocessor_base_class_name()
{
    return "lidar_preprocessing_plugins::VelodynePreprocessingPlugins";
}

template<>
const char *LidarPreProcessingNodelet<lidar_point_types::LivoxPoint>::preprocessor_base_class_name()
{
    return "lidar_preprocessing_plugins::LivoxPreprocessingPlugins";
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::onInit()
{
    m_node_handle = getMTNodeHandle();
    m_private_node_handle = getMTPrivateNodeHandle();
    m_tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>();
    m_tf_listener = std::make_unique<tf2_ros::TransformListener>(m_tf_buffer);
    m_gravity_parent_frame = kGravityParentFrameBase;
    std::string ns = m_node_handle.getNamespace();
    if (!ns.empty() && ns != "/")
    {
        if (ns.front() != '/')
            ns = "/" + ns;
        m_gravity_parent_frame = (ns + "/" + kGravityParentFrameBase);
        if (!m_gravity_parent_frame.empty() && m_gravity_parent_frame.front() == '/')
            m_gravity_parent_frame.erase(0, 1);
    }

    load_params();
    initialize_dynamic_reconfigure();

    init_publishers();
    init_subscribers();

    NODELET_INFO_STREAM("LidarPreProcessingNodelet initialized");
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::load_params()
{
    load_core_params(m_private_node_handle);
    load_gravity_align_params(m_private_node_handle);
    load_deskew_params(m_private_node_handle);
    load_shared_imu_params(m_private_node_handle);
    load_voxel_grid_params(m_private_node_handle);
    load_ror_params(m_private_node_handle);
    load_livox_tag_params(m_private_node_handle);
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::load_core_params(const ros::NodeHandle &nh)
{
    // Core parameters are grouped under the "core" namespace in the YAML config.
    ros::NodeHandle core_params(nh, "core");
    ros_utils::get_optional_param("input_topic", m_input_topic, std::string("points"), core_params);
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::load_gravity_align_params(const ros::NodeHandle &nh)
{
    ros::NodeHandle gravity_params(nh, "gravity_align");
    ros_utils::get_optional_param("enabled", m_plugin_params.gravity_align_params.enabled, false, gravity_params);
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::load_deskew_params(const ros::NodeHandle &nh)
{
    ros::NodeHandle deskew_params(nh, "pointcloud_deskew");
    ros_utils::get_optional_param("enabled", m_plugin_params.pointcloud_deskew_params.enabled, false, deskew_params);
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::load_shared_imu_params(const ros::NodeHandle &nh)
{
    ros::NodeHandle imu_params(nh, "imu_params");
    const bool needs_imu =
            m_plugin_params.pointcloud_deskew_params.enabled || m_plugin_params.gravity_align_params.enabled;
    if (!needs_imu)
    {
        m_imu_topic.clear();
        m_plugin_params.pointcloud_deskew_params.imu_to_lidar_transform = Eigen::Matrix4f::Identity();
        return;
    }

    load_imu_params(imu_params);
    load_imu_integration_params(imu_params);
    load_imu_calibration_params(imu_params);
    load_extrinsics_params(ros::NodeHandle(nh, "pointcloud_deskew"));
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::load_imu_params(const ros::NodeHandle &nh)
{
    ros_utils::get_mandatory_param("topic", m_imu_topic, nh);
    ros_utils::get_optional_param("buffer_size", m_imu_buffer_duration, 2.0, nh);
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::load_imu_integration_params(const ros::NodeHandle &nh)
{
    ros::NodeHandle integration_root(nh, "integration");
    ros::NodeHandle integration_params(integration_root, "integration_params");
    auto &imu_params = m_plugin_params.imu_integrator_params;
    ros_utils::get_optional_param("approximate_gravity", imu_params.approximate_gravity, imu_params.approximate_gravity,
                                  integration_params);
    double gravity_mps2 = imu_params.gravity_mps2;
    ros_utils::get_optional_param("gravity_mps2", gravity_mps2, gravity_mps2, integration_params);
    imu_params.gravity_mps2 = static_cast<float>(gravity_mps2);
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::load_imu_calibration_params(const ros::NodeHandle &nh)
{
    ros::NodeHandle calibration_params(nh, "calibration");
    auto &calibration = m_plugin_params.imu_integrator_params.imu_calibration_params;
    ros_utils::get_optional_param("enabled", calibration.enabled, calibration.enabled, calibration_params);

    int calibration_time = calibration.calibration_time_sec;
    ros_utils::get_optional_param("time", calibration_time, calibration_time, calibration_params);
    calibration.calibration_time_sec = static_cast<std::uint8_t>(
            std::clamp(calibration_time, 0, static_cast<int>(std::numeric_limits<std::uint8_t>::max())));

    ros::NodeHandle external_velocity_params(calibration_params, "external_velocity");
    double max_speed = calibration.external_velocity.max_speed_mps;
    ros_utils::get_optional_param("max_speed_mps", max_speed, max_speed, external_velocity_params);
    calibration.external_velocity.max_speed_mps = static_cast<float>(std::max(0.0, max_speed));
    double max_age = calibration.external_velocity.max_age_sec;
    ros_utils::get_optional_param("max_age_sec", max_age, max_age, external_velocity_params);
    calibration.external_velocity.max_age_sec = std::max(0.0, max_age);

    ros::NodeHandle gyro_params(calibration_params, "gyro");
    std::vector<double> gyro_bias = {calibration.gyro_bias.x(), calibration.gyro_bias.y(), calibration.gyro_bias.z()};
    ros_utils::get_optional_param("bias", gyro_bias, gyro_bias, gyro_params);
    if (gyro_bias.size() == 3)
    {
        calibration.gyro_bias = Eigen::Vector3f(static_cast<float>(gyro_bias[0]), static_cast<float>(gyro_bias[1]),
                                                static_cast<float>(gyro_bias[2]));
    }
    else
    {
        NODELET_WARN_STREAM("Parameter 'imu_params/calibration/gyro/bias' invalid. "
                            "Using existing values.");
    }

    ros::NodeHandle accel_params(calibration_params, "accel");
    std::vector<double> accel_bias = {calibration.accel_bias.x(), calibration.accel_bias.y(),
                                      calibration.accel_bias.z()};
    ros_utils::get_optional_param("bias", accel_bias, accel_bias, accel_params);
    if (accel_bias.size() == 3)
    {
        calibration.accel_bias = Eigen::Vector3f(static_cast<float>(accel_bias[0]), static_cast<float>(accel_bias[1]),
                                                 static_cast<float>(accel_bias[2]));
    }
    else
    {
        NODELET_WARN_STREAM("Parameter 'imu_params/calibration/accel/bias' invalid. "
                            "Using existing values.");
    }

    std::vector<double> accel_sm = {calibration.imu_accel_smatrix(0, 0), calibration.imu_accel_smatrix(0, 1),
                                    calibration.imu_accel_smatrix(0, 2), calibration.imu_accel_smatrix(1, 0),
                                    calibration.imu_accel_smatrix(1, 1), calibration.imu_accel_smatrix(1, 2),
                                    calibration.imu_accel_smatrix(2, 0), calibration.imu_accel_smatrix(2, 1),
                                    calibration.imu_accel_smatrix(2, 2)};
    ros_utils::get_optional_param("sm", accel_sm, accel_sm, accel_params);
    if (accel_sm.size() == 9)
    {
        calibration.imu_accel_smatrix << static_cast<float>(accel_sm[0]), static_cast<float>(accel_sm[1]),
                static_cast<float>(accel_sm[2]), static_cast<float>(accel_sm[3]), static_cast<float>(accel_sm[4]),
                static_cast<float>(accel_sm[5]), static_cast<float>(accel_sm[6]), static_cast<float>(accel_sm[7]),
                static_cast<float>(accel_sm[8]);
    }
    else
    {
        NODELET_WARN_STREAM("Parameter 'imu_params/calibration/accel/sm' invalid. "
                            "Using existing values.");
    }
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::load_extrinsics_params(const ros::NodeHandle &nh)
{
    std::vector<double> lidar_to_imu_translation;
    std::vector<double> lidar_to_imu_rotation;
    ros_utils::get_optional_param("lidar_to_imu_translation", lidar_to_imu_translation, std::vector<double>{}, nh);
    ros_utils::get_optional_param("lidar_to_imu_rotation", lidar_to_imu_rotation, std::vector<double>{}, nh);
    if (lidar_to_imu_translation.size() == 3 && lidar_to_imu_rotation.size() == 9)
    {
        Eigen::Matrix3f lidar_to_imu_R;
        lidar_to_imu_R << static_cast<float>(lidar_to_imu_rotation[0]), static_cast<float>(lidar_to_imu_rotation[1]),
                static_cast<float>(lidar_to_imu_rotation[2]), static_cast<float>(lidar_to_imu_rotation[3]),
                static_cast<float>(lidar_to_imu_rotation[4]), static_cast<float>(lidar_to_imu_rotation[5]),
                static_cast<float>(lidar_to_imu_rotation[6]), static_cast<float>(lidar_to_imu_rotation[7]),
                static_cast<float>(lidar_to_imu_rotation[8]);
        Eigen::Vector3f lidar_to_imu_t(static_cast<float>(lidar_to_imu_translation[0]),
                                       static_cast<float>(lidar_to_imu_translation[1]),
                                       static_cast<float>(lidar_to_imu_translation[2]));

        Eigen::Matrix4f lidar_to_imu = Eigen::Matrix4f::Identity();
        lidar_to_imu.block<3, 3>(0, 0) = lidar_to_imu_R;
        lidar_to_imu.block<3, 1>(0, 3) = lidar_to_imu_t;
        m_plugin_params.pointcloud_deskew_params.imu_to_lidar_transform = lidar_to_imu.inverse();
    }
    else
    {
        m_plugin_params.pointcloud_deskew_params.imu_to_lidar_transform = Eigen::Matrix4f::Identity();
        NODELET_WARN_STREAM("Parameter 'pointcloud_deskew/lidar_to_imu_translation' or "
                            "'pointcloud_deskew/lidar_to_imu_rotation' missing or invalid. "
                            "Using identity transform.");
    }
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::load_voxel_grid_params(const ros::NodeHandle &nh)
{
    ros::NodeHandle voxel_filter_params(nh, "voxel_grid_filter");
    ros_utils::get_optional_param("enabled", m_plugin_params.voxel_grid_filter_params.enabled, false,
                                  voxel_filter_params);
    ros_utils::get_optional_param("leaf_size", m_plugin_params.voxel_grid_filter_params.leaf_size, 0.1,
                                  voxel_filter_params);
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::load_ror_params(const ros::NodeHandle &nh)
{
    ros::NodeHandle ror_params(nh, "radius_outlier_removal");
    ros_utils::get_optional_param("enabled", m_plugin_params.ror_params.enabled, false, ror_params);
    ros_utils::get_optional_param("radius", m_plugin_params.ror_params.radius, 0.5, ror_params);
    ros_utils::get_optional_param("min_neighbors", m_plugin_params.ror_params.min_neighbors, 5, ror_params);
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::load_livox_tag_params(const ros::NodeHandle &nh)
{
    ros::NodeHandle livox_params(nh, "livox_tag_filter");
    ros_utils::get_optional_param("enabled", m_plugin_params.livox_tag_filter_params.enabled, false, livox_params);
    ros_utils::get_optional_param("remove_high_confidence_noise",
                                  m_plugin_params.livox_tag_filter_params.remove_high_confidence_noise, false,
                                  livox_params);
    ros_utils::get_optional_param("remove_moderate_confidence_noise",
                                  m_plugin_params.livox_tag_filter_params.remove_moderate_confidence_noise, false,
                                  livox_params);
    ros_utils::get_optional_param("remove_low_confidence_noise",
                                  m_plugin_params.livox_tag_filter_params.remove_low_confidence_noise, false,
                                  livox_params);
    ros_utils::get_optional_param("remove_intensity_noise",
                                  m_plugin_params.livox_tag_filter_params.remove_intensity_noise, false, livox_params);
    ros_utils::get_optional_param("output_per_return", m_plugin_params.livox_tag_filter_params.output_per_return, false,
                                  livox_params);
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::initialize_dynamic_reconfigure()
{
    m_reconfig_server = std::make_shared<dynamic_reconfigure::Server<PreprocessingParamsConfig>>(m_reconfig_mutex,
                                                                                                 m_private_node_handle);

    PreprocessingParamsConfig config;
    m_reconfig_server->getConfigDefault(config);

    // Set initial values from loaded params
    // PointCloud Deskew
    config.pointcloud_deskew_enabled = m_plugin_params.pointcloud_deskew_params.enabled;
    config.pointcloud_deskew_approximate_gravity = m_plugin_params.imu_integrator_params.approximate_gravity;
    config.gravity_align_enabled = m_plugin_params.gravity_align_params.enabled;

    // Voxel Grid Filter
    config.voxel_grid_filter_enabled = m_plugin_params.voxel_grid_filter_params.enabled;
    config.voxel_grid_filter_leaf_size = m_plugin_params.voxel_grid_filter_params.leaf_size;

    // Radius Outlier Removal
    config.radius_outlier_removal_enabled = m_plugin_params.ror_params.enabled;
    config.radius_outlier_removal_radius = m_plugin_params.ror_params.radius;
    config.radius_outlier_removal_min_neighbors = m_plugin_params.ror_params.min_neighbors;
    config.livox_tag_filter_enabled = m_plugin_params.livox_tag_filter_params.enabled;
    config.livox_tag_filter_remove_high_confidence_noise =
            m_plugin_params.livox_tag_filter_params.remove_high_confidence_noise;
    config.livox_tag_filter_remove_moderate_confidence_noise =
            m_plugin_params.livox_tag_filter_params.remove_moderate_confidence_noise;
    config.livox_tag_filter_remove_low_confidence_noise =
            m_plugin_params.livox_tag_filter_params.remove_low_confidence_noise;
    config.livox_tag_filter_remove_intensity_noise = m_plugin_params.livox_tag_filter_params.remove_intensity_noise;
    config.livox_tag_filter_output_per_return = m_plugin_params.livox_tag_filter_params.output_per_return;

    m_reconfig_server->updateConfig(config);

    dynamic_reconfigure::Server<PreprocessingParamsConfig>::CallbackType f;
    f = boost::bind(&LidarPreProcessingNodelet::reconfigure_callback, this, _1, _2);
    m_reconfig_server->setCallback(f);
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::reconfigure_callback(PreprocessingParamsConfig &config, uint32_t /*level*/)
{
    std::lock_guard lock(m_callback_mutex);

    const auto prev_params = m_plugin_params;

    apply_reconfigure_config(config);

    const auto flags = compute_param_change_flags(prev_params);

    update_output_publisher();
    update_livox_return_publishers();

    const auto plugins_to_init = build_plugins_to_init(flags);
    apply_plugin_updates(config, flags, plugins_to_init);

    log_reconfigure_changes(prev_params);
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::apply_reconfigure_config(const PreprocessingParamsConfig &config)
{
    m_plugin_params.pointcloud_deskew_params.enabled = config.pointcloud_deskew_enabled;
    m_plugin_params.imu_integrator_params.approximate_gravity = config.pointcloud_deskew_approximate_gravity;
    m_plugin_params.gravity_align_params.enabled = config.gravity_align_enabled;

    m_plugin_params.voxel_grid_filter_params.enabled = config.voxel_grid_filter_enabled;
    m_plugin_params.voxel_grid_filter_params.leaf_size = config.voxel_grid_filter_leaf_size;

    m_plugin_params.ror_params.enabled = config.radius_outlier_removal_enabled;
    m_plugin_params.ror_params.radius = config.radius_outlier_removal_radius;
    m_plugin_params.ror_params.min_neighbors = config.radius_outlier_removal_min_neighbors;

    m_plugin_params.livox_tag_filter_params.enabled = config.livox_tag_filter_enabled;
    m_plugin_params.livox_tag_filter_params.remove_high_confidence_noise =
            config.livox_tag_filter_remove_high_confidence_noise;
    m_plugin_params.livox_tag_filter_params.remove_moderate_confidence_noise =
            config.livox_tag_filter_remove_moderate_confidence_noise;
    m_plugin_params.livox_tag_filter_params.remove_low_confidence_noise =
            config.livox_tag_filter_remove_low_confidence_noise;
    m_plugin_params.livox_tag_filter_params.remove_intensity_noise = config.livox_tag_filter_remove_intensity_noise;
    m_plugin_params.livox_tag_filter_params.output_per_return = config.livox_tag_filter_output_per_return;
}

template<typename PointT>
typename LidarPreProcessingNodelet<PointT>::ParamChangeFlags
LidarPreProcessingNodelet<PointT>::compute_param_change_flags(
        const lidar_preprocessing_plugins::PreprocessingPluginParams &prev_params) const
{
    ParamChangeFlags flags;

    flags.deskew_enabled_changed =
            prev_params.pointcloud_deskew_params.enabled != m_plugin_params.pointcloud_deskew_params.enabled;
    flags.gravity_enabled_changed =
            prev_params.gravity_align_params.enabled != m_plugin_params.gravity_align_params.enabled;
    flags.voxel_enabled_changed =
            prev_params.voxel_grid_filter_params.enabled != m_plugin_params.voxel_grid_filter_params.enabled;
    flags.ror_enabled_changed = prev_params.ror_params.enabled != m_plugin_params.ror_params.enabled;
    flags.livox_enabled_changed =
            prev_params.livox_tag_filter_params.enabled != m_plugin_params.livox_tag_filter_params.enabled;
    flags.enable_flags_changed = flags.deskew_enabled_changed || flags.gravity_enabled_changed ||
                                 flags.voxel_enabled_changed || flags.ror_enabled_changed ||
                                 flags.livox_enabled_changed;

    flags.approx_gravity_changed = prev_params.imu_integrator_params.approximate_gravity !=
                                   m_plugin_params.imu_integrator_params.approximate_gravity;
    flags.voxel_params_changed =
            prev_params.voxel_grid_filter_params.leaf_size != m_plugin_params.voxel_grid_filter_params.leaf_size;
    flags.ror_params_changed = prev_params.ror_params.radius != m_plugin_params.ror_params.radius ||
                               prev_params.ror_params.min_neighbors != m_plugin_params.ror_params.min_neighbors;
    flags.livox_params_changed = prev_params.livox_tag_filter_params.remove_high_confidence_noise !=
                                         m_plugin_params.livox_tag_filter_params.remove_high_confidence_noise ||
                                 prev_params.livox_tag_filter_params.remove_moderate_confidence_noise !=
                                         m_plugin_params.livox_tag_filter_params.remove_moderate_confidence_noise ||
                                 prev_params.livox_tag_filter_params.remove_low_confidence_noise !=
                                         m_plugin_params.livox_tag_filter_params.remove_low_confidence_noise ||
                                 prev_params.livox_tag_filter_params.remove_intensity_noise !=
                                         m_plugin_params.livox_tag_filter_params.remove_intensity_noise ||
                                 prev_params.livox_tag_filter_params.output_per_return !=
                                         m_plugin_params.livox_tag_filter_params.output_per_return;

    return flags;
}

template<typename PointT>
std::vector<std::string> LidarPreProcessingNodelet<PointT>::build_plugins_to_init(const ParamChangeFlags &flags) const
{
    std::vector<std::string> plugins_to_init;
    if (m_plugin_params.pointcloud_deskew_params.enabled &&
        (flags.deskew_enabled_changed || flags.approx_gravity_changed))
    {
        plugins_to_init.emplace_back(kDeskewKey);
    }
    if (m_plugin_params.gravity_align_params.enabled && (flags.gravity_enabled_changed || flags.approx_gravity_changed))
    {
        plugins_to_init.emplace_back(kGravityAlignKey);
    }
    if (m_plugin_params.voxel_grid_filter_params.enabled && (flags.voxel_enabled_changed || flags.voxel_params_changed))
    {
        plugins_to_init.emplace_back(kVoxelKey);
    }
    if (m_plugin_params.ror_params.enabled && (flags.ror_enabled_changed || flags.ror_params_changed))
    {
        plugins_to_init.emplace_back(kRorKey);
    }
    if (m_plugin_params.livox_tag_filter_params.enabled && (flags.livox_enabled_changed || flags.livox_params_changed))
    {
        plugins_to_init.emplace_back(kLivoxTagKey);
    }

    return plugins_to_init;
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::apply_plugin_updates(const PreprocessingParamsConfig &config,
                                                             const ParamChangeFlags &flags,
                                                             const std::vector<std::string> &plugins_to_init)
{
    if (!m_plugins_initialized)
    {
        update_enabled_plugins(config);
        load_plugins(m_enabled_plugins);
        m_plugins_initialized = true;
        return;
    }

    if (flags.enable_flags_changed)
    {
        update_enabled_plugins(config);
    }

    if (flags.enable_flags_changed || !plugins_to_init.empty())
    {
        load_plugins(plugins_to_init);
    }
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::log_reconfigure_changes(
        const lidar_preprocessing_plugins::PreprocessingPluginParams &prev_params) const
{
    std::ostringstream changes;
    changes << std::boolalpha;
    bool has_changes = false;
    auto note_change = [&](const std::string &label, const auto &before, const auto &after) {
        if (before == after)
            return;
        if (has_changes)
            changes << ", ";
        changes << label << ": " << before << " -> " << after;
        has_changes = true;
    };

    note_change("deskew.enabled", prev_params.pointcloud_deskew_params.enabled,
                m_plugin_params.pointcloud_deskew_params.enabled);
    note_change("deskew.approx_gravity", prev_params.imu_integrator_params.approximate_gravity,
                m_plugin_params.imu_integrator_params.approximate_gravity);
    note_change("gravity_align.enabled", prev_params.gravity_align_params.enabled,
                m_plugin_params.gravity_align_params.enabled);
    note_change("voxel.enabled", prev_params.voxel_grid_filter_params.enabled,
                m_plugin_params.voxel_grid_filter_params.enabled);
    note_change("voxel.leaf_size", prev_params.voxel_grid_filter_params.leaf_size,
                m_plugin_params.voxel_grid_filter_params.leaf_size);
    note_change("ror.enabled", prev_params.ror_params.enabled, m_plugin_params.ror_params.enabled);
    note_change("ror.radius", prev_params.ror_params.radius, m_plugin_params.ror_params.radius);
    note_change("ror.min_neighbors", prev_params.ror_params.min_neighbors, m_plugin_params.ror_params.min_neighbors);
    note_change("livox.enabled", prev_params.livox_tag_filter_params.enabled,
                m_plugin_params.livox_tag_filter_params.enabled);
    note_change("livox.remove_high_conf_noise", prev_params.livox_tag_filter_params.remove_high_confidence_noise,
                m_plugin_params.livox_tag_filter_params.remove_high_confidence_noise);
    note_change("livox.remove_moderate_conf_noise",
                prev_params.livox_tag_filter_params.remove_moderate_confidence_noise,
                m_plugin_params.livox_tag_filter_params.remove_moderate_confidence_noise);
    note_change("livox.remove_low_conf_noise", prev_params.livox_tag_filter_params.remove_low_confidence_noise,
                m_plugin_params.livox_tag_filter_params.remove_low_confidence_noise);
    note_change("livox.remove_intensity_noise", prev_params.livox_tag_filter_params.remove_intensity_noise,
                m_plugin_params.livox_tag_filter_params.remove_intensity_noise);
    note_change("livox.output_per_return", prev_params.livox_tag_filter_params.output_per_return,
                m_plugin_params.livox_tag_filter_params.output_per_return);

    if (has_changes)
        NODELET_INFO_STREAM("LidarPreProcessingNodelet reconfigured: " << changes.str());
    else
        NODELET_INFO_STREAM("LidarPreProcessingNodelet reconfigured with no parameter changes");
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::update_enabled_plugins(const PreprocessingParamsConfig &config)
{
    m_enabled_plugins.clear();

    // Keep the execution order fixed so geometry-affecting steps happen before filtering.
    constexpr std::array<const char *, 5> kPluginOrder = {kDeskewKey, kGravityAlignKey, kVoxelKey,
                                                          kRorKey, kLivoxTagKey};
    for (const auto *key: kPluginOrder)
    {
        if (!is_preprocessor_enabled(key, config))
            continue;

        m_enabled_plugins.emplace_back(key);
    }

    NODELET_INFO_STREAM("Plugin Execution Order for enabled preprocessors: "
                        << (m_enabled_plugins.empty() ? "<none>"
                                                      : ros_utils::join_strings(m_enabled_plugins.begin(),
                                                                                m_enabled_plugins.end(), " -> ")));
}

template<typename PointT>
bool LidarPreProcessingNodelet<PointT>::is_preprocessor_enabled(const std::string &key,
                                                                const PreprocessingParamsConfig &config) const
{
    if (key == kDeskewKey)
        return config.pointcloud_deskew_enabled;
    if (key == kGravityAlignKey)
        return config.gravity_align_enabled;
    if (key == kVoxelKey)
        return config.voxel_grid_filter_enabled;
    if (key == kRorKey)
        return config.radius_outlier_removal_enabled;
    if (key == kLivoxTagKey)
    {
        if constexpr (!std::is_same_v<PointT, lidar_point_types::LivoxPoint>)
        {
            if (config.livox_tag_filter_enabled)
                NODELET_WARN_STREAM("Livox tag filter enabled for non-Livox sensor. Disabling.");
            return false;
        }
        return config.livox_tag_filter_enabled;
    }

    NODELET_WARN_STREAM("Unknown preprocessor key: " << key);
    return false;
}

template<typename PointT>
std::optional<std::size_t>
LidarPreProcessingNodelet<PointT>::find_cached_plugin_index(const std::string &plugin_name) const
{
    auto it = std::find_if(m_plugin_cache.begin(), m_plugin_cache.end(),
                           [&](const auto &entry) { return entry.plugin_name == plugin_name; });
    if (it == m_plugin_cache.end())
        return std::nullopt;
    return static_cast<std::size_t>(std::distance(m_plugin_cache.begin(), it));
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::load_plugins(const std::vector<std::string> &plugins_to_init)
{
    using plugin_interface = PreprocessorInterface;

    if (!m_plugins_loader)
    {
        m_plugins_loader = std::make_unique<pluginlib::ClassLoader<plugin_interface>>("lidar_preprocessing_pipeline",
                                                                                      preprocessor_base_class_name());
    }

    std::vector<PreprocessorPlugins> ordered_plugins;
    ordered_plugins.reserve(m_enabled_plugins.size());

    for (const auto &plugin_name: m_enabled_plugins)
    {
        // Check if plugin is cached (i.e. previously loaded)
        // Only load new plugins if it's being enabled for the first time
        auto cached_index = find_cached_plugin_index(plugin_name);
        bool needs_init =
                std::find(plugins_to_init.begin(), plugins_to_init.end(), plugin_name) != plugins_to_init.end();
        if (!cached_index)
        {
            try
            {
                auto plugin_instance = m_plugins_loader->createInstance(plugin_name);
                m_plugin_cache.push_back(PreprocessorPlugins{plugin_name, plugin_instance});
                cached_index = m_plugin_cache.size() - 1;
                needs_init = true;
            }
            catch (const pluginlib::PluginlibException &ex)
            {
                NODELET_ERROR_STREAM("Failed to load plugin '" << plugin_name << "': " << ex.what());
                continue;
            }
        }

        auto &cached_plugin = m_plugin_cache[*cached_index];
        if (needs_init)
        {
            cached_plugin.plugin_instance->initialize(m_plugin_params);
        }
        ordered_plugins.push_back(cached_plugin);
    }

    m_plugins = std::move(ordered_plugins);
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::init_publishers()
{
    update_output_publisher();
    update_livox_return_publishers();
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::init_subscribers()
{
    m_pointcloud_subscriber =
            m_node_handle.subscribe(m_input_topic, LIDAR_QUEUE_SIZE, &LidarPreProcessingNodelet::pointcloud_callback,
                                    this, ros::TransportHints().tcpNoDelay());

    if (!m_imu_topic.empty())
    {
        m_imu_subscriber =
                m_node_handle.subscribe(m_imu_topic, IMU_QUEUE_SIZE, &LidarPreProcessingNodelet::imu_callback, this,
                                        ros::TransportHints().tcpNoDelay());
    }

    const auto &calibration = m_plugin_params.imu_integrator_params.imu_calibration_params;
    if (!m_imu_topic.empty() && calibration.enabled)
    {
        m_odom_subscriber = m_node_handle.subscribe("odom", ODOM_QUEUE_SIZE, &LidarPreProcessingNodelet::odom_callback,
                                                    this, ros::TransportHints().tcpNoDelay());
    }
}

template<typename PointT>
bool LidarPreProcessingNodelet<PointT>::ensure_imu_ready()
{
    bool imu_ready = false;
    const bool imu_publishers = (m_imu_subscriber.getNumPublishers() > 0);
    {
        std::lock_guard imu_lock(m_imu_mutex);
        imu_ready = !m_imu_buffer.empty();
    }
    if (!imu_ready || !imu_publishers)
    {
        ROS_WARN_THROTTLE(2.0,
                          "IMU topic '%s' unavailable or no data yet; skipping "
                          "pointcloud processing.",
                          m_imu_topic.c_str());
        return false;
    }
    return true;
}

template<typename PointT>
bool LidarPreProcessingNodelet<PointT>::filter_points_by_imu_coverage(
        const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg, pcl::PointCloud<PointT> &cloud)
{
    double latest_imu_stamp = 0.0;
    {
        std::lock_guard imu_lock(m_imu_mutex);
        if (!m_imu_buffer.empty())
            latest_imu_stamp = m_imu_buffer.back().stamp;
    }

    if (latest_imu_stamp <= 0.0)
        return true;

    const double sweep_ref_time = pointcloud_msg->header.stamp.toSec();
    double min_raw_time = std::numeric_limits<double>::infinity();
    for (const auto &pt: cloud.points)
    {
        const double t = lidar_point_types::PointTimeAccessor<PointT>::time_seconds(pt, sweep_ref_time);
        min_raw_time = std::min(min_raw_time, t);
    }
    const double time_offset = std::isfinite(min_raw_time) ? (sweep_ref_time - min_raw_time) : 0.0;

    pcl::PointCloud<PointT> filtered_cloud;
    filtered_cloud.reserve(cloud.points.size());
    for (const auto &pt: cloud.points)
    {
        const double stamp =
                lidar_point_types::PointTimeAccessor<PointT>::time_seconds(pt, sweep_ref_time) + time_offset;
        if (stamp <= latest_imu_stamp)
            filtered_cloud.points.push_back(pt);
    }

    if (filtered_cloud.points.empty())
    {
        ROS_WARN_THROTTLE(2.0, "IMU data behind pointcloud; dropping scan (latest imu=%.6f).", latest_imu_stamp);
        return false;
    }

    if (filtered_cloud.points.size() != cloud.points.size())
    {
        const double dropped_ratio = 100.0 * (static_cast<double>(cloud.points.size() - filtered_cloud.points.size()) /
                                              static_cast<double>(cloud.points.size()));
        ROS_DEBUG_THROTTLE(2.0,
                           "Dropping %zu/%zu points (%.2f%%) beyond IMU coverage "
                           "(latest imu=%.6f).",
                           cloud.points.size() - filtered_cloud.points.size(), cloud.points.size(), dropped_ratio,
                           latest_imu_stamp);
    }

    filtered_cloud.header = cloud.header;
    filtered_cloud.is_dense = cloud.is_dense;
    filtered_cloud.width = static_cast<std::uint32_t>(filtered_cloud.points.size());
    filtered_cloud.height = 1;
    cloud = std::move(filtered_cloud);
    return true;
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::sort_points_by_time(double sweep_ref_time, pcl::PointCloud<PointT> &cloud) const
{
    // Deskewing expects points in ascending acquisition-time order.
    if (cloud.points.size() > 1)
    {
        std::sort(cloud.points.begin(), cloud.points.end(), [sweep_ref_time](const PointT &lhs, const PointT &rhs) {
            return lidar_point_types::PointTimeAccessor<PointT>::time_seconds(lhs, sweep_ref_time) <
                   lidar_point_types::PointTimeAccessor<PointT>::time_seconds(rhs, sweep_ref_time);
        });
    }

    cloud.width = static_cast<std::uint32_t>(cloud.points.size());
    cloud.height = 1;
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg)
{
    const bool imu_required =
            m_plugin_params.pointcloud_deskew_params.enabled || m_plugin_params.gravity_align_params.enabled;
    if (imu_required && !ensure_imu_ready())
        return;

    pcl::PointCloud<PointT> input_cloud;
    if (!parse_pointcloud(pointcloud_msg, input_cloud))
        return;

    if (imu_required)
    {
        if (!filter_points_by_imu_coverage(pointcloud_msg, input_cloud))
            return;
        sort_points_by_time(pointcloud_msg->header.stamp.toSec(), input_cloud);
    }

    std::lock_guard lock(m_callback_mutex);
    auto context = build_context(pointcloud_msg);

    pcl::PointCloud<PointT> output_cloud;
    run_pipeline(context, input_cloud, output_cloud);
    const auto gate_status = calibration_gate_status();
    if (gate_status)
        log_calibration_gate_status(*gate_status);
    if (gate_status && gate_status->calibration_enabled)
        return;

    publish_gravity_alignment_tf(pointcloud_msg);

    const bool publish_per_return = m_plugin_params.livox_tag_filter_params.enabled &&
                                    m_plugin_params.livox_tag_filter_params.output_per_return;
    if (publish_per_return && publish_livox_return_outputs(pointcloud_msg))
        return;

    publish_output(output_cloud, pointcloud_msg);
}

template<typename PointT>
bool LidarPreProcessingNodelet<PointT>::parse_pointcloud(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
                                                         pcl::PointCloud<PointT> &output_cloud) const
{
    try
    {
        pcl::fromROSMsg(*pointcloud_msg, output_cloud);
        return true;
    }
    catch (const std::exception &ex)
    {
        NODELET_ERROR_STREAM("Failed to parse input pointcloud: " << ex.what());
        return false;
    }
}

template<typename PointT>
std::string LidarPreProcessingNodelet<PointT>::build_output_topic(const std::string &input_topic,
                                                                  const std::string &suffix)
{
    if (input_topic.empty())
        return suffix;

    std::string trimmed = input_topic;
    while (!trimmed.empty() && trimmed.back() == '/')
        trimmed.pop_back();

    if (trimmed.empty())
        return suffix;

    const std::size_t sep = trimmed.rfind('/');
    if (sep == std::string::npos)
        return suffix;

    return trimmed.substr(0, sep + 1) + suffix;
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::update_output_publisher()
{
    const std::string output_topic = build_output_topic(m_input_topic, kDefaultOutputTopic);
    m_pointcloud_publisher = m_node_handle.advertise<sensor_msgs::PointCloud2>(output_topic, LIDAR_QUEUE_SIZE);
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::update_livox_return_publishers()
{
    if constexpr (!std::is_same_v<PointT, lidar_point_types::LivoxPoint>)
    {
        return;
    }

    const bool enable_return_outputs = m_plugin_params.livox_tag_filter_params.enabled &&
                                       m_plugin_params.livox_tag_filter_params.output_per_return;
    if (enable_return_outputs)
    {
        const std::string return1_topic = build_output_topic(m_input_topic, kReturn1Topic);
        const std::string return2_topic = build_output_topic(m_input_topic, kReturn2Topic);
        m_return1_publisher = m_node_handle.advertise<sensor_msgs::PointCloud2>(return1_topic, LIDAR_QUEUE_SIZE);
        m_return2_publisher = m_node_handle.advertise<sensor_msgs::PointCloud2>(return2_topic, LIDAR_QUEUE_SIZE);
    }
    else
    {
        m_return1_publisher.shutdown();
        m_return2_publisher.shutdown();
    }
}

template<typename PointT>
lidar_preprocessing_plugins::plugins_context_data::PreprocessContext
LidarPreProcessingNodelet<PointT>::build_context(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg)
{
    lidar_preprocessing_plugins::plugins_context_data::PreprocessContext context;
    context.sweep_ref_time = pointcloud_msg->header.stamp.toSec();

    if (m_plugin_params.pointcloud_deskew_params.enabled || m_plugin_params.gravity_align_params.enabled)
        populate_imu_context(context);

    if (m_plugin_params.imu_integrator_params.imu_calibration_params.enabled)
        populate_external_velocity_context(context);

    return context;
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::populate_imu_context(
        lidar_preprocessing_plugins::plugins_context_data::PreprocessContext &context)
{
    {
        std::lock_guard imu_lock(m_imu_mutex);
        context.imu_samples.assign(m_imu_buffer.begin(), m_imu_buffer.end());
    }

    if (context.imu_samples.empty())
        return;

    std::sort(context.imu_samples.begin(), context.imu_samples.end(),
              [](const auto &lhs, const auto &rhs) { return lhs.stamp < rhs.stamp; });

    lidar_preprocessing_plugins::plugins_context_data::ImuSampleBuffer filtered;
    filtered.reserve(context.imu_samples.size());

    for (const auto &sample: context.imu_samples)
    {
        if (filtered.empty())
        {
            filtered.push_back(sample);
            filtered.back().dt = 0.0;
            continue;
        }

        double dt = sample.stamp - filtered.back().stamp;
        if (dt <= 0.0)
            continue;

        auto adjusted = sample;
        adjusted.dt = dt;
        filtered.push_back(adjusted);
    }

    context.imu_samples = std::move(filtered);
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::populate_external_velocity_context(
        lidar_preprocessing_plugins::plugins_context_data::PreprocessContext &context)
{
    std::lock_guard lock(m_odom_mutex);
    context.external_velocity_valid = m_has_odom;
    context.external_velocity_stamp = m_latest_odom_stamp;
    context.external_speed_mps = m_latest_odom_speed;
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::run_pipeline(
        const lidar_preprocessing_plugins::plugins_context_data::PreprocessContext &context,
        const pcl::PointCloud<PointT> &input, pcl::PointCloud<PointT> &output) const
{
    // Working is the current pointcloud being processed
    // Scratch is a temporary pointcloud for plugin output that will be swapped
    // with working
    //    and consumed by the next plugin
    pcl::PointCloud<PointT> working = input;
    pcl::PointCloud<PointT> scratch;

    for (const auto &plugin: m_plugins)
    {
        if (!plugin.plugin_instance)
        {
            NODELET_WARN_STREAM("Enabled plugin " << plugin.plugin_name << " not loaded. Skipping.");
            continue;
        }

        plugin.plugin_instance->process(context, working, scratch);
        if (scratch.empty())
            scratch = working;
        working.swap(scratch);
        scratch.clear();
    }

    output = std::move(working);
}

template<typename PointT>
std::optional<lidar_preprocessing_utils::imu_integrator::CalibrationGateStatus>
LidarPreProcessingNodelet<PointT>::calibration_gate_status() const
{
    const auto &calibration = m_plugin_params.imu_integrator_params.imu_calibration_params;
    if (!calibration.enabled)
        return std::nullopt;

    using GateStatus = lidar_preprocessing_utils::imu_integrator::CalibrationGateStatus;

    auto find_entry = [&](const char *plugin_key) -> const PreprocessorPlugins * {
        auto it = std::find_if(m_plugins.begin(), m_plugins.end(),
                               [&](const auto &entry) { return entry.plugin_name == plugin_key; });
        if (it == m_plugins.end())
            return nullptr;
        return &(*it);
    };

    auto fetch_deskew_status = [&]() -> std::optional<GateStatus> {
        const auto *entry = find_entry(kDeskewKey);
        if (!entry || !entry->plugin_instance)
            return std::nullopt;
        auto *deskew_plugin = dynamic_cast<lidar_preprocessing_plugins::PointCloudDeskewPlugin<PointT> *>(
                entry->plugin_instance.get());
        if (!deskew_plugin)
            return std::nullopt;
        return deskew_plugin->calibration_gate_status();
    };

    auto fetch_gravity_status = [&]() -> std::optional<GateStatus> {
        const auto *entry = find_entry(kGravityAlignKey);
        if (!entry || !entry->plugin_instance)
            return std::nullopt;
        auto *gravity_plugin =
                dynamic_cast<lidar_preprocessing_plugins::GravityAlignPlugin<PointT> *>(entry->plugin_instance.get());
        if (!gravity_plugin)
            return std::nullopt;
        return gravity_plugin->calibration_gate_status();
    };

    if (m_plugin_params.pointcloud_deskew_params.enabled)
    {
        auto status = fetch_deskew_status();
        if (status)
            return status;
    }

    if (m_plugin_params.gravity_align_params.enabled)
        return fetch_gravity_status();

    return std::nullopt;
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::log_calibration_gate_status(
        const lidar_preprocessing_utils::imu_integrator::CalibrationGateStatus &status) const
{
    if (!status.calibration_enabled || !status.blocked)
        return;

    if (!status.external_velocity_valid)
    {
        ROS_WARN_THROTTLE(2.0, "IMU calibration waiting for external velocity on odom.");
        return;
    }

    if (status.external_velocity_stale)
    {
        ROS_WARN_THROTTLE(2.0,
                          "IMU calibration waiting: external velocity stale (age "
                          "%.3f s > %.3f s).",
                          status.velocity_age_sec, status.velocity_max_age_sec);
        return;
    }

    if (status.external_velocity_too_fast)
    {
        ROS_WARN_THROTTLE(2.0, "IMU calibration waiting: external velocity %.3f m/s exceeds %.3f m/s.",
                          status.speed_mps, status.speed_limit_mps);
    }
}

template<typename PointT>
bool LidarPreProcessingNodelet<PointT>::get_gravity_alignment(Eigen::Quaternionf &alignment) const
{
    if (!m_plugin_params.gravity_align_params.enabled)
        return false;

    auto it = std::find_if(m_plugins.begin(), m_plugins.end(),
                           [&](const auto &entry) { return entry.plugin_name == kGravityAlignKey; });
    if (it == m_plugins.end())
        return false;

    auto *gravity_plugin =
            dynamic_cast<lidar_preprocessing_plugins::GravityAlignPlugin<PointT> *>(it->plugin_instance.get());
    if (!gravity_plugin)
        return false;

    return gravity_plugin->last_alignment(alignment);
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::publish_output(const pcl::PointCloud<PointT> &output_cloud,
                                                       const sensor_msgs::PointCloud2::ConstPtr &input_msg) const
{
    sensor_msgs::PointCloud2 output_msg;
    pcl::toROSMsg(output_cloud, output_msg);
    output_msg.header = input_msg->header;
    // Gravity alignment changes the coordinate frame, so publish in a derived frame_id.
    //   {
    //       // Intentionally keep the original frame_id even when alignment is available.
    //   }
    //
    // [Fix] Set frame_id to gravity_aligned_frame when alignment is active
    Eigen::Quaternionf align_q;
    if (get_gravity_alignment(align_q))
    {
        output_msg.header.frame_id = gravity_aligned_frame(input_msg->header.frame_id);
    }
    m_pointcloud_publisher.publish(output_msg);
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::publish_gravity_alignment_tf(
        const sensor_msgs::PointCloud2::ConstPtr &input_msg)
{
    if (!m_tf_broadcaster)
        return;

    Eigen::Quaternionf align_q;
    if (!get_gravity_alignment(align_q))
        return;

    const std::string &lidar_frame = input_msg->header.frame_id;
    if (lidar_frame.empty())
        return;

    const std::string aligned_frame = gravity_aligned_frame(lidar_frame);
    if (aligned_frame.empty() || aligned_frame == lidar_frame)
        return;

    if (!m_cached_lidar_to_base_valid || m_cached_lidar_frame != lidar_frame)
    {
        try
        {
            m_cached_lidar_to_base_tf =
                    m_tf_buffer.lookupTransform(m_gravity_parent_frame, lidar_frame, ros::Time(0), ros::Duration(0.1));
            m_cached_lidar_to_base_valid = true;
            m_cached_lidar_frame = lidar_frame;
        }
        catch (const tf2::TransformException &ex)
        {
            NODELET_WARN_STREAM_THROTTLE(2.0, "Failed to lookup TF from '" << lidar_frame << "' to '"
                                                                           << m_gravity_parent_frame
                                                                           << "': " << ex.what());
            m_cached_lidar_to_base_valid = false;
            return;
        }
    }

    tf2::Transform tf_base_lidar;
    tf2::fromMsg(m_cached_lidar_to_base_tf.transform, tf_base_lidar);

    const Eigen::Quaternionf q = align_q;
    tf2::Quaternion q_lidar_gravity(q.x(), q.y(), q.z(), q.w());
    tf2::Transform tf_lidar_gravity(q_lidar_gravity, tf2::Vector3(0.0, 0.0, 0.0));

    const tf2::Transform tf_base_gravity = tf_base_lidar * tf_lidar_gravity;

    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header.stamp = input_msg->header.stamp;
    tf_msg.header.frame_id = m_gravity_parent_frame;
    tf_msg.child_frame_id = aligned_frame;
    tf_msg.transform = tf2::toMsg(tf_base_gravity);

    m_tf_broadcaster->sendTransform(tf_msg);
}

template<typename PointT>
std::string LidarPreProcessingNodelet<PointT>::gravity_aligned_frame(const std::string &input_frame) const
{
    if (input_frame.empty())
        return std::string("gravity_frame");
    return input_frame + "_gravity_frame";
}

template<typename PointT>
bool LidarPreProcessingNodelet<PointT>::publish_livox_return_outputs(
        const sensor_msgs::PointCloud2::ConstPtr &input_msg) const
{
    if constexpr (!std::is_same_v<PointT, lidar_point_types::LivoxPoint>)
    {
        return false;
    }

    auto it = std::find_if(m_plugins.begin(), m_plugins.end(),
                           [&](const auto &entry) { return entry.plugin_name == kLivoxTagKey; });
    if (it == m_plugins.end())
    {
        NODELET_WARN_STREAM("Livox tag filter plugin not loaded; skipping per-return outputs.");
        return false;
    }

    auto *livox_plugin =
            dynamic_cast<lidar_preprocessing_plugins::LivoxTagFilterPlugin<PointT> *>(it->plugin_instance.get());
    if (!livox_plugin)
    {
        NODELET_WARN_STREAM("Livox tag filter plugin type mismatch; skipping per-return outputs.");
        return false;
    }

    Eigen::Quaternionf align_q;
    const bool use_gravity_frame = get_gravity_alignment(align_q);
    const std::string output_frame =
            use_gravity_frame ? gravity_aligned_frame(input_msg->header.frame_id) : input_msg->header.frame_id;

    sensor_msgs::PointCloud2 output_msg;

    pcl::toROSMsg(livox_plugin->return0_cloud(), output_msg);
    output_msg.header = input_msg->header;
    if (use_gravity_frame)
        output_msg.header.frame_id = output_frame;
    m_pointcloud_publisher.publish(output_msg);

    pcl::toROSMsg(livox_plugin->return1_cloud(), output_msg);
    output_msg.header = input_msg->header;
    if (use_gravity_frame)
        output_msg.header.frame_id = output_frame;
    m_return1_publisher.publish(output_msg);

    pcl::toROSMsg(livox_plugin->return2_cloud(), output_msg);
    output_msg.header = input_msg->header;
    if (use_gravity_frame)
        output_msg.header.frame_id = output_frame;
    m_return2_publisher.publish(output_msg);

    return true;
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg)
{
    lidar_preprocessing_plugins::plugins_context_data::ImuSample sample;
    sample.stamp = imu_msg->header.stamp.toSec();
    sample.ang_vel = Eigen::Vector3f(static_cast<float>(imu_msg->angular_velocity.x),
                                     static_cast<float>(imu_msg->angular_velocity.y),
                                     static_cast<float>(imu_msg->angular_velocity.z));
    sample.lin_accel = Eigen::Vector3f(static_cast<float>(imu_msg->linear_acceleration.x),
                                       static_cast<float>(imu_msg->linear_acceleration.y),
                                       static_cast<float>(imu_msg->linear_acceleration.z));

    std::lock_guard lock(m_imu_mutex);
    if (!m_imu_buffer.empty() && sample.stamp < m_imu_buffer.front().stamp)
    {
        NODELET_WARN_STREAM("IMU time reset detected, clearing buffer.");
        m_imu_buffer.clear();
        m_latest_imu_stamp = 0.0;
    }

    m_imu_buffer.push_back(sample);
    m_latest_imu_stamp = std::max(m_latest_imu_stamp, sample.stamp);

    while (!m_imu_buffer.empty() && (m_latest_imu_stamp - m_imu_buffer.front().stamp) > m_imu_buffer_duration)
    {
        m_imu_buffer.pop_front();
    }
}

template<typename PointT>
void LidarPreProcessingNodelet<PointT>::odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg)
{
    const auto &twist = odom_msg->twist.twist;
    const double speed = std::sqrt((twist.linear.x * twist.linear.x) + (twist.linear.y * twist.linear.y) +
                                   (twist.linear.z * twist.linear.z));

    std::lock_guard lock(m_odom_mutex);
    m_latest_odom_speed = speed;
    m_latest_odom_stamp = odom_msg->header.stamp.toSec();
    m_has_odom = true;
}

}  // namespace lidar_preprocessing_pipeline

namespace lidar_preprocessing_nodelet
{
using OusterLidarPreProcessingNodelet =
        lidar_preprocessing_pipeline::LidarPreProcessingNodelet<lidar_point_types::OusterPoint>;
using VelodyneLidarPreProcessingNodelet =
        lidar_preprocessing_pipeline::LidarPreProcessingNodelet<lidar_point_types::VelodynePoint>;
using LivoxLidarPreProcessingNodelet =
        lidar_preprocessing_pipeline::LidarPreProcessingNodelet<lidar_point_types::LivoxPoint>;
}  // namespace lidar_preprocessing_nodelet

PLUGINLIB_EXPORT_CLASS(lidar_preprocessing_nodelet::OusterLidarPreProcessingNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(lidar_preprocessing_nodelet::VelodyneLidarPreProcessingNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(lidar_preprocessing_nodelet::LivoxLidarPreProcessingNodelet, nodelet::Nodelet)
