#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <dynamic_reconfigure/server.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <boost/thread/recursive_mutex.hpp>

#include "lidar_preprocessing_pipeline/PreprocessingParamsConfig.h"

#include "lidar_preprocessing_pipeline/dtypes/plugin_params.hpp"
#include "lidar_preprocessing_pipeline/plugins/plugin_interface.hpp"

namespace lidar_preprocessing_pipeline
{
/**
 * @class LidarPreProcessingNodelet
 * @brief Nodelet that runs a configurable pre-processing pipeline on a LiDAR pointcloud.
 *
 * @tparam PointT PCL point type used by the selected LiDAR.
 */
template<typename PointT>
class LidarPreProcessingNodelet : public nodelet::Nodelet
{
public:
    LidarPreProcessingNodelet() = default;
    ~LidarPreProcessingNodelet() override = default;

    // Nodelet method
    void onInit() override;

private:
    using PreprocessorInterface = lidar_preprocessing_plugins::ILidarPreProcessingPlugin<PointT>;
    struct PreprocessorPlugins
    {
        std::string plugin_name;
        boost::shared_ptr<PreprocessorInterface> plugin_instance;
    };

    struct ParamChangeFlags
    {
        bool deskew_enabled_changed{false};
        bool gravity_enabled_changed{false};
        bool voxel_enabled_changed{false};
        bool ror_enabled_changed{false};
        bool livox_enabled_changed{false};
        bool enable_flags_changed{false};
        bool approx_gravity_changed{false};
        bool voxel_params_changed{false};
        bool ror_params_changed{false};
        bool livox_params_changed{false};
    };


    /**
     * @brief Load parameters from the rosparam server.
     */
    void load_params();

    /**
     * @brief Load core nodelet parameters for eg. input topics.
     *
     * @param nh Node handle to use for parameter loading.
     */
    void load_core_params(const ros::NodeHandle &nh);

    /**
     * @brief Load pointcloud deskewing parameters.
     *
     * @param nh Node handle to use for parameter loading.
     */
    void load_deskew_params(const ros::NodeHandle &nh);

    /**
     * @brief Load IMU parameters shared by IMU-based plugins.
     *
     * @param nh Node handle to use for parameter loading.
     */
    void load_shared_imu_params(const ros::NodeHandle &nh);

    /**
     * @brief Load gravity alignment parameters.
     *
     * @param nh Node handle to use for parameter loading.
     */
    void load_gravity_align_params(const ros::NodeHandle &nh);

    /**
     * @brief Load IMU-related parameters (shared by IMU-based plugins).
     *
     * @param nh Node handle to use for parameter loading.
     */
    void load_imu_params(const ros::NodeHandle &nh);

    /**
     * @brief Load IMU integration parameters (shared by IMU-based plugins).
     *
     * @param nh Node handle to use for parameter loading.
     */
    void load_imu_integration_params(const ros::NodeHandle &nh);

    /**
     * @brief Load IMU calibration parameters (shared by IMU-based plugins).
     *
     * @param nh Node handle to use for parameter loading.
     */
    void load_imu_calibration_params(const ros::NodeHandle &nh);

    /**
     * @brief Load extrinsics parameters (shared by IMU-based plugins).
     *
     * @param nh Node handle to use for parameter loading.
     */
    void load_extrinsics_params(const ros::NodeHandle &nh);

    /**
     * @brief Load voxel grid filter parameters.
     *
     * @param nh Node handle to use for parameter loading.
     */
    void load_voxel_grid_params(const ros::NodeHandle &nh);

    /**
     * @brief Load radius outlier removal filter parameters.
     *
     * @param nh Node handle to use for parameter loading.
     */
    void load_ror_params(const ros::NodeHandle &nh);

    /**
     * @brief Load Livox tag filter parameters.
     *
     * @param nh Node handle to use for parameter loading.
     */
    void load_livox_tag_params(const ros::NodeHandle &nh);

    /**
     * @brief Initialize publishers.
     */
    void init_publishers();

    /**
     * @brief Initialize subscribers.
     */
    void init_subscribers();

    /**
     * @brief Initialize dynamic reconfigure server.
     */
    void initialize_dynamic_reconfigure();

    /**
     * @brief Dynamic reconfigure callback to update plugin parameters.
     *
     * @param config Updated parameters.
     * @param level Unused.
     */
    void reconfigure_callback(lidar_preprocessing_pipeline::PreprocessingParamsConfig &config, uint32_t level);

    /**
     * @brief Get the base class name for the preprocessor plugins.
     *
     * @return const char* Base class name.
     */
    static const char *preprocessor_base_class_name();

    /**
     * @brief Load enabled plugins and initialize the specified set.
     *
     * @param plugins_to_init Plugin keys to initialize with latest params.
     */

    /**
     * @brief Load enabled plugins and initialize the specified set.
     *
     * @param plugins_to_init Plugin keys to initialize with latest params.
     */
    void load_plugins(const std::vector<std::string> &plugins_to_init);

    /**
     * @brief Apply the dynamic reconfigure config to the plugin parameters.
     *
     * @param config Dynamic reconfigure config.
     */
    void apply_reconfigure_config(const PreprocessingParamsConfig &config);

    /**
     * @brief Compute which parameters have changed compared to previous params.
     *
     * @param prev_params Previous plugin parameters.
     * @return ParamChangeFlags Flags indicating which parameters have changed.
     */
    ParamChangeFlags
    compute_param_change_flags(const lidar_preprocessing_plugins::PreprocessingPluginParams &prev_params) const;

    /**
     * @brief Build a list of plugins that need to be initialized based on changed parameters.
     *
     * @param flags Parameter change flags.
     * @return std::vector<std::string> List of plugin keys to initialize.
     */
    std::vector<std::string> build_plugins_to_init(const ParamChangeFlags &flags) const;

    /**
     * @brief Apply plugin updates based on changed parameters.
     *
     * @param config Dynamic reconfigure config.
     * @param flags Parameter change flags.
     * @param plugins_to_init List of plugin keys to initialize.
     */
    void apply_plugin_updates(const PreprocessingParamsConfig &config, const ParamChangeFlags &flags,
                              const std::vector<std::string> &plugins_to_init);

    /**
     * @brief Log parameter changes after dynamic reconfigure.
     *
     * @param prev_params Previous plugin parameters.
     */
    void log_reconfigure_changes(const lidar_preprocessing_plugins::PreprocessingPluginParams &prev_params) const;

    /**
     * @brief Find the index of a cached plugin by name if it exists.
     *
     * @param plugin_name Name of the plugin to find.
     * @return std::optional<std::size_t> Index of the cached plugin, or std::nullopt if not found.
     */
    std::optional<std::size_t> find_cached_plugin_index(const std::string &plugin_name) const;

    /**
     * @brief Check whether a preprocessor is enabled for a given key.
     *
     * @param key Preprocessor key.
     * @param config Current configuration.
     * @return true if enabled, false otherwise.
     */
    bool is_preprocessor_enabled(const std::string &key,
                                 const lidar_preprocessing_pipeline::PreprocessingParamsConfig &config) const;

    /**
     * @brief Update the ordered list of enabled plugins based on config.
     *
     * @param config Current configuration.
     */
    void update_enabled_plugins(const lidar_preprocessing_pipeline::PreprocessingParamsConfig &config);

    /**
     * @brief Pointcloud callback to run the preprocessing pipeline.
     *
     * @param pointcloud_msg Input pointcloud message.
     */
    void pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg);

    /**
     * @brief Verify IMU data availability before processing.
     *
     * @return true if IMU data is available, false otherwise.
     */
    bool ensure_imu_ready();

    /**
     * @brief Drop points that are beyond the latest IMU timestamp.
     *
     * @param pointcloud_msg Input pointcloud message (for sweep reference time).
     * @param cloud Pointcloud to filter in-place.
     * @return true if processing should continue, false if the scan should be dropped.
     */
    bool filter_points_by_imu_coverage(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
                                       pcl::PointCloud<PointT> &cloud);

    /**
     * @brief Sort points by per-point timestamp and mark cloud as unorganized.
     *
     * @param sweep_ref_time Sweep reference time in seconds.
     * @param cloud Pointcloud to sort in-place.
     */
    void sort_points_by_time(double sweep_ref_time, pcl::PointCloud<PointT> &cloud) const;

    /**
     * @brief IMU callback to buffer measurements for deskewing.
     *
     * @param imu_msg Input IMU message.
     */
    void imu_callback(const sensor_msgs::Imu::ConstPtr &imu_msg);

    /**
     * @brief Odom callback to cache external velocity for calibration gating.
     *
     * @param odom_msg Input odometry message.
     */
    void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg);

    /**
     * @brief Parse the input pointcloud message into a PCL point cloud.
     *
     * @param pointcloud_msg Input pointcloud message.
     * @param output_cloud Output PCL point cloud.
     * @return true if parsing was successful, false otherwise.
     */
    bool parse_pointcloud(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg,
                          pcl::PointCloud<PointT> &output_cloud) const;

    /**
     * @brief Build the preprocessing context for the current pointcloud.
     *
     * @param pointcloud_msg Input pointcloud message.
     * @return lidar_preprocessing_plugins::plugins_context_data::PreprocessContext Preprocessing context.
     */
    lidar_preprocessing_plugins::plugins_context_data::PreprocessContext
    build_context(const sensor_msgs::PointCloud2::ConstPtr &pointcloud_msg);

    /**
     * @brief Populate IMU data into the preprocessing context.
     *  A time filter to populate monotonically increasing IMU data with dt > 0 is applied.
     *
     * @param context Preprocessing context to populate.
     */
    void populate_imu_context(lidar_preprocessing_plugins::plugins_context_data::PreprocessContext &context);

    /**
     * @brief Populate external velocity data into the preprocessing context.
     *
     * @param context Preprocessing context to populate.
     */
    void
    populate_external_velocity_context(lidar_preprocessing_plugins::plugins_context_data::PreprocessContext &context);

    /**
     * @brief Execute the preprocessing pipeline on the input pointcloud.
     *
     * @param context Context data for preprocessing.
     * @param input Input pointcloud.
     * @param output Output preprocessed pointcloud.
     */
    void run_pipeline(const lidar_preprocessing_plugins::plugins_context_data::PreprocessContext &context,
                      const pcl::PointCloud<PointT> &input, pcl::PointCloud<PointT> &output) const;

    /**
     * @brief Fetch the latest IMU calibration gate status from IMU-based plugins.
     */
    std::optional<lidar_preprocessing_utils::imu_integrator::CalibrationGateStatus> calibration_gate_status() const;

    /**
     * @brief Log external velocity gate status for IMU calibration.
     */
    void
    log_calibration_gate_status(const lidar_preprocessing_utils::imu_integrator::CalibrationGateStatus &status) const;

    /**
     * @brief Publish the output pointcloud.
     *
     * @param output_cloud Output preprocessed pointcloud.
     * @param input_msg Input pointcloud message (for header info).
     */
    void publish_output(const pcl::PointCloud<PointT> &output_cloud,
                        const sensor_msgs::PointCloud2::ConstPtr &input_msg) const;

    /**
     * @brief Fetch the latest gravity alignment rotation if available.
     *
     * @param alignment Output alignment quaternion.
     * @return true if a valid alignment is available, false otherwise.
     */
    bool get_gravity_alignment(Eigen::Quaternionf &alignment) const;

    /**
     * @brief Publish gravity alignment TF (LiDAR -> gravity-aligned frame).
     *
     * @param input_msg Input pointcloud message (for header info).
     */
    void publish_gravity_alignment_tf(const sensor_msgs::PointCloud2::ConstPtr &input_msg);

    /**
     * @brief Resolve the gravity-aligned frame name for the current input frame.
     */
    std::string gravity_aligned_frame(const std::string &input_frame) const;

    /**
     * @brief Publish Livox return-specific pointclouds if available.
     *
     * @param input_msg Input pointcloud message (for header info).
     * @return true if return-specific clouds were published.
     */
    bool publish_livox_return_outputs(const sensor_msgs::PointCloud2::ConstPtr &input_msg) const;

    /**
     * @brief Build an output topic by replacing the input suffix with the given suffix.
     */
    static std::string build_output_topic(const std::string &input_topic, const std::string &suffix);

    /**
     * @brief Advertise the main output publisher based on current params.
     */
    void update_output_publisher();

    /**
     * @brief Advertise/shutdown Livox return publishers based on current params.
     */
    void update_livox_return_publishers();

    // Node handles
    ros::NodeHandle m_node_handle;
    ros::NodeHandle m_private_node_handle;

    // Subscribers
    ros::Subscriber m_pointcloud_subscriber;
    ros::Subscriber m_imu_subscriber;

    // Publishers
    ros::Publisher m_pointcloud_publisher;

    // Only used for Livox tag filter return-specific outputs
    ros::Publisher m_return1_publisher;
    ros::Publisher m_return2_publisher;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    tf2_ros::Buffer m_tf_buffer;
    std::unique_ptr<tf2_ros::TransformListener> m_tf_listener;
    bool m_cached_lidar_to_base_valid{false};
    std::string m_cached_lidar_frame;
    geometry_msgs::TransformStamped m_cached_lidar_to_base_tf;

    // Topics
    std::string m_input_topic;
    std::string m_imu_topic;
    std::string m_gravity_parent_frame;

    static constexpr const char *kDefaultOutputTopic = "points_processed";
    static constexpr const char *kReturn1Topic = "points2_processed";
    static constexpr const char *kReturn2Topic = "points3_processed";
    static constexpr const char *kGravityParentFrameBase = "base_footprint";

    static constexpr int LIDAR_QUEUE_SIZE = 1;
    static constexpr int IMU_QUEUE_SIZE = 1000;
    static constexpr int ODOM_QUEUE_SIZE = 10;
    double m_imu_buffer_duration{2.0};
    std::deque<lidar_preprocessing_plugins::plugins_context_data::ImuSample,
               Eigen::aligned_allocator<lidar_preprocessing_plugins::plugins_context_data::ImuSample>>
            m_imu_buffer;
    std::mutex m_imu_mutex;
    double m_latest_imu_stamp{0.0};
    ros::Subscriber m_odom_subscriber;
    std::mutex m_odom_mutex;
    bool m_has_odom{false};
    double m_latest_odom_stamp{0.0};
    double m_latest_odom_speed{0.0};

    // Plugin keys
    static constexpr const char *kDeskewKey = "deskew";
    static constexpr const char *kGravityAlignKey = "gravity_align";
    static constexpr const char *kVoxelKey = "voxel_grid";
    static constexpr const char *kRorKey = "ror";
    static constexpr const char *kLivoxTagKey = "livox_tag_filter";

    // Plugin loader and instances
    std::unique_ptr<pluginlib::ClassLoader<PreprocessorInterface>> m_plugins_loader;
    std::vector<PreprocessorPlugins> m_plugin_cache;
    std::vector<PreprocessorPlugins> m_plugins;

    // Pipeline configuration
    std::vector<std::string> m_enabled_plugins;
    bool m_plugins_initialized{false};

    // Latest dynamic reconfigure parameters
    lidar_preprocessing_plugins::PreprocessingPluginParams m_plugin_params;

    // Dynamic reconfigure server
    std::shared_ptr<dynamic_reconfigure::Server<lidar_preprocessing_pipeline::PreprocessingParamsConfig>> m_reconfig_server;
    boost::recursive_mutex m_reconfig_mutex;

    // Protects the processing pipeline and config updates.
    std::mutex m_callback_mutex;
};

}  // namespace lidar_preprocessing_pipeline
