#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <cstdint>
#include <vector>

#include "lidar_preprocessing_pipeline/dtypes/preprocess_context.hpp"

namespace lidar_preprocessing_utils
{
namespace imu_integrator
{
/**
 * @brief External velocity gate for IMU calibration.
 */
struct ExternalVelocityCheckParams
{
    // max_speed_mps doc fix: header default didn't match YAML config (0.05).
    // Rosbag data shows sensor vibration ~0.014 m/s at rest — 0.01 too strict.
    //
    // [Original] float max_speed_mps{0.01f};
    // [Fix] Aligned to YAML value (0.05f)
    float max_speed_mps{0.05f};
    double max_age_sec{0.5};
};

/**
 * @brief State of the external velocity gate for IMU calibration.
 */
struct CalibrationGateStatus
{
    bool calibration_enabled{false};
    bool blocked{false};
    bool external_velocity_valid{false};
    bool external_velocity_stale{false};
    bool external_velocity_too_fast{false};
    double speed_mps{0.0};
    double speed_limit_mps{0.0};
    double velocity_age_sec{0.0};
    double velocity_max_age_sec{0.0};
};

/**
 * @brief Configuration for initial IMU bias calibration.
 */
struct ImuCalibrationParams
{
    bool enabled{false};
    uint8_t calibration_time_sec{3};
    ExternalVelocityCheckParams external_velocity;
    Eigen::Vector3f gyro_bias{Eigen::Vector3f::Zero()};
    Eigen::Vector3f accel_bias{Eigen::Vector3f::Zero()};
    Eigen::Matrix3f imu_accel_smatrix{Eigen::Matrix3f::Identity()};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief Parameters controlling IMU integration behavior.
 */
struct ImuIntegratorParams
{
    bool approximate_gravity{true};
    float gravity_mps2{9.80665f};
    ImuCalibrationParams imu_calibration_params;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};


/**
 * @brief Simple IMU bias container (body frame).
 */
struct ImuBias
{
    Eigen::Vector3f accel_bias{Eigen::Vector3f::Zero()};
    Eigen::Vector3f gyro_bias{Eigen::Vector3f::Zero()};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief Minimal IMU state for deskew integration.
 */
struct State
{
    Eigen::Vector3f p{Eigen::Vector3f::Zero()};
    Eigen::Quaternionf q{Eigen::Quaternionf::Identity()};
    Eigen::Vector3f v{Eigen::Vector3f::Zero()};
    ImuBias imu_bias;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief Output of IMU integration over the requested timestamps.
 */
struct IntegrationResult
{
    bool success{false};
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> poses;
    State reference_state;
    bool reference_state_valid{false};
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}  // namespace imu_integrator

class ImuIntegrator
{
public:
    /**
     * @brief Construct an IMU integrator with calibration + gravity parameters.
     *  The core logic for IMU integration is adapted from DLIO
     * https://github.com/vectr-ucla/direct_lidar_inertial_odometry
     * @param imu_integrator_params IMU integrator parameters.
     */
    explicit ImuIntegrator(const imu_integrator::ImuIntegratorParams &imu_integrator_params);

    /**
     * @brief Integrate IMU measurements to poses at the requested timestamps.
     *
     * @param integration_start_time Start time for integration.
     * @param sorted_timestamps Monotonic timestamps for which poses are requested.
     * @param reference_index Index into sorted_timestamps to return a reference state.
     * @param imu_samples IMU samples (monotonic time, dt > 0).
     * @return IntegrationResult containing poses and optional reference state.
     */
    imu_integrator::IntegrationResult
    integrate(double integration_start_time, const std::vector<double> &sorted_timestamps, std::size_t reference_index,
              const lidar_preprocessing_plugins::plugins_context_data::ImuSampleBuffer &imu_samples);

    /**
     * @brief Set the internal IMU state.
     *
     * @param state IMU state to set.
     */
    void set_state(const imu_integrator::State &state);

    /**
     * @brief Provide external velocity for calibration gating.
     *
     * @param speed_mps External speed in m/s.
     * @param stamp Timestamp of the velocity sample (s).
     * @param valid Whether the external velocity is valid.
     */
    void set_external_velocity(double speed_mps, double stamp, bool valid);

    /**
     * @brief Get the latest calibration gate status.
     */
    imu_integrator::CalibrationGateStatus calibration_gate_status() const;

private:
    /**
     * @brief Integrate quaternion forward with angular velocity.
     * @param q Initial orientation.
     * @param omega Angular velocity (rad/s).
     * @param dt Time step (s).
     * @return Integrated orientation.
     */
    static Eigen::Quaternionf integrate_quaternion(const Eigen::Quaternionf &q, const Eigen::Vector3f &omega, float dt);

    /**
     * @brief Compute IMU biases and optional gravity alignment from initial samples.
     * @param imu_samples IMU samples for calibration.
     * @return true once calibration completes.
     */
    bool execute_imu_calibration(const lidar_preprocessing_plugins::plugins_context_data::ImuSampleBuffer &imu_samples);

    /**
     * @brief Validate integration inputs and compute the requested end time.
     * @param integration_start_time Start time for integration.
     * @param sorted_timestamps Monotonic timestamps for which poses are requested.
     * @param reference_index Index into sorted_timestamps to return a reference state.
     * @param imu_samples IMU samples (monotonic time, dt > 0).
     * @param end_time Output end time for integration.
     * @return true when inputs cover the requested integration window.
     */
    static bool are_integration_inputs_valid(
            double integration_start_time, const std::vector<double> &sorted_timestamps, std::size_t reference_index,
            const lidar_preprocessing_plugins::plugins_context_data::ImuSampleBuffer &imu_samples, double &end_time);

    // Integration parameters
    float m_gravity_mps2{9.80665f};

    // Calibration parameters
    bool m_calibrate_imu{false};
    bool m_approximate_gravity{true};
    bool m_gravity_aligned{true};
    double m_calibration_time_sec{3.0};
    Eigen::Matrix3f m_imu_accel_smatrix{Eigen::Matrix3f::Identity()};
    // [Original] float m_external_velocity_max_speed{0.01f};
    // [Fix] Aligned to YAML value (0.05f) — same as max_speed_mps doc fix
    float m_external_velocity_max_speed{0.05f};
    double m_external_velocity_max_age_sec{0.5};
    bool m_external_velocity_valid{false};
    double m_external_velocity_stamp{0.0};
    double m_external_speed_mps{0.0};
    imu_integrator::CalibrationGateStatus m_calibration_gate_status;
    // IMU state/biases
    imu_integrator::State m_imu_state;
    double m_first_imu_stamp{0.0};
};

}  // namespace lidar_preprocessing_utils
