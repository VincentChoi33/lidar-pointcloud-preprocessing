#include "lidar_preprocessing_pipeline/utils/imu_integrator.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lidar_preprocessing_utils
{

ImuIntegrator::ImuIntegrator(const imu_integrator::ImuIntegratorParams &imu_integrator_params)
    : m_gravity_mps2(imu_integrator_params.gravity_mps2),
      m_calibrate_imu(imu_integrator_params.imu_calibration_params.enabled),
      m_approximate_gravity(imu_integrator_params.approximate_gravity),
      m_calibration_time_sec(imu_integrator_params.imu_calibration_params.calibration_time_sec),
      m_external_velocity_max_speed(imu_integrator_params.imu_calibration_params.external_velocity.max_speed_mps),
      m_external_velocity_max_age_sec(imu_integrator_params.imu_calibration_params.external_velocity.max_age_sec)
{
    m_imu_state.p.setZero();
    m_imu_state.q.setIdentity();
    m_imu_state.v.setZero();
    m_imu_state.imu_bias.accel_bias = imu_integrator_params.imu_calibration_params.accel_bias;
    m_imu_state.imu_bias.gyro_bias = imu_integrator_params.imu_calibration_params.gyro_bias;

    if (m_calibrate_imu)
    {
        // During calibration, follow DLIO and use raw accel samples for bias estimation.
        m_imu_accel_smatrix.setIdentity();
    }
    else
    {
        m_imu_accel_smatrix = imu_integrator_params.imu_calibration_params.imu_accel_smatrix;
    }

    m_calibration_gate_status.calibration_enabled = m_calibrate_imu;
    m_calibration_gate_status.speed_limit_mps = m_external_velocity_max_speed;
    m_calibration_gate_status.velocity_max_age_sec = m_external_velocity_max_age_sec;
}

Eigen::Quaternionf ImuIntegrator::integrate_quaternion(const Eigen::Quaternionf &q, const Eigen::Vector3f &omega,
                                                       float dt)
{
    Eigen::Quaternionf q_next(q.w() - 0.5f * (q.x() * omega[0] + q.y() * omega[1] + q.z() * omega[2]) * dt,
                              q.x() + 0.5f * (q.w() * omega[0] - q.z() * omega[1] + q.y() * omega[2]) * dt,
                              q.y() + 0.5f * (q.z() * omega[0] + q.w() * omega[1] - q.x() * omega[2]) * dt,
                              q.z() + 0.5f * (q.x() * omega[1] - q.y() * omega[0] + q.w() * omega[2]) * dt);
    q_next.normalize();
    return q_next;
}

bool ImuIntegrator::execute_imu_calibration(
        const lidar_preprocessing_plugins::plugins_context_data::ImuSampleBuffer &imu_samples)
{
    m_calibration_gate_status.calibration_enabled = m_calibrate_imu;
    m_calibration_gate_status.blocked = false;
    m_calibration_gate_status.external_velocity_valid = m_external_velocity_valid;
    m_calibration_gate_status.external_velocity_stale = false;
    m_calibration_gate_status.external_velocity_too_fast = false;
    m_calibration_gate_status.speed_mps = m_external_speed_mps;
    m_calibration_gate_status.speed_limit_mps = m_external_velocity_max_speed;
    m_calibration_gate_status.velocity_age_sec = 0.0;
    m_calibration_gate_status.velocity_max_age_sec = m_external_velocity_max_age_sec;

    if (!m_calibrate_imu)
        return true;

    if (imu_samples.empty())
        return false;

    const double latest_imu_stamp = imu_samples.back().stamp;
    if (m_external_velocity_valid)
        m_calibration_gate_status.velocity_age_sec = latest_imu_stamp - m_external_velocity_stamp;

    if (!m_external_velocity_valid)
    {
        m_calibration_gate_status.blocked = true;
        m_first_imu_stamp = 0.0;
        return false;
    }

    if (m_external_velocity_max_age_sec > 0.0)
    {
        // Only block calibration when the external-velocity estimate is too old.
        if (m_calibration_gate_status.velocity_age_sec > m_external_velocity_max_age_sec)
        {
            m_calibration_gate_status.blocked = true;
            m_calibration_gate_status.external_velocity_stale = true;
            m_first_imu_stamp = 0.0;
            return false;
        }
    }

    if (m_external_speed_mps > static_cast<double>(m_external_velocity_max_speed))
    {
        m_calibration_gate_status.blocked = true;
        m_calibration_gate_status.external_velocity_too_fast = true;
        m_first_imu_stamp = 0.0;
        return false;
    }

    if (m_first_imu_stamp == 0.0)
        m_first_imu_stamp = latest_imu_stamp;

    double calibration_end_time = m_first_imu_stamp + m_calibration_time_sec;
    if (latest_imu_stamp < calibration_end_time)
    {
        // Not enough data yet.
        return false;
    }

    Eigen::Vector3f gyro_avg = Eigen::Vector3f::Zero();
    Eigen::Vector3f accel_avg = Eigen::Vector3f::Zero();
    int num_samples = 0;

    for (const auto &sample: imu_samples)
    {
        if (sample.stamp < m_first_imu_stamp)
            continue;
        if (sample.stamp > calibration_end_time)
            break;
        gyro_avg += sample.ang_vel;
        accel_avg += sample.lin_accel;
        ++num_samples;
    }

    if (num_samples == 0)
        return false;

    gyro_avg /= static_cast<float>(num_samples);
    accel_avg /= static_cast<float>(num_samples);

    // Estimate gravity direction from the averaged acceleration.
    Eigen::Vector3f gravity_vec(0.f, 0.f, m_gravity_mps2);
    if (m_approximate_gravity)
    {
        Eigen::Vector3f accel_unbiased = accel_avg - m_imu_state.imu_bias.accel_bias;
        float accel_norm = accel_unbiased.norm();
        if (accel_norm > std::numeric_limits<float>::epsilon())
        {
            gravity_vec = accel_unbiased / accel_norm * std::abs(m_gravity_mps2);
        }

        Eigen::Quaternionf gravity_q =
                Eigen::Quaternionf::FromTwoVectors(gravity_vec, Eigen::Vector3f(0.f, 0.f, m_gravity_mps2));
        m_imu_state.q = gravity_q;
    }

    // Bias estimation matches DLIO: accel bias absorbs gravity, gyro bias is mean angular velocity.
    m_imu_state.imu_bias.accel_bias = accel_avg - gravity_vec;
    m_imu_state.imu_bias.gyro_bias = gyro_avg;

    m_calibrate_imu = false;
    m_calibration_gate_status.calibration_enabled = false;
    m_calibration_gate_status.blocked = false;
    return true;
}


void ImuIntegrator::set_state(const imu_integrator::State &state)
{
    m_imu_state = state;
}

void ImuIntegrator::set_external_velocity(double speed_mps, double stamp, bool valid)
{
    m_external_speed_mps = speed_mps;
    m_external_velocity_stamp = stamp;
    m_external_velocity_valid = valid;
    m_calibration_gate_status.calibration_enabled = m_calibrate_imu;
    m_calibration_gate_status.blocked = false;
    m_calibration_gate_status.external_velocity_valid = valid;
    m_calibration_gate_status.external_velocity_stale = false;
    m_calibration_gate_status.external_velocity_too_fast = false;
    m_calibration_gate_status.speed_mps = speed_mps;
    m_calibration_gate_status.speed_limit_mps = m_external_velocity_max_speed;
    m_calibration_gate_status.velocity_age_sec = 0.0;
    m_calibration_gate_status.velocity_max_age_sec = m_external_velocity_max_age_sec;
}

imu_integrator::CalibrationGateStatus ImuIntegrator::calibration_gate_status() const
{
    return m_calibration_gate_status;
}

bool ImuIntegrator::are_integration_inputs_valid(
        double integration_start_time, const std::vector<double> &sorted_timestamps, std::size_t reference_index,
        const lidar_preprocessing_plugins::plugins_context_data::ImuSampleBuffer &imu_samples, double &end_time)
{
    // Need timestamps to integrate, a valid reference index, and at least two IMU samples.
    if (sorted_timestamps.empty() || imu_samples.size() < 2 || reference_index >= sorted_timestamps.size())
        return false;

    // Integration must begin at or before the first requested pose time.
    if (integration_start_time > sorted_timestamps.front())
        return false;

    // Cannot integrate without IMU coverage at the integration start time.
    if (integration_start_time < imu_samples.front().stamp)
        return false;

    end_time = sorted_timestamps.back();
    // Require IMU data that covers the end of the requested sweep.
    if (imu_samples.back().stamp < end_time)
        return false;

    return true;
}

imu_integrator::IntegrationResult
ImuIntegrator::integrate(double integration_start_time, const std::vector<double> &sorted_timestamps,
                         std::size_t reference_index,
                         const lidar_preprocessing_plugins::plugins_context_data::ImuSampleBuffer &imu_samples)
{
    imu_integrator::IntegrationResult result;

    // Skip integration until the initial bias/gravity calibration window completes.
    if (m_calibrate_imu && !execute_imu_calibration(imu_samples))
        return result;

    double end_time = 0.0;
    if (!are_integration_inputs_valid(integration_start_time, sorted_timestamps, reference_index, imu_samples,
                                      end_time))
        return result;

    // Apply bias + scale correction
    lidar_preprocessing_plugins::plugins_context_data::ImuSampleBuffer corrected_samples = imu_samples;
    for (auto &sample: corrected_samples)
    {
        sample.lin_accel = (m_imu_accel_smatrix * sample.lin_accel) - m_imu_state.imu_bias.accel_bias;
        sample.ang_vel = sample.ang_vel - m_imu_state.imu_bias.gyro_bias;
    }

    // Find the IMU sample just before/at the integration start to seed the propagation.
    std::size_t start_index = 0;
    while (start_index + 1 < corrected_samples.size() &&
           corrected_samples[start_index + 1].stamp <= integration_start_time)
    {
        ++start_index;
    }

    // Require a following sample to interpolate angular velocity/acceleration.
    if (start_index + 1 >= corrected_samples.size())
        return result;

    // Find the IMU sample that reaches the end of the requested sweep.
    std::size_t end_index = start_index + 1;
    while (end_index < corrected_samples.size() && corrected_samples[end_index].stamp < end_time)
    {
        ++end_index;
    }

    // End time must be covered by the IMU buffer.
    if (end_index >= corrected_samples.size())
        return result;

    // Initialize State from Global Tracker
    imu_integrator::State init_state = m_imu_state;

    // Reset position to zero at integration start time since this is relative integration.
    init_state.p = Eigen::Vector3f::Zero();
    init_state.imu_bias = m_imu_state.imu_bias;

    // Keep velocity and bias across sweeps while resetting position for relative deskewing.

    const auto &start_sample = corrected_samples[start_index];
    const auto &next_sample = corrected_samples[start_index + 1];
    if (next_sample.dt <= 0.0)
        return result;

    auto sample_dt = static_cast<float>(next_sample.dt);
    auto start_offset_s = static_cast<float>(integration_start_time - start_sample.stamp);

    Eigen::Vector3f ang_vel_delta = next_sample.ang_vel - start_sample.ang_vel;
    Eigen::Vector3f ang_vel_slope = ang_vel_delta / sample_dt;

    // Back-propagate or Interpolate to find exact state at 'start_sample' time
    // (Assuming init_state is valid at 'integration_start_time')
    // We take start + 0.5 * change to get average angular velocity over the offset duration
    // Why negative? Because we are going backwards in time from init_state to start_sample
    Eigen::Vector3f omega_start = -(start_sample.ang_vel + 0.5f * ang_vel_slope * start_offset_s);
    Eigen::Quaternionf orientation_start = integrate_quaternion(init_state.q, omega_start, start_offset_s);

    // Now, we calculate the orientation at next_sample time for acceleration jerk
    Eigen::Vector3f omega_avg = start_sample.ang_vel + 0.5f * ang_vel_delta;
    Eigen::Quaternionf orientation_next = integrate_quaternion(orientation_start, omega_avg, sample_dt);

    // The IMU measures forces relative to the robot's tilt. If the robot is tilted 45 degrees,
    // the sensor feels gravity splitting into Y and Z axes. We use the quaternion
    // to rotate this vector so it aligns with the global map (North/East/Up).
    Eigen::Vector3f accel_start = orientation_start._transformVector(start_sample.lin_accel);

    // The accelerometer measures "Proper Acceleration" (G-Force). Even when stopped,
    // the ground pushes the robot Up at 9.81 m/s^2
    // Now that we rotated the vector to the World Frame (where Z is always Up),
    // we simply subtract gravity from the Z-axis to leave only "Pure Motion" acceleration
    accel_start[2] -= m_gravity_mps2;

    // We need to know the acceleration at the END of this time step as well
    // This allows us to see how the acceleration is CHANGING (Jerk) over time
    Eigen::Vector3f accel_next = orientation_next._transformVector(next_sample.lin_accel);
    accel_next[2] -= m_gravity_mps2;

    // Jerk is the rate of change of acceleration.
    // By comparing the start (accel_start) and end (accel_next) accelerations,
    // we define the slope of the "Push." This allows for 3rd-order integration
    Eigen::Vector3f accel_jerk = (accel_next - accel_start) / sample_dt;

    // Back propagate position and velocity to the exact integration start time
    Eigen::Vector3f velocity =
            init_state.v - accel_start * start_offset_s - 0.5f * accel_jerk * start_offset_s * start_offset_s;
    Eigen::Vector3f position = init_state.p - velocity * start_offset_s -
                               0.5f * accel_start * start_offset_s * start_offset_s -
                               (1.0f / 6.0f) * accel_jerk * start_offset_s * start_offset_s * start_offset_s;

    Eigen::Quaternionf orientation = orientation_start;
    Eigen::Vector3f accel_world = orientation._transformVector(start_sample.lin_accel);
    accel_world[2] -= m_gravity_mps2;

    auto stamp_it = sorted_timestamps.begin();
    result.poses.reserve(sorted_timestamps.size());

    // Main Integration Loop
    for (std::size_t idx = start_index + 1; idx <= end_index; ++idx)
    {
        const auto &prev_sample = corrected_samples[idx - 1];
        const auto &curr_sample = corrected_samples[idx];
        if (curr_sample.dt <= 0.0)
            return imu_integrator::IntegrationResult{};

        sample_dt = static_cast<float>(curr_sample.dt);

        ang_vel_delta = curr_sample.ang_vel - prev_sample.ang_vel;
        ang_vel_slope = ang_vel_delta / sample_dt;
        omega_avg = prev_sample.ang_vel + 0.5f * ang_vel_delta;

        orientation = integrate_quaternion(orientation, omega_avg, sample_dt);

        Eigen::Vector3f accel_prev = accel_world;
        accel_world = orientation._transformVector(curr_sample.lin_accel);
        accel_world[2] -= m_gravity_mps2;

        Eigen::Vector3f accel_delta = accel_world - accel_prev;
        accel_jerk = accel_delta / sample_dt;

        // Interpolate poses at requested timestamps between IMU samples.
        while (stamp_it != sorted_timestamps.end() && *stamp_it <= curr_sample.stamp)
        {
            auto interp_dt = static_cast<float>(*stamp_it - prev_sample.stamp);

            Eigen::Vector3f omega_interp = prev_sample.ang_vel + 0.5f * ang_vel_slope * interp_dt;
            Eigen::Quaternionf orientation_interp = integrate_quaternion(orientation, omega_interp, interp_dt);

            Eigen::Vector3f position_interp = position + velocity * interp_dt +
                                              0.5f * accel_prev * interp_dt * interp_dt +
                                              (1.0f / 6.0f) * accel_jerk * interp_dt * interp_dt * interp_dt;
            Eigen::Vector3f velocity_interp =
                    velocity + accel_prev * interp_dt + 0.5f * accel_jerk * interp_dt * interp_dt;

            Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();
            pose.block<3, 3>(0, 0) = orientation_interp.toRotationMatrix();
            pose.block<3, 1>(0, 3) = position_interp;

            std::size_t pose_index = result.poses.size();
            result.poses.push_back(pose);

            if (pose_index == reference_index)
            {
                result.reference_state.q = orientation_interp;
                result.reference_state.p = position_interp;
                result.reference_state.v = velocity_interp;
                result.reference_state.imu_bias = m_imu_state.imu_bias;
                result.reference_state_valid = true;
            }

            ++stamp_it;
        }

        position += velocity * sample_dt + 0.5f * accel_prev * sample_dt * sample_dt +
                    (1.0f / 6.0f) * accel_delta * sample_dt * sample_dt;
        velocity += accel_prev * sample_dt + 0.5f * accel_delta * sample_dt;
    }

    result.success = (result.poses.size() == sorted_timestamps.size());
    return result;
}

}  // namespace lidar_preprocessing_utils
