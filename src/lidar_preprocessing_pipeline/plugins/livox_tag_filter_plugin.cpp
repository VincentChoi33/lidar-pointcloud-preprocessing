#include "lidar_preprocessing_pipeline/plugins/livox_tag_filter_plugin.hpp"

#include <type_traits>

#include "lidar_preprocessing_pipeline/dtypes/point_types.hpp"

namespace lidar_preprocessing_plugins
{

template<typename PointT>
void LivoxTagFilterPlugin<PointT>::initialize(const PreprocessingPluginParams &params)
{
    m_enabled = params.livox_tag_filter_params.enabled;
    m_remove_high_confidence_noise = params.livox_tag_filter_params.remove_high_confidence_noise;
    m_remove_moderate_confidence_noise = params.livox_tag_filter_params.remove_moderate_confidence_noise;
    m_remove_low_confidence_noise = params.livox_tag_filter_params.remove_low_confidence_noise;
    m_remove_intensity_noise = params.livox_tag_filter_params.remove_intensity_noise;
    m_output_per_return = params.livox_tag_filter_params.output_per_return;
}

template<typename PointT>
void LivoxTagFilterPlugin<PointT>::process(const PointCloud &input, PointCloud &output) const
{
    static_assert(std::is_same_v<PointT, lidar_point_types::LivoxPoint>,
                  "LivoxTagFilterPlugin only supports Livox point type.");

    if (!m_enabled)
    {
        output = input;
        return;
    }

    output.clear();
    output.reserve(input.size());

    if (m_output_per_return)
    {
        m_return0.clear();
        m_return1.clear();
        m_return2.clear();
        m_return0.reserve(input.size());
        m_return1.reserve(input.size());
        m_return2.reserve(input.size());
    }

    for (const auto &point: input.points)
    {
        // Livox docs: (0,0,0) indicates no object or out-of-range, skip when filtering is enabled.
        if (point.x == 0.0f && point.y == 0.0f && point.z == 0.0f)
            continue;

        // Tag layout (byte): [7:6 reserved][5:4 return][3:2 intensity noise][1:0 spatial noise].
        // Reserved values are always kept; only known noise codes are filtered.
        const std::uint8_t tag = point.tag;
        const std::uint8_t intensity = intensity_group(tag);
        const std::uint8_t spatial = spatial_group(tag);

        // Group 3 (intensity): 01 indicates low-intensity noise (dust/rain/fog/snow).
        if (m_remove_intensity_noise && intensity == kIntensityNoise)
            continue;
        // Group 4 (spatial): 01/10/11 indicate high/moderate/low confidence noise.
        if (m_remove_high_confidence_noise && spatial == kSpatialHighNoise)
            continue;
        if (m_remove_moderate_confidence_noise && spatial == kSpatialModerateNoise)
            continue;
        if (m_remove_low_confidence_noise && spatial == kSpatialLowNoise)
            continue;

        output.push_back(point);

        if (!m_output_per_return)
            continue;

        const std::uint8_t ret = return_number(tag);
        if (ret == 0)
            m_return0.push_back(point);
        else if (ret == 1)
            m_return1.push_back(point);
        else if (ret == 2)
            m_return2.push_back(point);
    }

    output.header = input.header;
    output.is_dense = input.is_dense;

    if (m_output_per_return)
    {
        m_return0.header = input.header;
        m_return1.header = input.header;
        m_return2.header = input.header;
        m_return0.is_dense = input.is_dense;
        m_return1.is_dense = input.is_dense;
        m_return2.is_dense = input.is_dense;
    }
}

template<typename PointT>
std::uint8_t LivoxTagFilterPlugin<PointT>::return_number(std::uint8_t tag)
{
    return static_cast<std::uint8_t>((tag >> 4) & 0x3);
}

template<typename PointT>
std::uint8_t LivoxTagFilterPlugin<PointT>::intensity_group(std::uint8_t tag)
{
    return static_cast<std::uint8_t>((tag >> 2) & 0x3);
}

template<typename PointT>
std::uint8_t LivoxTagFilterPlugin<PointT>::spatial_group(std::uint8_t tag)
{
    return static_cast<std::uint8_t>(tag & 0x3);
}

template<typename PointT>
const typename LivoxTagFilterPlugin<PointT>::PointCloud &LivoxTagFilterPlugin<PointT>::return0_cloud() const
{
    return m_return0;
}

template<typename PointT>
const typename LivoxTagFilterPlugin<PointT>::PointCloud &LivoxTagFilterPlugin<PointT>::return1_cloud() const
{
    return m_return1;
}

template<typename PointT>
const typename LivoxTagFilterPlugin<PointT>::PointCloud &LivoxTagFilterPlugin<PointT>::return2_cloud() const
{
    return m_return2;
}

}  // namespace lidar_preprocessing_plugins

// Explicit template instantiation
template class lidar_preprocessing_plugins::LivoxTagFilterPlugin<lidar_point_types::LivoxPoint>;
