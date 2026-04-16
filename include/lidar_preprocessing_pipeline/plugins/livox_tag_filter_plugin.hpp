#pragma once

#include <pcl/point_cloud.h>

#include "lidar_preprocessing_pipeline/dtypes/plugin_params.hpp"
#include "lidar_preprocessing_pipeline/plugins/plugin_interface.hpp"

namespace lidar_preprocessing_plugins
{
template<typename PointT>
class LivoxTagFilterPlugin : public ILidarPreProcessingPlugin<PointT>
{
public:
    using PointCloud = pcl::PointCloud<PointT>;

    LivoxTagFilterPlugin() = default;
    ~LivoxTagFilterPlugin() = default;

    void initialize(const PreprocessingPluginParams &params) override;

    void process(const PointCloud &input, PointCloud &output) const override;

    const PointCloud &return0_cloud() const;
    const PointCloud &return1_cloud() const;
    const PointCloud &return2_cloud() const;

private:
    static constexpr std::uint8_t kIntensityNoise = 0x1;
    static constexpr std::uint8_t kSpatialHighNoise = 0x1;
    static constexpr std::uint8_t kSpatialModerateNoise = 0x2;
    static constexpr std::uint8_t kSpatialLowNoise = 0x3;

    static std::uint8_t return_number(std::uint8_t tag);
    static std::uint8_t intensity_group(std::uint8_t tag);
    static std::uint8_t spatial_group(std::uint8_t tag);

    bool m_enabled{false};
    bool m_remove_high_confidence_noise{false};
    bool m_remove_moderate_confidence_noise{false};
    bool m_remove_low_confidence_noise{false};
    bool m_remove_intensity_noise{false};
    bool m_output_per_return{false};

    mutable PointCloud m_return0;
    mutable PointCloud m_return1;
    mutable PointCloud m_return2;
};
}  // namespace lidar_preprocessing_plugins
