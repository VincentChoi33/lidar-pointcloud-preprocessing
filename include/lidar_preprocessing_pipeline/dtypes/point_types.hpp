#pragma once

#include <cstdint>
#include <type_traits>

#include <pcl/point_types.h>

namespace lidar_point_types
{

struct EIGEN_ALIGN16 OusterPoint
{
    PCL_ADD_POINT4D;
    float intensity;
    std::uint32_t t;
    std::uint16_t reflectivity;
    std::uint16_t ring;
    std::uint16_t ambient;
    std::uint32_t range;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 VelodynePoint
{
    PCL_ADD_POINT4D;
    float intensity;
    std::uint16_t ring;
    float time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 LivoxPoint
{
    PCL_ADD_POINT4D;
    float intensity;
    std::uint8_t tag;
    std::uint8_t line;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

template<typename>
struct dependent_false : std::false_type
{
    // Helper for static_assert in templates: only triggers on instantiation with an unsupported type.
};

template<typename PointT>
struct PointTimeAccessor
{
    static double time_seconds(const PointT &, double sweep_ref_time)
    {
        static_assert(dependent_false<PointT>::value, "Unsupported point type for deskewing.");
        return sweep_ref_time;
    }
};

template<>
struct PointTimeAccessor<OusterPoint>
{
    static double time_seconds(const OusterPoint &point, double sweep_ref_time)
    {
        return sweep_ref_time + static_cast<double>(point.t) * 1e-9;
    }
};

template<>
struct PointTimeAccessor<VelodynePoint>
{
    static double time_seconds(const VelodynePoint &point, double sweep_ref_time)
    {
        return sweep_ref_time + static_cast<double>(point.time);
    }
};

template<>
struct PointTimeAccessor<LivoxPoint>
{
    static double time_seconds(const LivoxPoint &point, double /*sweep_ref_time*/)
    {
        return point.timestamp * 1e-9;
    }
};

}  // namespace lidar_point_types

POINT_CLOUD_REGISTER_POINT_STRUCT(
        lidar_point_types::OusterPoint,
        (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(std::uint32_t, t, t)(
                std::uint16_t, reflectivity,
                reflectivity)(std::uint16_t, ring, ring)(std::uint16_t, ambient, ambient)(std::uint32_t, range, range))

POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_point_types::VelodynePoint,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity,
                                                                          intensity)(std::uint16_t, ring,
                                                                                     ring)(float, time, time))

POINT_CLOUD_REGISTER_POINT_STRUCT(lidar_point_types::LivoxPoint,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                          std::uint8_t, tag, tag)(std::uint8_t, line, line)(double, timestamp,
                                                                                            timestamp))
