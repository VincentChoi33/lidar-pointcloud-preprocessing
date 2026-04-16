#pragma once

#include <sstream>
#include <string>

#include <ros/node_handle.h>
#include <ros/ros.h>

namespace ros_utils
{
/**
 * @brief Get the mandatory param from rosparam server
 *  calls exit(1) if the param is not found
 *
 * @tparam T param type
 * @param param_name name of the param to get from rosparam server
 * @param param_value reference to the variable to store the param value
 * @param node_handle node handle
 */
template<typename T>
void get_mandatory_param(const std::string &param_name, T &param_value, const ros::NodeHandle &node_handle)
{
    if (!node_handle.getParam(param_name, param_value))
    {
        ROS_FATAL_STREAM("Couldn't find " << param_name << " on the ROS parameter server.");
        exit(1);
    }
}

/**
 * @brief Get the optional param from rosparam server
 *  if the param is not found, the default value is used
 *
 * @tparam T param type
 * @param param_name name of the param to get from rosparam server
 * @param param_value reference to the variable to store the param value
 * @param default_value default value to use if the param is not found
 * @param node_handle node handle
 */
template<typename T>
void get_optional_param(const std::string &param_name, T &param_value, const T &default_value,
                        const ros::NodeHandle &node_handle)
{
    if (node_handle.hasParam(param_name))
    {
        node_handle.getParam(param_name, param_value);
    }
    else
    {
        ROS_WARN("Optional parameter %s not set, using default value", param_name.c_str());
        param_value = default_value;
    }
}

template<typename Iter>
std::string join_strings(Iter begin, Iter end, const std::string &delimiter)
{
    if (begin == end)
        return std::string();

    std::ostringstream out;
    out << *begin;
    ++begin;
    for (; begin != end; ++begin)
    {
        out << delimiter << *begin;
    }
    return out.str();
}

}  // namespace ros_utils
