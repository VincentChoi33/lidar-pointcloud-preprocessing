cmake_minimum_required(VERSION 3.0.2)
project(lidar_preprocessing_pipeline VERSION 0.1.0 LANGUAGES CXX)

# Set C++ Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# For PCL warning
set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1)

# Required Deps
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    nodelet
    pluginlib
    pcl_conversions
    pcl_ros
    dynamic_reconfigure
    tf2_ros
)

find_package(PCL 1.10.0 REQUIRED)
find_package(OpenMP)

# CXX Args
set(CXX_FLAGS "-Wall -pedantic")
set(CMAKE_CXX_FLAGS "${CXX_FLAGS}")

# Dynamic Reconfigure
generate_dynamic_reconfigure_options(
    config/PreprocessingParams.cfg
)

# Catkin Package
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_core ${PROJECT_NAME}_plugins
    CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs sensor_msgs nodelet pluginlib pcl_conversions pcl_ros
                   dynamic_reconfigure tf2_ros
    DEPENDS PCL
)

# Force build type to Release if it isn't provided
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
endif()

# Include directories
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
)

# Link Directories
link_directories(
    ${PCL_LIBRARY_DIRS}
)

# Definitions for external libs
add_definitions(
    ${PCL_DEFINITIONS}
)

# Add coverage flags if building in Debug or Test mode
if(CMAKE_BUILD_TYPE MATCHES "Debug|Test")
    add_definitions(-DPROFILE=1)
    add_definitions(-DDEBUG=1)
    add_compile_options(-g --coverage -O0 -fprofile-arcs -ftest-coverage)
    add_link_options(--coverage)
    link_libraries(gcov)
else()
    add_definitions(-DPROFILE=1)
    add_definitions(-DDEBUG=0)
    add_compile_options(-O3)
endif()


# Nodelet
# Keep nodelet sources separate from plugin implementation sources.
# RELATIVE paths make the exclusion pattern stable across workspaces.
file(GLOB_RECURSE SOURCES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" CONFIGURE_DEPENDS "src/*.cpp")
list(FILTER SOURCES EXCLUDE REGEX "^src/lidar_preprocessing_pipeline/.*")
add_library(${PROJECT_NAME} ${SOURCES})
target_compile_definitions(${PROJECT_NAME} PRIVATE PCL_NO_PRECOMPILE)
target_link_libraries(
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
if(OpenMP_CXX_FOUND)
    target_compile_options(${PROJECT_NAME} PRIVATE ${OpenMP_CXX_FLAGS})
    target_link_libraries(${PROJECT_NAME} ${OpenMP_CXX_FLAGS})
endif()
if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_LIBRARIES} ${PROJECT_NAME}_gencfg)
endif()


# Pre-Processing Plugins
file(GLOB_RECURSE PLUGIN_SOURCES CONFIGURE_DEPENDS "src/lidar_preprocessing_pipeline/*.cpp")
set(PLUGIN_NAME "${PROJECT_NAME}_plugins")
add_library(${PLUGIN_NAME} ${PLUGIN_SOURCES})
target_compile_definitions(${PLUGIN_NAME} PRIVATE PCL_NO_PRECOMPILE)
target_link_libraries(
    ${PLUGIN_NAME}
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
)
if(OpenMP_CXX_FOUND)
    target_compile_options(${PLUGIN_NAME} PRIVATE ${OpenMP_CXX_FLAGS})
    target_link_libraries(${PLUGIN_NAME} ${OpenMP_CXX_FLAGS})
endif()
if(catkin_EXPORTED_LIBRARIES)
    add_dependencies(${PLUGIN_NAME} ${catkin_EXPORTED_LIBRARIES})
endif()

install(DIRECTORY launch config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(FILES lidar_preprocessing_plugins.xml nodelet_plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(
  TARGETS ${PLUGIN_NAME} ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
