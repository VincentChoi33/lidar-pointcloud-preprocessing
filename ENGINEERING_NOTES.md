# Engineering Notes

## What this repository demonstrates

This repository is a compact ROS LiDAR preprocessing project that demonstrates:

- plugin-based preprocessing design for multiple LiDAR point types
- IMU-aware sweep deskewing and gravity alignment
- sensor-specific filtering for Livox, Ouster, and Velodyne data
- practical ROS packaging with launch files, Docker, and visualization helpers

## Recent engineering focus

The recent cleanup work in this repository focused on making the pipeline easier to understand and more reproducible as a standalone public project:

- simplified the public-facing documentation
- removed delivery-specific debugging commentary from the codebase
- standardized launch defaults and helper-script defaults
- kept the code comments focused on behavior and implementation rather than issue-tracker style narration

## Validation approach

The pipeline was validated through a mix of:

- `catkin build` in a ROS Noetic workspace
- launch-time checks for plugin loading and topic publication
- bag replay with topic / frame inspection
- point-cloud visualization using the scripts in `viz/`

## Useful things to inspect in the code

- `src/lidar_preprocessing_nodelet.cpp` for pipeline orchestration and ROS I/O
- `src/lidar_preprocessing_pipeline/plugins/pointcloud_deskew_plugin.cpp` for per-point deskew logic
- `src/lidar_preprocessing_pipeline/utils/imu_integrator.cpp` for IMU state propagation
- `lidar_preprocessing_plugins.xml` for plugin registration

## Known extension ideas

Potential follow-up directions if the project is expanded further:

- add automated tests around plugin ordering and frame semantics
- add benchmark scripts for point throughput and latency
- add richer examples for non-Livox sensors
- split visualization helpers into a small tools package
