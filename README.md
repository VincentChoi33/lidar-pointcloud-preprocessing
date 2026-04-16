# LiDAR Preprocessing Pipeline

`lidar_preprocessing_pipeline` is a ROS Noetic nodelet package for LiDAR point-cloud preprocessing with a configurable, sensor-aware pipeline.

The repository packages a practical preprocessing stack that can sit in front of mapping, localization, or downstream perception systems:

- IMU-based deskewing / motion compensation
- gravity alignment
- voxel-grid downsampling
- radius outlier removal
- Livox tag filtering

The implementation is organized as plugins so each stage can be enabled or disabled per sensor and deployment context.

## Design goals

The project is structured around three goals:

1. **geometry-aware preprocessing** for LiDAR sweeps collected on moving platforms
2. **sensor-specific handling** for Ouster, Velodyne, and Livox point types
3. **reproducible engineering** with Docker, launch files, configuration, and visualization helpers

## Supported sensors

- Ouster
- Velodyne
- Livox

## Processing stages

Enabled stages run in this order:

1. Deskew
2. Gravity alignment
3. Voxel grid filter
4. Radius outlier removal
5. Livox tag filter

## Repository layout

```text
.
├── config/                       # ROS params and dynamic reconfigure schema
├── include/                      # Nodelet headers and ROS helpers
├── launch/                       # Launch files for standalone nodelet execution
├── src/                          # Nodelet and plugin implementation
├── viz/                          # Rosbag / visualization helper scripts
├── CMakeLists.txt
├── Dockerfile
├── ENGINEERING_NOTES.md
└── package.xml
```

## Architecture

```text
Inputs
  LiDAR PointCloud2 (Ouster / Velodyne / Livox)
  IMU stream
  Optional external velocity (<namespace>/odom)
         |
         v
ROS nodelet runtime
  - parameter loading
  - ROS subscription / publication
  - plugin selection and execution order
         |
         v
Processing chain
  Deskew
    -> Gravity alignment
    -> Voxel grid filter
    -> Radius outlier removal
    -> Livox tag filter
         |
         +--> Processed PointCloud2 topics
         |
         +--> Gravity-aligned TF frame (when enabled)
                |
                v
Downstream mapping / localization / perception
```

### Component roles

| Component | Responsibility |
|---|---|
| Nodelet manager | Hosts the nodelet runtime inside the ROS nodelet process |
| `LidarPreProcessingNodelet` | Owns ROS I/O, parameter loading, plugin wiring, and output publication |
| Deskew | Compensates scan distortion from platform motion using IMU integration |
| Gravity alignment | Rotates the cloud into a gravity-aligned frame and publishes the derived TF |
| Voxel grid filter | Downsamples the cloud for lighter downstream processing |
| Radius outlier removal | Drops isolated noisy points |
| Livox tag filter | Removes Livox-specific noise / return classes and can emit per-return outputs |

### Data flow summary

- **LiDAR input** provides the raw sweep to preprocess.
- **IMU input** is used for deskewing and gravity alignment.
- **Optional external velocity** gates the initial IMU calibration when enabled.
- The **nodelet** owns ROS I/O, parameter loading, plugin selection, and publication.
- The **plugin chain** applies geometry-changing stages before filtering stages.
- Outputs are published as processed point-cloud topics, with an extra TF frame when gravity alignment is active.

## Quick start

### 1) Build the container

```bash
docker build -t lidar-preprocessing .

docker run -it --rm \
  -v $(pwd):/ws/src/lidar_preprocessing_pipeline \
  -v /path/to/bags:/ws/bags:ro \
  lidar-preprocessing bash
```

Inside the container:

```bash
source /opt/ros/noetic/setup.bash
cd /ws
catkin build lidar_preprocessing_pipeline
source /ws/devel/setup.bash
```

### 2) Launch the nodelet

```bash
roslaunch lidar_preprocessing_pipeline lidar_preprocessing_nodelet.launch \
  namespace:=robot \
  sensor:=Livox
```

### 3) Play a bag and inspect the output

```bash
rosbag play /ws/bags/example.bag --clock
rostopic hz /robot/livox_front/points_processed
```

## ROS interface

### Inputs

- `core/input_topic` (for example `livox_front/points`)
- `imu_params/topic` when deskew or gravity alignment is enabled
- `<namespace>/odom` when IMU calibration uses external-velocity gating

### Outputs

The nodelet publishes processed clouds derived from the input topic prefix.

Example for `livox_front/points`:

- `livox_front/points_processed`
- `livox_front/points2_processed` and `livox_front/points3_processed` when Livox per-return output is enabled

## Frames and TF

When gravity alignment is enabled, the processed cloud is published in a derived frame:

- input frame: `<input_frame>`
- gravity-aligned frame: `<input_frame>_gravity_frame`

A TF transform is also published from `<input_frame>` to `<input_frame>_gravity_frame`.

## Configuration

The default configuration lives in `config/lidar_preprocessing_params.yaml`.

Important parameters:

- `core/input_topic`
- `imu_params/topic`
- `imu_params/buffer_size`
- `imu_params/calibration/*`
- `pointcloud_deskew/enabled`
- `gravity_align/enabled`
- `voxel_grid_filter/*`
- `radius_outlier_removal/*`
- `livox_tag_filter/*`

### Example

```yaml
core:
  input_topic: "livox_front/points"

imu_params:
  topic: "livox_front/imu"
  buffer_size: 4.0

  calibration:
    enabled: true
    time: 3
    external_velocity:
      max_speed_mps: 0.05
      max_age_sec: 0.5

  integration:
    integration_params:
      approximate_gravity: true
      gravity_mps2: 9.80665

pointcloud_deskew:
  enabled: true

gravity_align:
  enabled: true

voxel_grid_filter:
  enabled: false
  leaf_size: 0.1

radius_outlier_removal:
  enabled: false
  radius: 0.5
  min_neighbors: 5

livox_tag_filter:
  enabled: true
  remove_high_confidence_noise: true
  remove_moderate_confidence_noise: true
  remove_low_confidence_noise: false
  remove_intensity_noise: true
  output_per_return: false
```

## Livox tag filtering

When enabled, the Livox filter can:

- drop out-of-range `(0, 0, 0)` points
- remove configurable spatial / intensity noise groups
- optionally emit separate clouds per return channel

For tag semantics, refer to the public Livox sensor documentation for your device generation.

## Visualization helpers

The `viz/` directory contains utilities for:

- extracting point-cloud frames from rosbags
- generating lightweight HTML point-cloud animations
- recording processed output for batches of bags

These helpers are optional and are intended for inspection, debugging, and lightweight result review.

## Engineering notes

See [`ENGINEERING_NOTES.md`](ENGINEERING_NOTES.md) for a concise summary of implementation decisions, validation focus, and known follow-up items.

## Attribution

The deskew / IMU integration approach draws on ideas from direct LiDAR-inertial odometry pipelines, including DLIO:

- https://github.com/vectr-ucla/direct_lidar_inertial_odometry

## License

Apache License 2.0. See [`LICENSE`](LICENSE).
