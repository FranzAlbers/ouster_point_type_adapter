# Ouster PointCloud Adapter

This package provides a ROS2 composable node for converting a `PointCloud2` message from Ouster Format to Autoware Format.

## Features

- Subscribes to `PointCloud2` messages in [Ouster Format](https://github.com/ouster-lidar/ouster-ros/blob/master/include/ouster_ros/os_point.h).
- Publishes `PointCloud2` messages in [Autoware Format](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/sensing/data-types/point-cloud/).

## Usage

1. Build the package:
   ```bash
   colcon build --symlink-install --packages-select ouster_point_type_adapter
2. Adapt topic remappings in `ouster_point_type_adapter.launch.py` as needed.
3. Launch `ouster_point_type_adapter` component in a new container:
   ```bash
   ros2 launch ouster_point_type_adapter ouster_point_type_adapter.launch.py
