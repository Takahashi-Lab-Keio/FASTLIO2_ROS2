# FASTLIO2_ROS2 — Ouster/Jazzy PoC fork

This branch adapts upstream FASTLIO2_ROS2 for an Ouster OS1-32 and ROS 2 Jazzy.
It is based on upstream commit `f516daac08bc46e50e814a2e7d6c8352ed8141bb`.

## Supported input

- `/ouster/points` (`sensor_msgs/msg/PointCloud2`) in `os_lidar`
- `/ouster/imu` (`sensor_msgs/msg/Imu`) in `os_imu`
- Ouster "original" point fields: scalar `float32 x/y/z/intensity` and scalar
  `uint32 t`

The `t` field is interpreted as nanoseconds from the cloud header timestamp.
Clouds with missing/malformed fields, no usable points, non-finite values, or a
point offset above `maximum_point_time_ms` are rejected. Set the ROS parameter
`require_point_time:=false` only for a simulator smoke test; real data must
contain per-point time.

Ouster IMU acceleration is already SI (`m/s^2`), so `imu_acc_scale` defaults to
`1.0`. The old Livox driver and its fixed acceleration multiplier are not used.

## Frames and outputs

The filter state remains in the IMU frame. Published pose, path, TF, and
`body_cloud` are converted to `base_footprint` using `r_bi`/`t_bi`; the first
valid base pose anchors `fastlio_odom` when `anchor_output_frame` is true.

With the LIO node in namespace `/fastlio2`, it publishes:

- `body_cloud`, `world_cloud`, `lio_path`, and `lio_odom`
- TF `fastlio_odom -> base_footprint`

ROS parameter overrides for `require_point_time`, `maximum_point_time_ms`,
`imu_acc_scale`, and `anchor_output_frame` take precedence over values in the
file given by `config_path`.

## Mapping and localization

The packages are named `fastlio2`, `fastlio2_interfaces`, `fastlio2_pgo`, and
`fastlio2_localizer`. HBA is intentionally excluded with `COLCON_IGNORE`.

Standalone launches are available for development:

```bash
ros2 launch fastlio2_pgo pgo_launch.py
ros2 launch fastlio2_localizer localizer_launch.py
```

The PGO node is expected in namespace `/fastlio2/pgo` and provides relative
service `save_maps`. It refuses to write into a non-empty directory, writes a
binary `map.pcd` plus a populated `poses.txt`, and optionally writes `patches/`.

```bash
mkdir -p /tmp/fastlio2-map
ros2 service call /fastlio2/pgo/save_maps \
  fastlio2_interfaces/srv/SaveMaps \
  "{file_path: '/tmp/fastlio2-map', save_patches: false}"
```

The localizer is expected in namespace `/fastlio2/localizer`. A relocalize
response only acknowledges that the map and initial pose were accepted. Poll
`relocalize_check` with `code: 0` for actual ICP success. No `map ->
fastlio_odom` TF is broadcast until that success occurs.

PGO and the localizer both own `map -> fastlio_odom`; never run them together.

## Validation

The `fastlio2` tests cover Ouster PointCloud2 conversion, stable point-time
sorting, malformed/empty input, the corrected rotation Jacobian, IMU scaling,
base/lidar extrinsics, output anchoring, and timestamp regression policy.
A clock regression greater than 0.5 seconds terminates `lio_node`; restart the
node before replaying a bag whose clock resets.
