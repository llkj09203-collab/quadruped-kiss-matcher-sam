# KISS-SAM regression baseline

This document freezes the behavior at Git tag
`baseline/kiss-sam-before-refactor`. Refactoring changes must be compared against
this baseline before they are accepted.

## Dataset

- Bag: `fastlio2_ws/data/test/实验室场景/raw_20261115_171734_0.db3`
- Duration: 214.124759415 seconds
- LiDAR: `/livox/lidar`, `sensor_msgs/msg/PointCloud2`, 2140 messages
- IMU: `/livox/imu`, `sensor_msgs/msg/Imu`, 42825 messages
- Recorded odometry is deliberately excluded from playback.

## Launch contract

FAST-LIO publishes `/odometry` and `/cloud_registered`. KISS-SAM consumes those
topics through `/odom` and `/cloud` remappings. GPS altitude is disabled for the
baseline.

```bash
ros2 launch spark_fast_lio mapping_mid360.launch.yaml start_rviz:=false

ros2 run kiss_matcher_ros kiss_matcher_sam --ros-args \
  --params-file src/KISS-Matcher/ros/config/slam_config.yaml \
  -p result.seq_name:=lab_regression \
  -p result.output_directory:=/tmp/kiss_sam_regression \
  -r /odom:=/odometry \
  -r /cloud:=/cloud_registered

ros2 bag play "data/test/实验室场景/raw_20261115_171734_0.db3" \
  --topics /livox/lidar /livox/imu
```

Stop KISS-SAM with `SIGINT` after playback so final products are flushed.

## Required invariants

- All three processes exit without a crash.
- Timestamps are strictly increasing.
- Keyframe PCD count equals timestamp count and odometry trajectory count.
- Optimized trajectory contains one header plus one pose per keyframe.
- Metadata format version is `1`.
- Every PCD is readable and contains at least one point.
- Final map exists and contains at least one point.
- No NaN or infinity occurs in either trajectory.

## Reference observations

The earlier 84.958-second small-motion recording produced 68 keyframes. It is
useful for fast smoke tests, but it is not a campus-loop accuracy benchmark. The
214-second laboratory recording is the primary refactoring regression bag.

Numerical trajectory equality is not required because asynchronous ROS timers
can change candidate timing. Any material change in keyframe count, accepted
loops, endpoint pose, or map extent must be reviewed rather than silently
accepted.
