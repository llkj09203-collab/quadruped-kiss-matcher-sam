# Mapping data product

`kiss_matcher_sam` stores one mapping session below
`result.output_directory/result.seq_name`.

```text
<output_directory>/<sequence>/
├── metadata.yaml
├── timestamps.txt
├── odometry_tum.txt
├── optimized_trajectory_tum.txt
├── keyframes/
│   ├── 000000.pcd
│   └── ...
└── <sequence>_map.pcd
```

- `keyframes/*.pcd`: LiDAR-frame keyframe scans, named by graph node index.
- `timestamps.txt`: graph node index and ROS timestamp in seconds.
- `odometry_tum.txt`: uncorrected FAST-LIO keyframe trajectory.
- `optimized_trajectory_tum.txt`: latest GTSAM trajectory in TUM format.
- `<sequence>_map.pcd`: final map reconstructed from keyframes and optimized poses.
- `metadata.yaml`: frames, format version, and relevant mapping parameters.

The point clouds and trajectory files form one atomic data product: do not copy a
map without its keyframes, timestamps, trajectory, and metadata. GPS altitude is
stored only as a graph constraint; latitude and longitude are not used as global
map coordinates.
