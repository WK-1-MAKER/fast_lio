# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build

This is a ROS1 (Catkin) package. Build from the workspace root:

```bash
cd ~/fast_lio  # or your catkin workspace root
catkin_make
source devel/setup.bash
```

Requirements: ROS >= Melodic, PCL >= 1.8, Eigen >= 3.3.4, `livox_ros_driver` must be sourced before building.

The CMakeLists.txt uses C++14 with O3 optimization and OpenMP. It sets `MP_EN` based on CPU architecture (different thread counts for x86 vs ARM).

## Running

Each supported LiDAR has its own launch file:

```bash
roslaunch fast_lio mapping_avia.launch        # Livox Avia
roslaunch fast_lio mapping_velodyne.launch    # Velodyne
roslaunch fast_lio mapping_ouster64.launch    # Ouster 64-line
roslaunch fast_lio mapping_marsim.launch      # MARSIM simulator
roslaunch fast_lio fusion_framework.launch    # Livox + mmWave radar fusion
```

Then play a rosbag in a second terminal. Output PCD map saves to `PCD/scans.pcd`.

Key launch parameters (set per-launch or overridden on command line):
- `point_filter_num`: keep every Nth point (downsampling)
- `filter_size_surf` / `filter_size_map`: voxel sizes in meters
- `max_iteration`: EKF iterations per frame (default 3)
- `cube_side_length`: local map window size in meters

## Testing

No automated test suite. Validation is done via rosbag playback and visual inspection of the resulting `PCD/scans.pcd` with `pcl_viewer scans.pcd`.

Debug logs are written under `Log/` and can be plotted with `Log/plot.py`. The `DEBUG_FILE_DIR(name)` macro in `include/common_lib.h` controls log file paths.

## Architecture

### Core pipeline

```
LiDAR callback → ring buffer → sync with IMU → preprocess →
  motion undistortion (IMU) → downsample → ikd-Tree nearest-neighbor search →
  iterated EKF update → map update → publish odometry/path/cloud
```

### Key files

| File | Role |
|------|------|
| `src/laserMapping.cpp` | Main node: EKF state machine, map management, all publishers/subscribers |
| `src/preprocess.cpp` + `preprocess.h` | Per-LiDAR-type point cloud conversion and optional feature extraction |
| `src/IMU_Processing.hpp` | IMU initialization, motion undistortion, process noise |
| `include/use-ikfom.hpp` | IKFOM manifold EKF state definition (`state_ikfom`), `df_dx`/`df_dw` Jacobians |
| `include/common_lib.h` | Shared structs (`MeasureGroup`, `StatesGroup`), constants (`DIM_STATE=18`) |
| `include/ikd-Tree/ikd_Tree.cpp` | Incremental KD-Tree (insert, delete, k-NN search, auto-rebuild) |
| `src/radar_livox_fusion_node.cpp` | Standalone node fusing mmWave radar with Livox (smoke-adaptive) |

### EKF state vector (18D on manifold)

`state_ikfom` in `include/use-ikfom.hpp`:
- position (3D), rotation SO(3), LiDAR-IMU extrinsic R/T (6D), velocity (3D), gyro bias (3D), accel bias (3D), gravity on S2 (2D)

Online extrinsic calibration is enabled via `extrinsic_est_en: true` in the YAML config.

### LiDAR type enum (in `preprocess.h`)

```cpp
enum LID_TYPE { AVIA=1, VELO16=2, OUST64=3, MARSIM=4 };
```

Each type has its own handler in `preprocess.cpp` (`avia_handler`, `velodyne_handler`, `oust64_handler`, `sim_handler`). Feature extraction (plane/edge classification) is optional (`feature_extract_enable`).

### Radar-Livox fusion module

`src/radar_livox_fusion_node.cpp` is a separate ROS node (launched via `fusion_framework.launch` or `mapping_lidar_radar_fusion.launch`). It fuses Livox (`/livox/lidar`) and mmWave radar (`/radar_enhanced_pcl`) point clouds with:

- **Smoke-adaptive mode**: detects smoke conditions by scoring Livox frame quality (valid point ratio, far-range low-reflectivity, isolated voxels). In smoke mode, radar power threshold is lowered and point count limit is raised.
- **Time synchronization**: configurable delay (`livox_delay`) and max drift (`max_sync_dt`) to align async sensors.
- Configuration in `config/fusion_framework.yaml`.

Design specs for this module are in `docs/superpowers/specs/`.

## Configuration

All sensor parameters live in `config/*.yaml`. Common fields across all configs:

```yaml
lid_topic / imu_topic   # ROS topic names
lidar_type              # 1=Livox, 2=Velodyne, 3=Ouster, 4=Marsim
blind                   # min range to discard (meters)
acc_cov / gyr_cov       # IMU noise covariances
extrinsic_T / extrinsic_R  # LiDAR-to-IMU transform (3-vector + 9-element row-major matrix)
pcd_save_en             # save output map to PCD/scans.pcd
```
- **开发过程中禁止自动提交代码**:修改代码过程中禁止通过git提交代码，禁止git add，git commit, git push等操作，代码的git管理统一由人工完成。
- **未经允许不可修改代码**:每次修改代码前都要询问一次，未经允许不可直接修改代码