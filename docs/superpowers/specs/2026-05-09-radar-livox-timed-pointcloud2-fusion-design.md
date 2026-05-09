# Radar-Livox Timed PointCloud2 Fusion Design

## Context

The soft-sync fusion node now matches millimeter-wave radar frames to Livox frames using Livox as the primary clock. The current fused output is still a simple `PointXYZI` `sensor_msgs/PointCloud2`.

That output is enough to visualize concatenated points, but it loses Livox point-level timing. FAST-LIO depends on per-point relative time for scan end-time estimation and IMU-based motion undistortion. In this FAST-LIO codebase:

- Livox `CustomMsg` uses `offset_time`, then stores it in `PointType.curvature` in milliseconds.
- Velodyne-style `PointCloud2` expects fields `x`, `y`, `z`, `intensity`, `time`, and `ring`.
- Ouster-style `PointCloud2` expects fields including `x`, `y`, `z`, `intensity`, `t`, and `ring`.
- The undistortion path sorts or uses points by `PointType.curvature`.

The current rosbag radar topic `/radar_enhanced_pcl` is `sensor_msgs/PointCloud` with channels `Doppler`, `Range`, `Power`, `Alpha`, and `Beta`. It has no per-point time channel, so all radar points can only be assigned a frame-level timestamp for this dataset.

## Goals

- Publish a FAST-LIO-compatible timed `PointCloud2` from the fusion node.
- Preserve Livox point-level time from `livox_ros_driver::CustomMsg::points[i].offset_time`.
- Keep the current radar soft-sync behavior: use at most one radar frame per Livox frame, and publish Livox-only when no radar match exists.
- Represent current radar rosbag points with one frame-level relative time.
- Leave a clean path for future radar drivers that may provide per-point time.
- Keep the fused cloud in the configured Livox output frame after applying radar-to-Livox extrinsics.

## Non-Goals

- Do not invent synthetic radar per-point time from point order.
- Do not solve radar Doppler/RCS semantic fusion in this change.
- Do not modify FAST-LIO's core undistortion or EKF logic.
- Do not introduce a custom ROS message type for the fused cloud.

## Chosen Approach

Publish a Velodyne-like `sensor_msgs::PointCloud2` with these fields:

```text
x
y
z
intensity
time
ring
```

FAST-LIO should consume this topic through its existing `VELO16` PointCloud2 preprocessing path. The name `VELO16` is only used to select the parser that understands `time` and `ring`; the fused data is still Livox-primary radar-Livox data.

Recommended FAST-LIO configuration:

```yaml
feature_extract_enable: false

common:
  lid_topic: "/fusion/points"

preprocess:
  lidar_type: 2
  timestamp_unit: 0
  scan_line: 6
```

`timestamp_unit: 0` means the published `time` field is in seconds. FAST-LIO's Velodyne handler converts seconds to milliseconds internally before storing the value in `curvature`.

## Point Time Rules

For Livox points:

```text
time = offset_time / 1e9
```

Livox `offset_time` is in nanoseconds relative to the Livox message timestamp. This produces a floating-point relative time in seconds.

For the current radar rosbag:

```text
radar_frame_time = radar_msg.header.stamp - livox_msg.header.stamp
radar_point.time = clamp(radar_frame_time, 0, livox_scan_duration)
```

Every radar point in the matched radar frame receives the same relative time because the message does not contain per-point timing.

For future radar messages with per-point time:

```text
radar_point.time = convert_to_seconds(radar_point_absolute_or_relative_time) - livox_msg.header.stamp
radar_point.time = clamp(radar_point.time, 0, livox_scan_duration)
```

The first implementation may keep this as a documented extension point. A later implementation can add configuration such as:

```yaml
radar_time:
  mode: "frame"      # frame, channel_relative, channel_absolute, auto
  channel: "time"
  unit: "sec"        # sec, ms, us, ns
```

## Clamp and Sort Rules

Radar relative time can be negative when the best radar frame is slightly earlier than the Livox header time. It can also be later than the Livox scan duration. FAST-LIO expects per-scan relative time, so radar time must be clamped into the Livox scan interval.

The Livox scan duration should be computed from the maximum retained Livox point time in the fused frame. If no valid Livox time exists, fall back to a configured scan duration.

After Livox and radar points are merged, sort the output by `time` ascending before publishing. This protects FAST-LIO's scan end-time logic, which uses the last point's `curvature` after preprocessing.

## Ring Rules

Livox points:

```text
ring = livox_point.line
```

Radar points:

```text
ring = configured_radar_ring
```

The default radar ring should be `0`. With FAST-LIO feature extraction disabled, radar `ring` mainly satisfies the Velodyne point layout and keeps the parser happy.

## Intensity Rules

Livox points:

```text
intensity = reflectivity
```

Radar points for the current rosbag:

```text
intensity = Power channel if available, otherwise 1.0
```

The existing code only looks for a lowercase `intensity` channel. The current radar bag uses `Power`, so the timed-output implementation should prefer `Power` for radar intensity and optionally keep `intensity` as a fallback.

## Filtering Rules

The fused FAST-LIO input should not use voxel filtering inside `radar_livox_fusion_node`.

Voxel filtering custom timed points can average or corrupt `time` and `ring` semantics. Range filtering is acceptable because it only removes points. Downsampling should be left to FAST-LIO's own preprocess and map filters unless a later implementation adds a time-aware custom downsampler.

Recommended fusion config:

```yaml
filter:
  is_filter: false
```

## Data Flow

1. `mmwaveCallback(msg)`
   - Store radar frames in the existing soft-sync queue.
   - Keep current no-reuse and stale-drop behavior.

2. `livoxCallback(msg)`
   - Build Livox timed points with `x`, `y`, `z`, `intensity`, `time`, and `ring`.
   - Publish optional raw Livox debug cloud if still useful.
   - Take the best matched radar frame from the soft-sync queue.
   - If a radar frame exists, convert radar points to the Livox frame and assign frame-level `time`.
   - Merge Livox and radar points.
   - Apply range filtering if enabled.
   - Sort merged points by `time`.
   - Publish `/fusion/points` with the Livox header stamp and configured Livox output frame.

## Error Handling

- If no radar frame matches, publish a Livox-only timed cloud.
- If the radar cloud is empty, consume the matched frame and publish Livox-only content.
- If Livox scan duration cannot be inferred, use a configured fallback duration.
- If a future radar time channel is configured but missing, warn and fall back to frame-level radar time.
- If a radar time channel exists but has a different length from `points`, warn and fall back to frame-level radar time.

## Testing

Manual tests:

- Confirm `/fusion/points` fields include `x`, `y`, `z`, `intensity`, `time`, and `ring`.
- Confirm Livox points preserve nondecreasing relative `time`.
- Confirm radar points receive a clamped frame-level relative `time`.
- Confirm fused points are sorted by `time`.
- Confirm FAST-LIO can subscribe to `/fusion/points` with `preprocess/lidar_type: 2`.
- Confirm `/fusion/points` continues when radar data is absent.

Build and smoke tests:

- Build `radar_livox_fusion_node`.
- Run the fusion node against the current rosbag and inspect one output message with a small Python subscriber.
- Run FAST-LIO with `/fusion/points` as `common/lid_topic` and verify it reaches the preprocessing callback without field conversion errors.

## Open Follow-Ups

- Add optional radar time-channel support when a real radar driver provides per-point timing.
- Decide whether radar Doppler should be preserved in a separate debug topic or a richer fused point type.
- Consider a separate visualization-only topic if users still want a plain `PointXYZI` cloud for RViz compatibility.
