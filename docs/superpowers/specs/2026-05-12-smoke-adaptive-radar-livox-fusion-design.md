# Smoke-Adaptive Radar-Livox Fusion Design

## Context

The current radar-Livox fusion node publishes a FAST-LIO-compatible Velodyne-like `PointCloud2` with `x`, `y`, `z`, `intensity`, `time`, and `ring`. Livox points keep their per-point relative time, and millimeter-wave radar points are frame-synchronized to the Livox scan.

The target scene is a fire environment with intermittent dense smoke. This is not a constant smoke-only scene. In normal areas, Livox should remain the primary high-precision geometric source. In dense smoke, far Livox returns can become noisy or sparse, while millimeter-wave radar remains more stable through smoke but has coarser geometric accuracy.

The next improvement should therefore be adaptive. It should avoid treating the entire run as smoke, while still making the radar more useful when Livox quality degrades.

## Goals

- Improve robustness in smoke-heavy fire scenes without reducing normal-environment Livox accuracy.
- Detect Livox quality degradation before data enters FAST-LIO.
- Switch fusion behavior between normal and smoke modes with hysteresis.
- Keep FAST-LIO core unchanged in the first implementation.
- Preserve point-source separation for future FAST-LIO residual weighting.
- Keep the system explainable through logs and diagnostics.

## Non-Goals

- Do not add radar Doppler velocity residuals to FAST-LIO in this stage.
- Do not modify FAST-LIO's EKF, undistortion, or map update logic.
- Do not train or depend on a learned smoke detector.
- Do not assume every fire-scene frame contains dense smoke.

## Chosen Approach

Add a smoke-adaptive frontend filter inside `radar_livox_fusion_node`.

The fusion node computes a lightweight Livox quality score for each Livox frame. The score estimates whether the current frame looks like a smoke-degraded LiDAR frame. The node then chooses one of three environment modes:

```text
normal
smoke_suspected
smoke
```

The environment mode controls how aggressively the frontend filters far Livox points and how many radar points it keeps.

This first stage remains a frontend adaptation layer. FAST-LIO still receives one fused timed `PointCloud2` topic and still uses its existing Velodyne-style parser.

## Quality Signals

The first implementation should use explainable statistics from the raw Livox frame and the matched radar frame.

### Livox Valid Point Count

Track the number of Livox points that pass the existing line, tag, point-filter, and range checks.

The node should maintain a slow-moving baseline during normal operation:

```text
valid_points_ratio = current_valid_livox_points / baseline_valid_livox_points
```

Low valid point ratio suggests that Livox returns are degraded.

### Far Low-Reflectivity Ratio

For Livox points beyond a configured far range, count points whose reflectivity is below a configured threshold.

```text
far_low_reflectivity_ratio =
  far_low_reflectivity_livox_points / max(1, far_livox_points)
```

A high ratio suggests weak or smoke-affected far returns.

### Far Isolated Voxel Ratio

For Livox points beyond the far range, place points into coarse voxels. A voxel is isolated when it contains no more than a configured number of points.

```text
far_isolated_voxel_ratio =
  isolated_far_voxels / max(1, total_far_voxels)
```

Smoke noise often appears as sparse, unstable points rather than coherent surfaces.

### Radar Strong Point Count

For the matched radar frame, count points that pass range checks and have `Power` above the current mode threshold.

Radar strong points are not proof of smoke, but they help decide whether radar can safely contribute more points when Livox quality drops.

## Smoke Score

Compute a normalized smoke score:

```text
smoke_score =
  w_valid_count * low_valid_point_score
+ w_reflectivity * far_low_reflectivity_score
+ w_isolation * far_isolated_voxel_score
```

Each component should be clamped to `[0, 1]`. The weights should be configurable and sum to 1 in the recommended config.

The first implementation should keep the formula simple and log every component. It should not hide the decision behind a black-box model.

## Mode Hysteresis

Use separate enter and exit thresholds plus frame counters:

```text
if smoke_score >= enter_score for enter_frames:
  mode = smoke

if smoke_score <= exit_score for exit_frames:
  mode = normal

otherwise:
  keep current stable mode or use smoke_suspected while counters accumulate
```

This avoids rapid mode switching near threshold boundaries.

Recommended first values:

```yaml
smoke_adaptive:
  enabled: true
  enter_frames: 5
  exit_frames: 15
  enter_score: 0.65
  exit_score: 0.35
```

## Fusion Rules By Mode

### Normal Mode

Normal mode should preserve Livox accuracy.

- Keep Livox filtering close to the current behavior.
- Keep radar as a limited supplemental source.
- Use a stricter radar `Power` threshold.
- Use a lower radar point cap.

### Smoke Suspected Mode

Smoke suspected mode should be a transition state.

- Mildly reduce far low-reflectivity Livox points.
- Relax radar filtering slightly.
- Keep diagnostics visible to help tune thresholds.

### Smoke Mode

Smoke mode should protect FAST-LIO from noisy far Livox geometry while allowing radar to contribute more through-smoke structure.

- Strongly filter far Livox points with low reflectivity.
- Optionally reduce isolated far Livox voxels.
- Use a lower radar `Power` threshold.
- Use a higher radar point cap.

The first implementation should avoid deleting near Livox points aggressively. Near Livox geometry is usually still more accurate than radar and should remain the main constraint.

## Source Separation

Radar points should use a ring value outside the Livox line range.

Recommended:

```yaml
radar:
  ring: 31
```

With Livox lines `0-5`, this makes source separation available in the fused cloud:

```text
ring 0-5  -> Livox
ring 31   -> radar
```

This does not change FAST-LIO in the first stage. It prepares the data path for a later stage where FAST-LIO can apply different residual weights by point source.

## Configuration

Recommended first configuration shape:

```yaml
smoke_adaptive:
  enabled: true
  enter_frames: 5
  exit_frames: 15
  enter_score: 0.65
  exit_score: 0.35

  range:
    near: 8.0
    far: 20.0

  score_weights:
    valid_count: 0.35
    far_low_reflectivity: 0.35
    far_isolation: 0.30

  livox:
    far_low_reflectivity: 8.0
    min_valid_points_ratio: 0.5
    isolated_voxel_leaf: 0.5
    isolated_voxel_max_points: 2

  radar:
    min_power_normal: 7.0
    min_power_smoke: 5.5
    max_points_normal: 300
    max_points_smoke: 1000
```

These values are starting points. They should be tuned with rosbag playback from normal, light-smoke, and dense-smoke segments.

## Diagnostics

The node should publish or log the following at a throttled rate:

```text
mode
smoke_score
low_valid_point_score
far_low_reflectivity_score
far_isolated_voxel_score
valid_livox_points
baseline_valid_livox_points
far_livox_points
far_low_reflectivity_ratio
far_isolated_voxel_ratio
radar_points_before
radar_points_after
livox_points_before
livox_points_after
```

The first implementation can use `ROS_INFO_THROTTLE`. A later implementation can publish a small diagnostic topic for plotting.

## Data Flow

1. Build Livox candidate points and collect quality statistics.
2. Match the radar frame with the existing soft-sync queue.
3. Build radar candidate points and collect `Power`/range statistics.
4. Update the smoke score and mode state.
5. Apply mode-specific Livox and radar filtering.
6. Transform radar points into the Livox frame.
7. Merge, time-sort, and publish the timed fused `PointCloud2`.

## Testing

Manual tests:

- Playback normal-environment data and confirm the node remains in `normal` mode.
- Playback dense-smoke data and confirm the node enters `smoke` mode after the configured hysteresis.
- Confirm normal mode preserves most Livox points.
- Confirm smoke mode reduces far low-reflectivity Livox points.
- Confirm smoke mode keeps more radar points than normal mode.
- Confirm `/fusion/points` still contains `x`, `y`, `z`, `intensity`, `time`, and `ring`.
- Confirm FAST-LIO still runs through the existing Velodyne-style PointCloud2 path.

Tuning checks:

- Plot `smoke_score` over normal, light-smoke, and dense-smoke bag segments.
- Compare mapping stability with adaptive filtering disabled and enabled.
- Verify that mode switching does not oscillate during boundary cases.

## Future Work

- Feed point source and smoke quality into FAST-LIO residual weighting.
- Preserve radar `Doppler`, `Power`, and source metadata in a richer point type or companion diagnostic topic.
- Add a dedicated diagnostic message once the useful metrics stabilize.
- Evaluate Doppler velocity residuals only after the frontend smoke adaptation is validated.
