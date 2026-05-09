# Radar-Livox Soft Sync Design

## Context

The current `radar_livox_fusion_node` fuses Livox `CustomMsg` points with the latest received millimeter-wave radar `sensor_msgs/PointCloud`. This is simple, but it has two timing problems:

- The latest radar frame is not guaranteed to be the closest frame to the current Livox scan.
- When filtering is disabled, the current code bypasses the time-difference check and may fuse stale radar points.

The fused output is intended to keep the Livox stream continuous for FAST-LIO. Radar points are supplemental and should not control the main output rate.

## Goals

- Use Livox as the primary clock.
- Match at most one radar frame to each Livox frame.
- Consume each radar frame at most once.
- Keep publishing Livox-only point clouds when no radar frame matches.
- Use a fixed first-version sync threshold of 30 ms.
- Keep the output frame in the Livox coordinate frame after applying the radar-to-Livox extrinsic transform.

## Non-Goals

- This design does not solve Livox `CustomMsg` to FAST-LIO time-field preservation.
- This design does not add radar velocity, RCS, or sensor-type fields to the output cloud.
- This design does not use `message_filters::ApproximateTime`, because unmatched Livox frames must still be published.

## Proposed Approach

Use a Livox-primary soft synchronization queue.

The radar callback stores incoming radar frames in a bounded `std::deque<sensor_msgs::PointCloud::ConstPtr>`. The Livox callback searches that queue for the radar frame with the smallest absolute timestamp difference from the Livox header timestamp. If the best match is within `max_sync_dt`, the node removes that radar frame from the queue and fuses it with the Livox cloud. If no match is found, the node publishes a Livox-only cloud.

This avoids duplicate radar use while preserving Livox output continuity.

## Data Flow

1. `mmwaveCallback(msg)`
   - Lock the radar queue mutex.
   - If the queue already contains newer data and `msg` is older than the last radar timestamp, warn, clear the queue, and treat `msg` as the new radar time base.
   - Push `msg` into `radar_queue_`.
   - Enforce `radar_queue_size`.

2. `livoxCallback(msg)`
   - Build and publish the Livox raw debug cloud.
   - Build the filtered Livox cloud.
   - Lock the radar queue mutex.
   - Find the radar frame with minimum `abs(radar_stamp - livox_stamp)`.
   - Remove radar frames older than `livox_stamp - stale_time`.
   - If the best frame has `best_dt <= max_sync_dt`, copy it out and erase it from the queue.
   - Unlock.
   - If a radar frame was selected, convert it to PCL, filter it, transform it into the Livox frame, and append it to the Livox cloud.
   - Apply optional voxel filtering.
   - Publish the fused cloud using the Livox timestamp.

## Parameters

The existing `common/max_sync_dt` should be kept for compatibility or moved to a clearer `sync` namespace. The first implementation can support both, with `sync/max_sync_dt` preferred if present.

Recommended config:

```yaml
sync:
  max_sync_dt: 0.03
  radar_queue_size: 50
  stale_time: 0.20
```

Parameter behavior:

- `max_sync_dt`: maximum accepted absolute timestamp difference. Initial value: `0.03` seconds.
- `radar_queue_size`: maximum number of radar frames buffered. Initial value: `50`.
- `stale_time`: radar frames older than the current Livox timestamp by more than this value are removed. Initial value: `0.20` seconds.

## Matching Rules

- A radar frame can be used by only one Livox frame.
- A Livox frame is never dropped because of radar mismatch.
- If multiple radar frames have equal timestamp difference, use the earlier frame to avoid letting old data accumulate.
- If the radar queue is empty, publish Livox-only immediately.
- If the best radar frame is outside the 30 ms window, publish Livox-only and keep only radar frames that are not stale.

## Diagnostics

Use throttled ROS logs to avoid flooding the terminal:

- Matched radar frame and timestamp difference.
- No radar match found, publishing Livox-only.
- Dropped stale radar frame count.
- Radar queue overflow or time rollback.

Optional counters can be added as member variables:

- `matched_count_`
- `livox_only_count_`
- `dropped_stale_count_`
- `dropped_overflow_count_`

## Error Handling

- Invalid extrinsic sizes continue to fall back to identity, as the current node already does.
- Empty Livox cloud returns without publishing fused output, preserving current behavior.
- Empty radar cloud can be consumed as a matched frame but contributes no points.
- Radar timestamp rollback should not crash the node. The first version warns, clears the radar queue, and stores the rollback frame as the new radar time base.

## Testing

Manual tests:

- Start the fusion node with radar and Livox topics active.
- Confirm `/fusion/points` continues publishing when `/radar_enhanced_pcl` is stopped.
- Confirm matched radar frames show `dt <= 0.03` in throttled logs.
- Confirm radar frames are not reused across consecutive Livox frames.
- Confirm output header stamp matches the Livox frame stamp.
- Confirm output frame id matches the Livox frame id or configured Livox-frame output.

Code-level tests can be added later by extracting the queue matching function into a small pure helper, then testing:

- Empty queue returns no match.
- Closest frame within threshold is selected.
- Closest frame outside threshold is not selected.
- Selected frame is erased.
- Stale frames are removed.

## Open Follow-Up

The next design question is whether the final FAST-LIO input should remain `PointCloud2` or be converted back into a Livox-compatible `CustomMsg`-like stream to preserve point-level timing. This is intentionally outside the soft-sync change.
