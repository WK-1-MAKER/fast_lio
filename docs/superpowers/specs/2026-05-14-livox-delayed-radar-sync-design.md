# Livox-Delayed Radar Sync Design

## Context

The radar-Livox fusion node currently uses Livox as the primary clock. Each Livox callback immediately searches the radar queue for the closest radar frame by `header.stamp`.

Recent timestamp diagnostics show that the two header time streams are broadly aligned in their overlapping time range:

```text
nearest-frame overlap dt = radar_stamp - livox_stamp
mean=-0.001344s median=+0.002840s std=0.024375s
min=-0.055401s max=+0.046872s
```

Livox is close to 10 Hz, while radar is about 12 Hz:

```text
livox period mean=0.099996s
radar period mean=0.083366s
```

There is no fixed time offset and no large header gap. The practical issue is callback timing: when a Livox message is processed immediately, the radar frame with the closest timestamp may not have arrived in the node yet. The node may then match an older radar frame or publish Livox-only even though a better radar frame arrives shortly afterward.

## Goals

- Improve online radar matching by waiting briefly before processing each Livox frame.
- Preserve Livox-primary output behavior.
- Continue publishing Livox-only frames when no radar frame matches.
- Avoid blocking ROS subscriber callbacks.
- Keep the existing timed `PointCloud2` output and smoke-adaptive filtering behavior.
- Keep FAST-LIO unchanged.

## Non-Goals

- Do not use `message_filters::ApproximateTime` as the primary implementation.
- Do not require radar for every Livox frame.
- Do not introduce a fixed radar timestamp offset.
- Do not change point field layout, source rings, smoke scoring, or FAST-LIO configuration.

## Chosen Approach

Add a Livox delay queue and a ROS timer.

The Livox subscriber callback only stores incoming Livox messages in a bounded queue. A timer periodically checks the oldest queued Livox message. If that message has waited at least `sync/livox_delay`, the node removes it from the queue and runs the existing fusion pipeline for that Livox frame.

Radar messages continue to be stored in the existing radar queue. By the time a Livox frame is processed, radar frames that arrived slightly later in wall-clock/callback order should already be available for timestamp matching.

## Parameters

Recommended first configuration:

```yaml
sync:
  max_sync_dt: 0.06
  livox_delay: 0.08
  livox_queue_size: 20
  radar_queue_size: 50
  stale_time: 0.20
  process_timer_period: 0.005
```

Parameter meanings:

- `max_sync_dt`: maximum accepted absolute header timestamp difference between matched Livox and radar frames.
- `livox_delay`: minimum wall-clock time a Livox message waits before processing.
- `livox_queue_size`: maximum number of Livox frames buffered.
- `radar_queue_size`: maximum number of radar frames buffered.
- `stale_time`: radar frames older than the current Livox timestamp by more than this value are dropped.
- `process_timer_period`: timer period for checking delayed Livox frames.

The first values are based on the measured overlap nearest-frame range of roughly `[-55 ms, +47 ms]`. A `60 ms` sync threshold should cover most valid nearest radar matches. An `80 ms` processing delay should allow slightly later-arriving radar messages to enter the queue.

## Data Flow

1. `mmwaveCallback(msg)`
   - Keep the current behavior:
     - detect radar timestamp rollback,
     - push radar messages into `radar_queue_`,
     - enforce `radar_queue_size`.

2. `livoxCallback(msg)`
   - Lock the shared queue mutex.
   - Detect Livox timestamp rollback and clear the Livox queue if needed.
   - Push a wrapper containing:
     - `livox_ros_driver::CustomMsg::ConstPtr msg`,
     - `ros::WallTime received_time`.
   - Enforce `livox_queue_size`.
   - Do not build or publish point clouds inside the subscriber callback.

3. `processTimerCallback(event)`
   - Repeatedly inspect the front of `livox_queue_`.
   - If `now - received_time < livox_delay`, stop.
   - Otherwise pop the Livox frame.
   - Run the existing fusion processing pipeline on that Livox frame:
     - publish raw Livox debug cloud,
     - build Livox candidates,
     - select best radar frame using header timestamps,
     - run smoke-adaptive filtering,
     - merge, time-sort, and publish `/fusion/points`.

## Matching Rules

The radar matching rule remains header-time based:

```text
best radar = argmin abs(radar_stamp - livox_stamp)
```

If the best radar frame has:

```text
abs(radar_stamp - livox_stamp) <= max_sync_dt
```

the radar frame is consumed and fused. Otherwise the Livox frame is published without radar.

Each radar frame is still consumed at most once.

## Callback and Locking Rules

The node should avoid heavy point cloud work while holding the queue mutex.

Recommended structure:

- Subscriber callbacks only push messages and enforce queue size under lock.
- Timer callback pops one ready Livox message under lock, then releases the lock before building point clouds.
- `takeBestMmwave()` can keep its own short lock while selecting/erasing from `radar_queue_`.

This keeps callback latency low and avoids blocking radar reception while processing Livox clouds.

## Diagnostics

Add throttled logs/counters for:

```text
livox_delay
livox_queue_size
dropped_livox_overflow_count
processed_livox_count
matched_count
livox_only_count
best_dt
```

The existing radar match logs should remain useful. A successful implementation should reduce cases where the node reports best radar matches around `150 ms` when the offline overlap nearest-frame statistics show much smaller deltas.

## Failure Behavior

- If radar is absent, each Livox frame is delayed by `livox_delay` and then published Livox-only.
- If radar arrives too late to satisfy `max_sync_dt`, the Livox frame is still published Livox-only.
- If Livox input is faster than processing, old Livox frames beyond `livox_queue_size` are dropped with a warning.
- If Livox timestamps roll back during bag replay, the Livox queue is cleared.

## Testing

Build test:

```bash
cmake --build build --target radar_livox_fusion_node -- -j2
```

Runtime checks:

- Run `check_time_offset.py` to confirm overlap nearest-frame dt remains near zero.
- Run the fusion node with rosbag playback.
- Confirm radar match logs usually show `dt <= 0.06`.
- Confirm `/fusion/points` still publishes when radar is stopped.
- Confirm `/fusion/points` fields remain `x`, `y`, `z`, `intensity`, `time`, `ring`.
- Confirm smoke-adaptive diagnostic logs still appear after delayed processing.

Tuning checks:

- Try `livox_delay` values `0.05`, `0.08`, and `0.10`.
- Try `max_sync_dt` values `0.05` and `0.06`.
- Pick the smallest delay and sync window that give stable radar match rates.

## Future Work

- Add a small diagnostic topic for match latency and queue depth.
- Make the delay adaptive based on observed radar arrival latency.
- Consider per-bag offline analysis to recommend `livox_delay` and `max_sync_dt`.
