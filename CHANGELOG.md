# Changelog

All notable changes to vxlros2 are documented in this file.
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.1.0/);
versioning follows [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.3.0] — 2026-05-06

Long-term-stability follow-up to v0.2.0 — second pass on the code-review
A–G items. End-to-end validated on UTM VM with diagnostics topic showing
the device + per-stream status at 1 Hz.

### Performance

- **Frame path: 2 copies → 1 copy.** `BackendFrame` now holds the SDK
  `vxl::FramePtr` as a keep-alive (`borrowed`) rather than memcpy'ing the
  pixel buffer in `publishLoop`. The single remaining unavoidable copy
  is in `buildImageMsg`, where bytes go from the SDK-owned buffer to the
  ROS `Image::data` vector that DDS owns. (The v4l2 mmap → SDK buffer
  copy that the SDK does itself is also unavoidable; that's where the
  buffer-recycle window lives.)
  - Visible effect: `ros2 topic hz /vxl_camera/depth/image_raw` now
    reports a stable 15 Hz on UTM where v0.2.0's `echo --once` reported
    "A message was lost" because publish was bandwidth-bound.
- `BackendFrame::data()` / `dataSize()` are the new accessor methods —
  callers must NOT touch `.borrowed` / `.owned` directly. Mock backend
  uses the `owned` vector path.

### Added

- **`~/diagnostics`** (`diagnostic_msgs/DiagnosticArray`) at 1 Hz.
  Publishes one `DiagnosticStatus` for the device (serial / firmware /
  product / output_mode / sync_mode / streaming state) and one per
  enabled stream (frame count, last-frame age, OK/WARN/STALE/ERROR
  level). Compatible with `rqt_robot_monitor`.
- **`publish_rgbd_composite`** ROS parameter (default `true`). Controls
  whether `~/rgbd` (composite RGB+Depth message) is advertised when
  both color and depth flow.
- `PointCloudGenerator` accepts both `16UC1` / `mono16` (uint16_t mm,
  multiplied by `depth_scale_`) and `32FC1` (float meters) depth
  encodings; previously hardcoded to uint16_t which would silently
  produce garbage for float-depth devices.

### Changed

- **Topology unification.** Per-stream `~/<stream>/image_raw` +
  `~/<stream>/camera_info` topics are ALWAYS advertised when the
  corresponding stream is enabled — independent of `output_mode`. The
  RGBD composite (`~/rgbd`) is additional, gated on the
  `publish_rgbd_composite` parameter. v0.2.0's behaviour where
  `output_mode=rgbd` would only publish the composite (and not
  `~/depth/image_raw`) is gone — downstream launch / rviz config no
  longer needs to branch on `output_mode`.
- Hardware-timestamp `frame_utils::toRosTimeFromHardware()` doc-block
  expanded to spell out single-device monotonicity, sub-frame
  precision, NOT-wall-clock semantics, and the multi-device offset
  requirement for sensor fusion.

### Removed

- `vxl_camera::BackendFrame::data` member (the owned `std::vector<uint8_t>`
  was the source of the redundant copy). Mock-backend / test code reads /
  writes via the new `owned` field; production reads via the unified
  `data()` accessor.
- `VxlCameraNode::StreamStats` / `frames_published` counters — replaced
  by the shared `StreamDiagCounters` (with `total_frames` +
  `last_publish_ns`) consumed by the diagnostics publisher.

### Internal / refactor

- `node_helpers.{hpp,cpp}` gained: `StreamDiagCounters`,
  `DiagnosticInputs`, `buildDiagnosticArray`. ~150 LOC, fully tested by
  the existing mock launch test (no regressions).

### Migration guide

For downstream subscribers:
- `~/color/image_raw`, `~/depth/image_raw`, etc. are now ALWAYS available
  when the corresponding stream is on (no more output_mode dependency).
  Existing subscribers to `~/rgbd` keep working.
- Set `publish_rgbd_composite:=false` if you only want the per-stream
  topics.

For `PointCloudGenerator` users:
- If you were feeding a non-`16UC1` depth Image, it would have produced
  garbage in v0.2.0; now `32FC1` is supported, anything else returns
  `nullptr`.

### Known issues

- vxlsdk v1.1.7 Release build crashes in `vxl_context_create()` with a
  stack-protector overflow (every binary linking the lib, not just
  vxl_camera). Workaround: build SDK with
  `cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_FLAGS='-O0 -fno-stack-protector'`.
  The SDK team needs to fix the underlying overflow (likely a fixed-
  size local array somewhere in the context init path). v1.1.6 was
  not affected.

## [0.2.0] — 2026-05-06

Bundled P0–P3 fix from a code review. End-to-end RGBD validated on UTM VM
(Humble + v4l2 backend). Includes one breaking API change — see Migration
below.

### Breaking changes

- **`image_transport` and `camera_info_manager` are no longer dependencies.**
  Both `VxlCameraNode` and `VxlCameraLifecycleNode` now publish raw
  `sensor_msgs/Image` and `sensor_msgs/CameraInfo` directly.
  - The lifecycle node already had to do this (image_transport doesn't
    support LifecycleNode in Humble); this brings the non-lifecycle node
    in line, so downstream subscribers see the same topology in both
    variants.
  - Enables `pub->publish(std::move(unique_ptr_msg))` everywhere → unlocks
    intra-process zero-copy for composable subscribers.
  - **Tradeoff**: `compressed`, `theora`, and other image_transport plugin
    topics are no longer auto-advertised. Run a republisher node
    downstream if you need them.
- **`SetInt32` / `GetInt32` services add a required `sensor` field**
  (`"color"` | `"depth"` | `"ir"`). The previous heuristic ("option_id ≥
  100 ⇒ depth") would silently misroute any future option whose id
  happens to fall in either range. Update clients accordingly:
  ```python
  req.sensor = "color"
  req.option_name = "1"   # VXL_OPTION_EXPOSURE
  req.value = 5000
  ```

### Added

- `node_helpers.{hpp,cpp}` — shared parameter declaration, dynamic sensor
  option declaration, BackendStreamConfig builder, the four service
  handlers, message builders (Image / RGBD / Metadata / Extrinsics), and
  the parameter-change side-effect plumbing. Adding a new ROS parameter
  or extending a service now requires one edit, not two.
- `inter_stream_start_delay_ms` cold parameter, default 1000 ms (matches
  the SDK `dst` reference program). Drop to 200 ms on physical Linux for
  snappier startup; 0 disables (only safe when verified single-shot).
- `frame_utils::toRosTimeFromHardware()` and `frame_utils::mmToMeters()`
  helpers (centralise unit conversion).

### Changed

- **Image / RGBD / Metadata header `stamp` now uses the SDK hardware
  timestamp** (microseconds since device epoch, converted via
  `toRosTimeFromHardware`). Falls back to `now()` if the SDK reports 0.
  Downstream TF / sensor fusion now sees the actual exposure time
  rather than the publish moment (saves tens of ms of jitter from
  publish_thread scheduling + USB latency).
- **`sdk_mutex_` scope narrowed**: frame `convert` / filter chain /
  `vxl_dip_align_depth_to_rgb` / `copyFrame` now run unlocked in
  publishLoop. Main-thread `setOption` no longer blocks behind 1080p
  MJPEG decode (~10–30 ms). `computeAlignedDepth` re-acquires the mutex
  only briefly for the calibration cache fill.
- **Lifecycle reopen uses exponential backoff** (1s → 2s → … → 10s
  capped, resets on successful open). The 1 Hz hotplug monitor no
  longer spams the SDK and log when a USB device stays disconnected.
- `~/hw_reset` service now runs the full `close → hwReset →
  sleep(500ms) → open` sequence inside the service. Bare hwReset
  previously left the device in a state where the next open failed
  (VXL615 close/reset must be paired with USB reset before next open).
  Callers that get `success = true` know the device is usable.
- `SdkCameraBackend` caches sensor handles via `sensorForLocked()` —
  each `setOption` / `getOption` no longer issues a `device_->getSensor()`
  round-trip. Cache cleared in `close()`.

### Removed

- SDK→`onStreamFrame` `fprintf(stderr, ...)` debug spam at 30 Hz.
  Replaced with a static `std::atomic<uint64_t>[3]` counter for future
  diagnostics use.
- `VxlCameraNode::declareDynamicOptions` member function (replaced by
  free function `declareDynamicSensorOptions(node, backend)` from
  `node_helpers`).

### Internal / refactor

- ~570 LOC of duplication between `vxl_camera_node` and
  `vxl_camera_lifecycle_node` eliminated by routing through
  `node_helpers`. Net diff: +429 / -789, −360 LOC overall.
- Both node files trimmed substantially:
  - `vxl_camera_node.cpp`: 790 → 480
  - `vxl_camera_lifecycle_node.cpp`: 954 → 690

### Migration guide

For callers of `~/set_option` / `~/get_option`:
```diff
  req = SetInt32.Request()
+ req.sensor = "color"          # or "depth", "ir"
  req.option_name = "1"
  req.value = 5000
```

For users of `image_transport` plugin topics
(`<topic>/compressed`, `<topic>/theora`, etc.): these topics are no
longer advertised by the driver. Either (a) use the raw `image_raw`
topic directly, or (b) run an `image_transport_republisher` downstream:
```bash
ros2 run image_transport republish raw \
  in:=/vxl_camera/color/image_raw \
  compressed out:=/vxl_camera/color/image_raw
```

`/vxl_camera/rgbd` (the composite RGBD message) is unchanged.

## [0.1.x] — pre-history

See `git log` for changes prior to 0.2.0; the changelog format starts
here.
