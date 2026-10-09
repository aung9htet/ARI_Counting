# Interfaces and data semantics

## ROS graph

| Interface | Type | Purpose |
|---|---|---|
| `/experiment/cameras/{overhead,front,robot}/image_raw` | `sensor_msgs/msg/Image` | RGB camera stream, reliable/volatile QoS |
| `/experiment/cameras/{overhead,front,robot}/camera_info` | `sensor_msgs/msg/CameraInfo` | Intrinsics and optical frame |
| `/joint_states` | `sensor_msgs/msg/JointState` | All measured robot joint interfaces, nominally 50 Hz |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Robot and camera transforms |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation clock, 200 Hz |
| `/arm_{left,right}_controller/follow_joint_trajectory` | `control_msgs/action/FollowJointTrajectory` | Controller execution |
| `/experiment/commands` | `std_msgs/msg/String` | JSON tool requests |
| `/experiment/results` | `std_msgs/msg/String` | JSON tool replies |
| `/experiment/action_events` | `std_msgs/msg/String` | Timestamped action lifecycle log |
| `/experiment/scene` | `std_msgs/msg/String` | Scene metadata/ground truth, transient-local |
| `/experiment/scene_ready` | `std_msgs/msg/Bool` | MoveIt collision scene applied, transient-local |

`world` is the fixed experiment frame. `base_link` is the robot frame; external camera mounts are children of `world`. Each `<name>_camera_optical_frame` follows ROS optical axes (+Z forward, +X right, +Y down). The robot camera is mounted on `base_link` at the configured pose. Arm tips are `arm_left_hand_link` and `arm_right_hand_link`.

## Tool request protocol

Use `ExperimentClient` instead of constructing JSON manually. The wire protocol uses standard ROS messages to avoid a custom interface build dependency. Each request has `id` (unique nonempty string), `operation`, and optionally `arm`. Replies match `id` and include `success`, `result`, and `error`.

| Operation | Fields | Result |
|---|---|---|
| `status` | none | Readiness, stream freshness, recording path, limits and active-command state |
| `joints` | `positions` (five radians), `duration` (seconds), **or** `waypoints` (positions/absolute time list) | Measured final positions |
| `xyz` | `position` (three metres), `frame` | Measured final joint positions |
| `cancel` | none | Cancellation requested; controller stops and holds |
| `start_recording` | `instruction` (optional string) | Persistent episode path |
| `stop_recording` | none | Episode path and bag-finalization status |

Commands outside limits, too fast, in collision, unreachable, stale or concurrent fail explicitly. Motion planning and execution run outside the ROS callback thread so cancellation and subscriptions remain responsive. Reusing a command ID is rejected. A successful cancellation reply acknowledges the request; the original command subsequently reports its termination. If a client response timeout occurs, request cancellation before issuing another motion.

Action events contain `schema_version`, `stamp_ns`, `command_id`, `mode`, `arm`, and `event`; requested events additionally carry original tool arguments, planned events carry the full joint trajectory, and terminal events carry measured positions or an error. Requested actions and measured proprioception are separate sources.

## Observation and exported arrays

`get_observation()` returns the observation time, ROS JointState, three RGB arrays/calibrations, source timestamps and freshness masks. It chooses the newest buffered sample at or before the observation capture time. The live timestamp is the most recent common camera capture time, bounded by the simulation clock; `query_stamp_ns` and `latency_ns` expose delivery lag. Per-sample freshness still uses the 100 ms limit at that capture time. Gateway readiness requires cameras within 500 ms of the physics clock and joint states within 100 ms. Missing images/states are `None`. Ground truth is intentionally absent.

`observations.npz` uses one row per aligned time. `joint_names` gives the column order for joint matrices. Position/velocity/effort validity masks distinguish missing interfaces and stale samples. Each camera frame index maps directly to its MP4 and `*_frames.csv`. `action_event_index` maps to the corresponding JSONL line; it is `−1` before any event. All timestamp arrays are signed 64-bit nanoseconds; missing source timestamps are `−1`.

The exporter rejects backward source timestamps. Restart simulation and begin a new episode rather than mixing reset clocks. Export refuses an existing destination to avoid replacing a previous dataset. Original ROS messages and their headers remain in the bag for alternate processing.
