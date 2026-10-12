# Validation results

Re-validated on 12 October 2026 on the development laptop: Ubuntu 22.04, x86-64 (24 threads), Docker 28 with Compose v2.32, NVIDIA RTX 4060 Laptop GPU, ROS 2 Humble and Gazebo 11 inside `ari-counting:humble`. The image was rebuilt with `scripts/ari build --no-cache --pull` from a copy of the repository containing only what a fresh clone has, and the README was then followed from that copy. Source revisions are in `dependencies.repos`; each recorded episode includes the revisions, source checksum and scene snapshots. This replaces the first validation of 9 October, which missed the problems listed below.

## The robot moves

![ARI waving, reaching and pointing in Gazebo](media/demo_gazebo.gif)

| Evidence | File |
|---|---|
| Gazebo window during `examples/demo_motion.py` | [media/demo_gazebo.mp4](media/demo_gazebo.mp4) |
| The same motion from the front, overhead and robot cameras | [media/demo_cameras.mp4](media/demo_cameras.mp4) |
| Gazebo window on the desktop, 100 balls, GPU rendering | [media/gazebo_gui.png](media/gazebo_gui.png) |

The camera video is the unedited export of episode `e4263c6c61fd4aa788f4d0864b58f341` (GPU rendering, 45 s, 902 frames per camera). Its joint log agrees with the picture:

| Arm | Joint 1 | Joint 2 | Joint 3 | Joint 4 | Joint 5 |
|---|---|---|---|---|---|
| Right, travel | 56° | 85° | 112° | 80° | 1° |
| Left, travel | 69° | 3° | 0° | 6° | 0° |

All seven commands in the episode (four joint trajectories, three XYZ targets) were planned, accepted and completed; the worst final difference between commanded and measured joint position was 0.00002 rad. The head joints did not move.

The Gazebo-window video was captured from a Gazebo client on a virtual display inside the container (software rendering at 0.41× real time) and is played back at simulation speed. On the Wayland desktop the window opens and renders at real-time factor 1.00, but X11 screen grabs of its 3D view return a frozen frame, so motion in that particular window was not captured; watch it with `demo_motion.py --no-record`.

## Checks

| Check | Result |
|---|---|
| Image build from scratch, no cache | Passed in 3 min 46 s; package versions identical to the 9 October image |
| README followed from a fresh copy | Every command ran as written: quick start, arm examples, record/export, H.264 conversion, Python skeleton, examples, development commands |
| Unit tests | 22 passed on the host (Python 3.12) and in the container (Python 3.10) |
| Headless software-rendered simulation | Passed: ready 15 s after launch, 25 balls, three cameras, controllers, MoveIt scene |
| NVIDIA simulation with Gazebo GUI | Passed with 25 and with 100 balls; real-time factor 1.00 |
| Right- and left-arm joint commands | Passed; each of the five joints of each arm moved independently and returned |
| Keyboard jogging | Passed through a pseudo-terminal: select, jog, reverse, step change, non-ASCII key, quit |
| Reachable IK | Passed; position error 0.3–1.1 mm over three runs, below the 1 cm requirement |
| Invalid, unreachable and too-fast commands | Rejected without motion |
| Collision rejection | Temporary MoveIt obstacle rejected a direct trajectory without movement |
| Concurrency and cancellation | Second command rejected; cancellation stopped and held the arm |
| Long motions below real time | 8, 12 and 16 s trajectories completed at real-time factor 0.56 |
| Record, export, verify | Bags finalized; frame counts match timestamp sidecars; masks and causal alignment verified by `tests/verify_export.py` |
| Shutdown during recording | Bag finalized with Ctrl+C in a terminal, with `scripts/ari stop` and with `docker stop`; 2–3 s each |
| Custom configuration | Passed: 320×240 cameras at 10 FPS, two colours, smaller table |
| Launch argument validation | `ball_count:=101` and `arm:=both` refused |
| Interface document | Topic names, types and QoS match; `/clock` 200.0 Hz, `/joint_states` 50.0 Hz, cameras 20.0 Hz in simulation time |
| Loopback-only networking | Passed; discovery between containers and all camera streams work |

## Problems found and fixed on 12 October

| Problem | Effect | Fix |
|---|---|---|
| Motion timeouts counted wall-clock seconds | Headless (real-time factor 0.44) the README's IK example and every trajectory longer than about 10 s were cancelled with `ROS response timed out` | Deadlines follow the simulation clock, with a 30 s wall-clock guard for a stopped clock |
| Simulator bound to a LAN interface | When the laptop's Ethernet lost its address mid-run, every tool reported `experiment gateway not found` and the launch needed SIGKILL | `ROS_LOCALHOST_ONLY=1` is passed through and also keeps Gazebo's transport on loopback. It is opt-in; see the README |
| Nothing held the head | The head tilt joint sagged about 1° per minute | A `head_controller` holds it; measured drift is now zero |
| Ctrl+C in a terminal never reached the nodes | Nodes were SIGTERMed after 5 s and Gazebo SIGKILLed after 15 s; in headless mode X stopped before Gazebo | Launch runs with `--noninteractive`; Xvfb runs in its own session |
| `wait_ready(120)` gave up after 5 s | Tools started right after the launch could fail | It retries until its own deadline |
| `check` assumed 640×480 images | Failed with any custom camera size | Compares against the camera calibration |
| Keyboard jogging crashed on a non-ASCII key such as `£` | Tool exited | Such bytes are ignored |
| The integration test overwrote `docs/previews/*.png` | Tracked files changed on every run | Saving previews needs `--save-previews DIR` |
| No visible motion example; README examples move 6° | Hard to see that the robot moves | `examples/demo_motion.py`, a closer default Gazebo camera |
| README hard-coded a local path and an unverified IK target | Not usable elsewhere | Rewritten and re-run |

## Measured characteristics

- **Speed.** Real-time factor 1.00 with NVIDIA rendering (25 or 100 balls, GUI open); 0.44–0.56 with headless software rendering and one client; 0.41 with a software-rendered Gazebo client as well.
- **Reach.** Over 3,000 sampled postures, 2,923 collision-free, the right hand stayed within x −0.77…0.33 m, y −0.74…0.12 m, z 0.70…1.66 m in `world`. None was over the table, whose near edge is at x = 0.35 m; the closest approach to that edge was 0.21 m. ARI can point toward the balls but cannot touch them.
- **IK coverage.** 120 of 120 hand positions taken from random collision-free postures were solved from the arms-down posture, each to better than 0.1 mm.
- **IK repeatability.** From arms-down, three calls for one target returned three different valid postures, one with joint 3 at its limit. From a raised posture near the target, the same posture was returned on every run, in the demo and in the README example.
- **Recording size.** 2.4 GB for the 45 s demo, consistent with 55 MB per simulated second.

## Camera views

Captured from a 100-ball simulation on 9 October; the cameras are unchanged.

| Overhead | Front | Robot-mounted |
|---|---|---|
| ![Overhead view](previews/overhead.png) | ![Front view](previews/front.png) | ![Robot view](previews/robot.png) |

## Runtime limitations

- This validates one laptop and the bundled configuration plus one custom configuration. Other GPUs, machines without `docker` group access and other table or camera placements have not been exercised.
- By default the simulator still depends on the host's network interface and shares domain 42 with the local network. `ROS_LOCALHOST_ONLY=1` removes both, but is not the default.
- Only the arms are commanded. The base is anchored and the head is held still; the `robot` camera is fixed to the base, not to the head.
- Software rendering runs below real time. Sensor timestamps and exported alignment use simulation time; live observations expose rendering latency.
- The installed Humble MoveIt binary reports a segmentation fault in its trajectory-execution-manager destructor on every shutdown, after shutdown was requested. Gazebo and the gateway exit normally and the recorder flushes. The repository does not patch upstream MoveIt binaries.
- Position-only IK leaves orientation, elbow and wrist free; see IK repeatability above.
- Proprioception comes from Gazebo's exported interfaces. Missing effort/velocity fields remain missing; these records do not establish physical-robot dynamics.
- Force-killing the container or losing power cannot guarantee bag finalization. The exporter refuses episodes without a finalized bag; original data can be inspected/recovered separately with ROS bag tools.

Recordings are ignored by Git. Reproduce the motion checks against a running simulator:

```bash
scripts/ari run ros2 run ari_counting_tools check --motion --record
scripts/ari run python3 /workspace/tests/integration_controls.py
scripts/ari run python3 /workspace/tests/verify_export.py /data/episodes/EPISODE_ID/export
```
