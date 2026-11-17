# semiH — Semi-Humanoid Mobile Manipulator

A ROS 2 (Humble) stack for a semi-humanoid mobile manipulator — Gazebo simulation,
SLAM/Nav2 autonomous navigation, YOLO-based perception with analytical arm IK, and
synchronized multimodal data collection — plus a Streamlit web dashboard
(`semiH_web/`) that runs and monitors the whole stack without a terminal.

## Repository Layout

```
src/
├── semih_description/      URDF/xacro robot model, Gazebo world, bringup/SLAM/Nav2/explore launch files
├── semih_perception/       YOLO object detection, 3D target localization, analytical arm IK
├── semih_data_collection/  Synchronized HDF5 data collection (camera, lidar, joint states, odometry)
└── m-explore-ros2/         External dependency (frontier exploration) — not vendored, see Setup

semiH_web/                  Streamlit dashboard that launches/stops/monitors the ROS 2 stack above
```

## Robot Stack (`src/`)

### `semih_description` — simulation & bringup
A single `bringup.launch.py` entry point replaces running Gazebo, SLAM, and Nav2 in
separate terminals, via one cumulative `mode` argument:

```bash
ros2 launch semih_description bringup.launch.py mode:=sim      # Gazebo only (default)
ros2 launch semih_description bringup.launch.py mode:=slam     # + slam_toolbox
ros2 launch semih_description bringup.launch.py mode:=nav      # + slam_toolbox + Nav2
ros2 launch semih_description bringup.launch.py mode:=explore  # + slam_toolbox + Nav2 + explore_lite (autonomous frontier exploration)
```

`mode:=explore` drives and picks its own goals with no human-provided target, and
stops once no reachable unexplored frontier remains. One specific bug found and
worked around during development: the robot spawns at exactly `(0,0)`, which sits on
the edge of the initial SLAM-derived costmap, so `explore_lite`'s first frontier
search fails immediately and does not retry. The launch file compensates with a short
automatic forward nudge ~20s after startup to move the robot off that exact boundary
coordinate before exploration starts (a pure rotation doesn't fix it — it has to be
linear motion).

### `semih_perception` — perception & arm control
- `yolo_detection_node` — YOLOv8 (Ultralytics) object detection
- `target_3d_node` — depth-camera 3D coordinate extraction for a detected target
- `arm_reach_node` — analytical inverse kinematics to reach the localized target

### `semih_data_collection` — dataset recording
- `hdf5_collector_node` — synchronizes camera, lidar, joint-state, and odometry
  topics (via `message_filters`) into HDF5 files for imitation-learning-style datasets.
  `hdf5_to_lerobot.py` (repo root) converts the recorded HDF5 output into LeRobot
  dataset format.

## Web Dashboard (`semiH_web/`)

A single-page Streamlit app that wraps the `ros2 launch`/`ros2 run` processes above
so the stack can be started, stopped, and monitored from a browser instead of 4-5
terminals. `semiH_web` only ever reads from the robot stack above — it never modifies
files under `src/`.

- **`core/process_manager.py`** — job/lock engine. Persists run state to
  `state/locks/*.json` on disk (survives page reloads / server restarts, not just
  Streamlit session state); uses atomic `O_CREAT | O_EXCL` file creation to prevent
  race conditions from duplicate start requests; runs every job as the leader of a
  new process group (`start_new_session=True`) and tears it down with `os.killpg`
  (SIGINT → SIGKILL on timeout) so grandchild processes spawned through shell
  wrappers (e.g. `ign gazebo`) don't end up orphaned; treats a lock as stale based on
  actual PID liveness (`psutil`/`os.waitpid`), not a timeout, to avoid misjudging
  zombie processes; and flags resource conflicts via a per-stage `STAGE_RESOURCES`
  table.
- **`core/launch_runner.py`** — wraps `bringup.launch.py`'s four modes with web-layer
  enum validation (the underlying launch file silently falls back to `sim` on an
  invalid mode value), adds a standalone RViz2 launcher (there's no UI for setting
  Nav2 goals otherwise) and an `rqt_image_view` button for the YOLO-annotated image
  topic, and injects software-rendering environment variables for WSLg setups without
  GPU acceleration.
- **`core/node_runner.py`** — starts/stops the four `ros2 run` nodes that have no
  launch file of their own. Lock policy differs per node based on observed behavior:
  `hdf5_collector` gets a hard single-instance lock after a confirmed data-loss bug
  from filename collisions on concurrent runs; the other three allow multiple
  instances under distinct keys since concurrent runs were verified safe.
- **`monitoring/topic_watcher.py`** — merges `launch_runner`/`node_runner` state into
  one dict for the UI, distinguishing process liveness from actual topic publication
  (the latter is a scoped TODO).
- **`pages/main.py` / `lib/ui_components.py`** — the dashboard itself: an environment
  (bringup) card, a perception pipeline card (YOLO → 3D localization → arm reach),
  and a data-collection card, with state-aware call-to-action prompts and a
  first-run guide tab.

## Setup

1. **ROS 2 workspace** (requires ROS 2 Humble):
   ```bash
   cd src && git clone https://github.com/robo-friends/m-explore-ros2.git  # frontier exploration dependency, not vendored
   cd .. && colcon build
   ```
2. **YOLO weights**: place `yolov8n.pt` (Ultralytics YOLOv8n) at the workspace root —
   referenced by `semiH_web/config/settings.yaml` → `yolo_model_path`.
3. **Web dashboard**:
   ```bash
   cd semiH_web && pip install -r requirements.txt
   streamlit run pages/main.py
   ```
   `semiH_web/config/settings.yaml` holds the workspace path and other machine-level
   settings — update it if the workspace lives somewhere other than the path baked in
   during development.

## Tech Stack

- **Robotics**: ROS 2 Humble, Gazebo, slam_toolbox, Nav2, explore_lite, RViz2
- **Perception**: YOLOv8 (Ultralytics), depth-camera 3D localization, analytical IK
- **Data**: HDF5 (h5py) synchronized multimodal recording, LeRobot format conversion
- **Dashboard**: Python, Streamlit, PyYAML, psutil, Pillow
- **Runtime**: WSL2 (Ubuntu 22.04), `subprocess`-based process-group management
