# Sensing stack (RealSense + OptiTrack + ATI) in this workspace

This workspace is a ROS 2 *colcon* workspace. The important top-level folders are:

- `src/`: Source code for ROS 2 packages (plus a few vendored repos).
- `build/`: CMake build artifacts per package (generated).
- `install/`: The “overlay” you source so ROS can find the packages you built (generated).
- `log/`: Build logs (generated).

The sensing system is split into three driver packages (one per modality), one bringup package (launch), and one recording package (synchronization + logging).

## Packages and responsibilities

### OptiTrack / MoCap

- Package: `src/rros-mocap-optitrack/mocap_optitrack_client/`
- Node: `mocap_optitrack_client` (executable: `mocap_optitrack_client`)

What it does:

- Connects to Motive via NatNet.
- Publishes motion-capture frames on the topic configured by `pub_topic` (default in `natnetclient.yaml` is `mocap_Data`).
- If `record` is true, it starts/stops recording inside Motive, and uses `take_name` for the take name.
- Serves `get_mocap_data` (`mocap_optitrack_interfaces/srv/GetMotionCaptureData`) so you can “pull the latest” without running a second wrapper node.

### ATI NetFT force/torque

- Package: `src/rros-mocap-optitrack/ati_sensor_service/`
- Node: `ati_sensor_service` (executable: `ati_service`)

What it does:

- Streams from the ATI NetFT sensor over UDP.
- Publishes:
  - `force_torque` (`ati_sensor_interfaces/msg/ForceTorque`) for backwards compatibility.
  - `wrench` (`geometry_msgs/msg/WrenchStamped`) for standard ROS tooling and timestamped sync.
- Serves `get_force_torque` (`ati_sensor_interfaces/srv/GetForceTorque`) so you can “pull the latest” from the same node.

### RealSense (4 cameras)

- Vendor repo: `realsense-ros/` (contains `realsense2_camera`)
- Node per camera: `realsense2_camera_node`

What it does:

- Each camera runs in its own ROS namespace (for example, `camera_1`, `camera_2`, …).
- Each camera publishes RGB, depth, and camera info topics under `/<namespace>/camera/...`.

## Bringup (enable/disable sensors cleanly)

- Package: `src/service_launch/`
- Config: `src/service_launch/config/sensors.yaml`
- Launch: `src/service_launch/launch/record_take.launch.py`

The YAML file is the “single place” where you enable/disable sensors and cameras. For example, you can disable one camera by setting its `enabled: false`, and the rest of the system continues recording the remaining sensors.

## Synchronization + logging (“recorder”)

- Package: `src/rros-mocap-optitrack/data_collection/`
- Node: `data_collection` (executable: `multi_sensor_recorder`)
- Export tool: `data_collection` (executable: `export_take`)

What it does:

- Subscribes to:
  - MoCap: `mocap_Data` (`mocap_optitrack_interfaces/msg/MotionCaptureData`)
  - ATI: `wrench` (`geometry_msgs/msg/WrenchStamped`)
  - RealSense: per-camera RGB/depth topics under `/<camera_name>/camera/...`
- Chooses the *closest message in time* to the current “sample time” (within `max_sync_slop_ms`).
- Writes a take directory like:
  - `samples.csv` (one row per sample, includes timestamps + image paths + force values)
  - `cameras/<camera_name>/rgb/*.png`
  - `cameras/<camera_name>/depth/*.png`
  - `mocap/sample_*.yaml` (full MoCap message per sample)
- Provides services:
  - `set_recording` (`std_srvs/srv/SetBool`) to pause/resume continuous recording.
  - `capture_sample` (`std_srvs/srv/Trigger`) to force a single snapshot.

### Exporting into one flattened CSV

The recorder stores the full MoCap message per sample as YAML (because MoCap can contain variable-length arrays). When you want “everything in one CSV”, run:

`ros2 run data_collection export_take --take-dir ~/multi_iiwa_ws_data/takes/ft_001`

This produces `export.csv` in that take directory, with rigid body poses flattened into `rb_<id>_*` columns.

## Running a take

This launch file:

- Reads `src/service_launch/config/sensors.yaml`.
- Increments a counter file under `paths.take_root_dir` (for example, `~/multi_iiwa_ws_data/takes/take_number.txt`).
- Launches the enabled sensors and the recorder.

Command:

`ros2 launch service_launch record_take.launch.py`

## Notes on timestamps

- MoCap messages already have `header.stamp` set from NatNet’s mid-exposure timestamp (converted to system-time epoch nanoseconds).
- RealSense images and camera info use `header.stamp` from the RealSense ROS driver.
- The ATI node publishes `wrench` as `WrenchStamped`, which includes a `header.stamp`.

Because all three modalities carry timestamps, the recorder can align them by time and log the chosen stamps into `samples.csv`.
