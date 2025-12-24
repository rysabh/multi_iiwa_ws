import os
from pathlib import Path
from typing import Any, Dict, List

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node


def _read_yaml(path: str) -> Dict[str, Any]:
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data if isinstance(data, dict) else {}


def _merge_dicts(base: Dict[str, Any], overlay: Dict[str, Any]) -> Dict[str, Any]:
    merged = dict(base)
    for key, value in overlay.items():
        if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
            merged[key] = _merge_dicts(merged[key], value)
        else:
            merged[key] = value
    return merged


def _expand_dot_keys(params: Dict[str, Any]) -> Dict[str, Any]:
    expanded: Dict[str, Any] = {}
    for key, value in params.items():
        if isinstance(value, dict):
            value = _expand_dot_keys(value)

        if "." not in key:
            if key in expanded and isinstance(expanded[key], dict) and isinstance(value, dict):
                expanded[key] = _merge_dicts(expanded[key], value)
            else:
                expanded[key] = value
            continue

        parts = key.split(".")
        cursor = expanded
        for part in parts[:-1]:
            if part not in cursor or not isinstance(cursor[part], dict):
                cursor[part] = {}
            cursor = cursor[part]
        leaf = parts[-1]
        if leaf in cursor and isinstance(cursor[leaf], dict) and isinstance(value, dict):
            cursor[leaf] = _merge_dicts(cursor[leaf], value)
        else:
            cursor[leaf] = value
    return expanded


def _next_take_id(take_root_dir: str) -> str:
    Path(take_root_dir).mkdir(parents=True, exist_ok=True)
    take_number_file = os.path.join(take_root_dir, "take_number.txt")

    if not os.path.exists(take_number_file):
        with open(take_number_file, "w", encoding="utf-8") as f:
            f.write("1\n")

    with open(take_number_file, "r", encoding="utf-8") as f:
        take_number = int(f.readline().strip())

    with open(take_number_file, "w", encoding="utf-8") as f:
        f.write(f"{take_number + 1}\n")

    return f"{take_number:03d}"

_TAKE_ENV_NAME = "SERVICE_LAUNCH_TAKE_NAME"
_TAKE_ENV_DIR = "SERVICE_LAUNCH_TAKE_DIR"


def _launch_setup(context, *args, **kwargs):
    log_level = "warn"

    service_launch_share = get_package_share_directory("service_launch")
    config_path = os.path.join(service_launch_share, "config", "sensors.yaml")
    cfg = _read_yaml(config_path)

    paths_cfg = cfg.get("paths", {})
    take_root_dir = os.path.expanduser(
        str(paths_cfg.get("take_root_dir", os.path.join(os.path.expanduser("~"), "multi_iiwa_ws_data", "takes")))
    )

    # Some launch setups may evaluate generate_launch_description more than once in the same process.
    # Use process env (not module globals) to ensure we only bump the take counter once per launch.
    take_name = os.environ.get(_TAKE_ENV_NAME, "").strip()
    take_dir = os.environ.get(_TAKE_ENV_DIR, "").strip()
    if not take_name or not take_dir:
        take_id = _next_take_id(take_root_dir)
        take_name = f"ft_{take_id}"
        take_dir = os.path.join(take_root_dir, take_name)
        Path(take_dir).mkdir(parents=True, exist_ok=True)
        os.environ[_TAKE_ENV_NAME] = take_name
        os.environ[_TAKE_ENV_DIR] = take_dir

    print(f"\n======================\nTake: {take_name}\nDir:  {take_dir}\n======================\n")
    actions = []

    mocap_cfg = cfg.get("mocap", {})
    if mocap_cfg.get("enabled", True):
        natnet_config_file = mocap_cfg.get("natnet_config_file", "natnetclient.yaml")
        natnet_config_path = os.path.join(
            get_package_share_directory("mocap_optitrack_client"), "config", natnet_config_file
        )

        natnet_client = Node(
            package="mocap_optitrack_client",
            executable="mocap_optitrack_client",
            name="natnet_client",
            parameters=[
                natnet_config_path,
                {
                    "record": bool(mocap_cfg.get("record_in_motive", True)),
                    "take_name": take_name,
                },
            ],
            arguments=["--ros-args", "--log-level", log_level],
        )
        actions.append(natnet_client)

    ati_cfg = cfg.get("ati", {})
    if ati_cfg.get("enabled", True):
        ati_sensor_node = Node(
            package="ati_sensor_service",
            executable="ati_service",
            name="ati_sensor",
            output="screen",
            parameters=[
                {"sensor_ip": str(ati_cfg.get("sensor_ip", "192.168.10.100"))},
                {"sample_rate": float(ati_cfg.get("sample_rate_hz", 240.0))},
                {"frame_id": str(ati_cfg.get("frame_id", "ati_sensor"))},
            ],
        )
        actions.append(ati_sensor_node)

    realsense_cfg = cfg.get("realsense", {})
    enabled_camera_names: List[str] = []
    if realsense_cfg.get("enabled", True):
        common_params = dict(realsense_cfg.get("common", {}))
        cameras: List[Dict[str, Any]] = list(realsense_cfg.get("cameras", []))

        for cam in cameras:
            if not cam.get("enabled", True):
                continue

            cam_namespace = str(cam.get("name", "camera"))
            enabled_camera_names.append(cam_namespace)
            serial_no = str(cam.get("serial_no", ""))
            if serial_no and not serial_no.startswith("_"):
                serial_no = f"_{serial_no}"

            params = dict(common_params)
            cam_params = cam.get("params", {})
            if isinstance(cam_params, dict):
                params.update(cam_params)
            params.update(
                {
                    "camera_name": "camera",
                    "serial_no": serial_no,
                }
            )
            params = _expand_dot_keys(params)

            realsense_node = Node(
                package="realsense2_camera",
                executable="realsense2_camera_node",
                namespace=cam_namespace,
                name="camera",
                output="screen",
                parameters=[params],
                arguments=["--ros-args", "--log-level", log_level],
                emulate_tty=True,
            )
            actions.append(realsense_node)

    recorder_cfg = cfg.get("recorder", {})
    if recorder_cfg.get("enabled", True):
        enable_cameras = len(enabled_camera_names) > 0
        recorder_params: Dict[str, Any] = {
            "output_dir": take_dir,
            "take_name": take_name,
            "mocap_topic": "mocap_Data",
            "wrench_topic": "wrench",
            "record_hz": float(recorder_cfg.get("record_hz", 30.0)),
            "record_enabled": bool(recorder_cfg.get("record_enabled", True)),
            "max_sync_slop_ms": float(recorder_cfg.get("max_sync_slop_ms", 50.0)),
            "enable_cameras": enable_cameras,
        }
        if enable_cameras:
            # launch_ros can't normalize empty arrays (it turns them into () and fails type checks),
            # so only pass camera_names when non-empty.
            recorder_params["camera_names"] = enabled_camera_names

        recorder_node = Node(
            package="data_collection",
            executable="multi_sensor_recorder",
            name="multi_sensor_recorder",
            output="screen",
            additional_env={"PYTHONNOUSERSITE": "1"},
            parameters=[
                recorder_params
            ],
        )
        actions.append(recorder_node)

    screen_cfg = cfg.get("screen_record", {})
    if screen_cfg.get("enabled", False):
        allowed = {
            "backend",
            "monitor",
            "geometry",
            "display",
            "wayland_output",
            "codec",
            "pix_fmt",
            "profile",
            "tune",
            "x264_params",
            "bitrate_kbps",
            "fps",
            "crf",
            "preset",
        }
        screen_params: Dict[str, Any] = {k: v for k, v in screen_cfg.items() if k in allowed}
        screen_params.update(
            {
                "output_dir": take_dir,
                "filename_prefix": take_name,
            }
        )
        screen_node = Node(
            package="data_collection",
            executable="screen_recorder",
            name="screen_recorder",
            output="screen",
            parameters=[screen_params],
        )
        actions.append(screen_node)

    return actions


def generate_launch_description() -> LaunchDescription:
    # Some launch setups evaluate generate_launch_description in a "describe" phase and then
    # again for the real run. Keep all side-effects (take folder creation / counter bump) inside
    # an OpaqueFunction so it only happens during the actual launch execution.
    ld = LaunchDescription()
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
