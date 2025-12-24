import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def _read_yaml(path: str):
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    return data if isinstance(data, dict) else {}


def generate_launch_description() -> LaunchDescription:
    service_launch_share = get_package_share_directory("service_launch")
    config_path = os.path.join(service_launch_share, "config", "sensors.yaml")
    cfg = _read_yaml(config_path)

    recorder_enabled = bool(cfg.get("recorder", {}).get("enabled", True))
    screen_enabled = bool(cfg.get("screen_record", {}).get("enabled", False))

    router_params = {
        "recording_service": "/set_recording" if recorder_enabled else "",
        "screen_recording_service": "/screen_recorder/set_recording" if screen_enabled else "",
    }

    ld = LaunchDescription()

    ld.add_action(
        Node(
            package="data_collection",
            executable="arduino_blue",
            name="arduino_blue",
            output="screen",
            arguments=["--no-interactive"],
        )
    )
    ld.add_action(
        Node(
            package="data_collection",
            executable="arduino_green",
            name="arduino_green",
            output="screen",
            arguments=["--no-interactive"],
        )
    )
    ld.add_action(
        Node(
            package="data_collection",
            executable="switchboard_router",
            name="switchboard_router",
            output="screen",
            parameters=[router_params],
        )
    )

    return ld

