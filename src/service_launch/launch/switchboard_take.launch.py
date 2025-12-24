import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description() -> LaunchDescription:
    service_launch_share = get_package_share_directory("service_launch")
    record_take_launch = os.path.join(service_launch_share, "launch", "record_take.launch.py")
    switchboard_core_launch = os.path.join(service_launch_share, "launch", "switchboard_core.launch.py")

    ld = LaunchDescription()

    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(switchboard_core_launch)))
    ld.add_action(IncludeLaunchDescription(PythonLaunchDescriptionSource(record_take_launch)))

    return ld
