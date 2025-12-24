"""
DEPRECATED: Use `record_take.launch.py` (configurable via `config/sensors.yaml`).
"""

from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # The ATI node publishes `force_torque` and serves `get_force_torque`.
    ati_node = Node(
        package="ati_sensor_service",
        executable="ati_service",
        name="ati_sensor",
        output="screen",
        parameters=[
            {"sensor_ip": "192.168.10.100"},
            {"sample_rate": 240.0},
        ],
    )
    
    take_dir = "/home/cam/Downloads/GitHub/multi_iiwa_ws/src/service_launch/takes"
    take_number_file = os.path.join(take_dir, "take_number.txt")

    with open(take_number_file, "r") as f:
        take_number = int(f.readline().strip())
    
    with open(take_number_file, "w") as f:
        f.write(str(take_number+1))
    
    
    format_take_number = f"{take_number:03}"
    print(f"\n\n======================\n\nTake number: {format_take_number}\n\n======================\n\n")
    ft_data_file = os.path.join(take_dir, f"ft_{format_take_number}.csv")


    ld.add_action(ati_node)

    return ld
    
