from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    # ForceTorque Node
    ati_node = Node(
        package='ati_sensor_service',
        executable='ati_service',
        name='ati_service',
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


    sensor_parameters = [
        {'sensor_ip': '192.168.10.100'},  # Replace with your sensor IP
        {'output_file': ft_data_file},  # Dynamically generated file name
        {'sample_rate': '240'}  # Replace with your desired rate
    ]
    
    ati_sensor_node = Node(
            package='data_collection',
            executable='ati_wrench_publisher',
            name='ati_wrench_publisher',
            output='screen',
            parameters=sensor_parameters
        )
    
    ld.add_action(ati_node)
    ld.add_action(ati_sensor_node)

    return ld
    