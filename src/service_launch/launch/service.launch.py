from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    log_level = "warn"
    ld = LaunchDescription()
    
    config = os.path.join(
        get_package_share_directory('mocap_optitrack_client'),
        'config',
        'natnetclient.yaml'
    )
    
    
    natnet_client = Node(
        package='mocap_optitrack_client',
        executable='mocap_optitrack_client',
        name='natnet_client',
        parameters = [config],
        arguments=['--ros-args', '--log-level', log_level]
    )

    # Mocap Node
    mocap_node = Node(
        package='mocap_service',
        executable='mocap_service_subscriber',
        name='mocap_service_subscriber',
    )
    # Diffusion Inference Service
    diffusion_node = Node(
        package='diffusion_service',
        executable='diffusion_inference_service',
        name='diffusion_inference_service',
    )
    # ForceTorque Node
    ati_node = Node(
        package='ati_sensor_service',
        executable='ati_service',
        name='ati_service',
    )
    
    take_dir = "src/service_launch/takes"
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
    
    ld.add_action(mocap_node)
    ld.add_action(diffusion_node)
    ld.add_action(natnet_client)
    ld.add_action(ati_node)
    # ld.add_action(ati_sensor_node)

    return ld
    