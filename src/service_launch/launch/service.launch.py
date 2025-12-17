from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    log_level = "warn"
    ld = LaunchDescription()
    
    # Mocap Node -->  redundant with natnet client
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
    # ForceTorque Node --> redundant with ati_wrench_publisher
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

    config = os.path.join(
        get_package_share_directory('mocap_optitrack_client'),
        'config',
        'natnetclient.yaml'
    )

    natnet_client = Node(
        package='mocap_optitrack_client',
        executable='mocap_optitrack_client',
        name='natnet_client',
        parameters=[
            config,
            {'record': True, 'take_name': f"ft_{format_take_number}"},
        ],
        arguments=['--ros-args', '--log-level', log_level]
    )

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
    
    arduino_node = Node(
        package='arduino_signal',
        executable='arduino_node',
        name='arduino_node',
        output = 'screen',
    )
    
    # ld.add_action(diffusion_node)
    
    
    ld.add_action(natnet_client) # orginal service
    ld.add_action(mocap_node) # redundant service
    
    ld.add_action(ati_sensor_node) # original service
    ld.add_action(ati_node) # redundant service
    
    
    
    # ld.add_action(arduino_node)

    return ld
    
