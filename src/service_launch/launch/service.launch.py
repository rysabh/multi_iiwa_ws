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

    ld.add_action(mocap_node)
    ld.add_action(diffusion_node)
    ld.add_action(natnet_client)

    return ld
    