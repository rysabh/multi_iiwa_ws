from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

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

    return ld
    