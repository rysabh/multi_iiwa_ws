import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder(
            "dual_iiwa7", package_name = "dual_arm_w_tool_moveit_config"
        )
        .robot_description(file_path="config/dual_iiwa7.urdf.xacro")
        # .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(
            pipelines = ["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    move_group_capabilities = {
        "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService"
    }

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output = "screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )
   
    run_move_group_blue_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output = "screen",
        namespace="kuka_blue",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    run_move_group_green_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output = "screen",
        namespace="kuka_green",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
        ],
    )

    # rviz_config_file = (
    #     "config/moveit.rviz"
    # )

    rviz_config_file = (
        get_package_share_directory("dual_arm_w_tool_moveit_config") + "/config/moveit.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "kuka_blue_link_0"],
    )

    static_tf_green = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["1.0", "1.0", "0.0", "0.0", "0.0", "0.0", "world", "kuka_green_link_0"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_controllers_path = "config/ros2_controllers.yaml"
    ros2_controllers_path = os.path.join(
        get_package_share_directory("dual_arm_w_tool_moveit_config"),
        "config",
        "ros2_controllers.yaml",
        )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    load_controllers = []
    for controller in [
        "joint_state_broadcaster",
        "kuka_blue_controller",
        "kuka_green_controller",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {}".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    return LaunchDescription(
        [
            rviz_node,
            # static_tf,
            # static_tf_green,
            robot_state_publisher,
            run_move_group_node,
            # run_move_group_blue_node,
            # run_move_group_green_node,
            ros2_control_node,
        ]
        + load_controllers
    )
    # return generate_demo_launch(moveit_config)

    