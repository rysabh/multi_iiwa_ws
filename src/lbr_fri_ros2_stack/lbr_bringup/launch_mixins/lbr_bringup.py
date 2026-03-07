import os
import re
import tempfile
from pathlib import Path
from typing import Any, Dict, List
import yaml

from ament_index_python import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigs, MoveItConfigsBuilder

# NOTE TO SELF:
# due to individual moveit configs, put mixins into lbr_bringup rather than lbr_moveit_config
# most of the configs are taken from Python package moveit_configs_utils.launches


class LBRMoveGroupMixin:
    @staticmethod
    def moveit_config_name(package_name: str, fallback: str) -> str:
        package_share = Path(get_package_share_directory(package_name))
        setup_assistant_path = package_share / ".setup_assistant"
        if setup_assistant_path.is_file():
            with setup_assistant_path.open("r", encoding="utf-8") as file:
                setup_assistant = yaml.safe_load(file) or {}
            srdf_relative_path = (
                setup_assistant.get("moveit_setup_assistant_config", {})
                .get("srdf", {})
                .get("relative_path", "")
            )
            if srdf_relative_path:
                return Path(srdf_relative_path).stem

        srdf_files = sorted((package_share / "config").glob("*.srdf"))
        if len(srdf_files) == 1:
            return srdf_files[0].stem

        return fallback

    @staticmethod
    def arg_moveit_config_pkg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="moveit_config_pkg",
            default_value=PythonExpression(
                ["'", LaunchConfiguration("model", default="iiwa7"), "_moveit_config'"]
            ),
            description="MoveIt configuration package to load. Defaults to <model>_moveit_config; override this when using a custom description entry.",
        )

    @staticmethod
    def arg_allow_trajectory_execution() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="allow_trajectory_execution",
            default_value="true",
        )

    @staticmethod
    def args_publish_monitored_planning_scene() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="publish_monitored_planning_scene",
            default_value="true",
        )

    @staticmethod
    def arg_capabilities() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="capabilities",
            default_value="",
            description="Non-default space separated non-default list of MoveGroup capabilities.",
        )

    @staticmethod
    def arg_disable_capabilities() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="disable_capabilities",
            default_value="",
            description="Disable MoveGroup capabilities, space separated list.",
        )

    @staticmethod
    def arg_monitor_dynamics() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="monitor_dynamics",
            default_value="false",
            description="Whether to copy robot dynamics into MoveGroup.",
        )

    @staticmethod
    def moveit_configs_builder(
        model: str,
        description_package: str,
        description_name: str,
        package_name: str,
        robot_name: str,
        sim: str = "false",
        **kwargs,
    ) -> MoveItConfigsBuilder:
        moveit_config_name = LBRMoveGroupMixin.moveit_config_name(
            package_name=package_name,
            fallback=model,
        )
        return (
            MoveItConfigsBuilder(
                robot_name=moveit_config_name,
                package_name=package_name,
            )
            .robot_description(
                os.path.join(
                    get_package_share_directory(description_package),
                    f"urdf/{description_name}/{description_name}.xacro",
                ),
                mappings={
                    "robot_name": robot_name,
                    "sim": sim,
                },
            )
            .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl", "pilz_industrial_motion_planner"])
        )

    @staticmethod
    def configured_rviz_config_path(
        package_name: str,
        config_path: str,
        move_group_namespace: str,
    ) -> str:
        source_path = Path(get_package_share_directory(package_name)) / config_path
        rviz_config = source_path.read_text(encoding="utf-8")
        rendered_config, replacements = re.subn(
            r'^(\s*Move Group Namespace:).*$',
            rf'\1 "{move_group_namespace}"',
            rviz_config,
            flags=re.MULTILINE,
        )
        if replacements == 0:
            return str(source_path)

        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            prefix=f"{move_group_namespace or 'move_group'}_rviz_",
            suffix=".rviz",
            delete=False,
        ) as temp_config:
            temp_config.write(rendered_config)
            return temp_config.name

    @staticmethod
    def params_move_group() -> Dict[str, Any]:
        move_group_configuration = {
            "publish_robot_description_semantic": True,
            "allow_trajectory_execution": LaunchConfiguration(
                "allow_trajectory_execution"
            ),
            # Note: Wrapping the following values is necessary so that the parameter value can be the empty string
            "capabilities": ParameterValue(
                LaunchConfiguration("capabilities"), value_type=str
            ),
            "capabilities": "pilz_industrial_motion_planner/MoveGroupSequenceAction pilz_industrial_motion_planner/MoveGroupSequenceService",
            "disable_capabilities": ParameterValue(
                LaunchConfiguration("disable_capabilities"), value_type=str
            ),
            # Publish the planning scene of the physical robot so that rviz plugin can know actual robot
            "publish_planning_scene": LaunchConfiguration(
                "publish_monitored_planning_scene"
            ),
            "publish_geometry_updates": LaunchConfiguration(
                "publish_monitored_planning_scene"
            ),
            "publish_state_updates": LaunchConfiguration(
                "publish_monitored_planning_scene"
            ),
            "publish_transforms_updates": LaunchConfiguration(
                "publish_monitored_planning_scene"
            ),
            "monitor_dynamics": False,
        }
        return move_group_configuration

    @staticmethod
    def params_rviz(
        moveit_configs: MoveItConfigs,
    ) -> List[Dict[str, Any]]:
        return [
            moveit_configs.planning_pipelines,
            moveit_configs.robot_description_kinematics,
        ]

    @staticmethod
    def node_move_group(**kwargs) -> Node:
        return Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            **kwargs,
        )
