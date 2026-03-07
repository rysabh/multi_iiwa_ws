import re
import tempfile
from pathlib import Path
from typing import Dict, List, Optional, Union

from ament_index_python import get_package_share_directory

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class GazeboMixin:
    @staticmethod
    def include_gazebo(**kwargs) -> IncludeLaunchDescription:
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [
                        FindPackageShare("gazebo_ros"),
                        "launch",
                        "gazebo.launch.py",
                    ]
                )
            ),
            **kwargs,
        )

    @staticmethod
    def node_spawn_entity(
        robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        tf: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        **kwargs,
    ) -> Node:
        label = ["-x", "-y", "-z", "-R", "-P", "-Y"]
        tf = [str(x) for x in tf]
        return Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-topic",
                "robot_description",
                "-entity",
                robot_name,
            ]
            + [item for pair in zip(label, tf) for item in pair],
            output="screen",
            namespace=robot_name,
            **kwargs,
        )


class LBRDescriptionMixin:
    @staticmethod
    def param_robot_description(
        description_package: Optional[
            Union[LaunchConfiguration, str]
        ] = LaunchConfiguration(
            "description_package", default="lbr_description"
        ),
        description_name: Optional[
            Union[LaunchConfiguration, str]
        ] = LaunchConfiguration(
            "xacro_name", default=LaunchConfiguration("model", default="iiwa7")
        ),
        robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        sim: Optional[Union[LaunchConfiguration, bool]] = LaunchConfiguration(
            "sim", default="true"
        ),
        controllers_path: Optional[Union[LaunchConfiguration, str]] = None,
    ) -> Dict[str, str]:
        if type(sim) is bool:
            sim = "true" if sim else "false"
        command = [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare(description_package),
                    "urdf",
                    description_name,
                    description_name,
                ]
            ),
            ".xacro",
            " robot_name:=",
            robot_name,
            " sim:=",
            sim,
        ]
        if controllers_path is not None:
            command.extend(
                [
                    " controllers_path:=",
                    controllers_path,
                ]
            )
        robot_description = {
            "robot_description": Command(command)
        }
        return robot_description

    @staticmethod
    def arg_model(default_value: str = "iiwa7") -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="model",
            default_value=default_value,
            description="The physical LBR model in use.",
            choices=["iiwa7", "iiwa14", "med7", "med14"],
        )

    @staticmethod
    def arg_description_package(default_value: str = "lbr_description") -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="description_package",
            default_value=default_value,
            description="ROS package containing the top-level robot description xacro.",
        )

    @staticmethod
    def arg_description_name(
        default_value: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "model", default="iiwa7"
        ),
    ) -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="xacro_name",
            default_value=default_value,
            description="Top-level xacro entry name. This resolves to urdf/<name>/<name>.xacro within description_package and is independent from robot_name.",
        )

    @staticmethod
    def arg_robot_name(default_value: str = "lbr") -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="robot_name",
            default_value=default_value,
            description="Runtime robot instance name and namespace. This is independent from the selected xacro entry and MoveIt package.",
        )

    @staticmethod
    def arg_sim(default_value: str = "true") -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="sim",
            default_value=default_value,
            description="Whether to use the simulation or not.",
        )

    @staticmethod
    def param_robot_name() -> Dict[str, LaunchConfiguration]:
        return {"robot_name": LaunchConfiguration("robot_name", default="lbr")}

    @staticmethod
    def param_sim() -> Dict[str, LaunchConfiguration]:
        return {"sim": LaunchConfiguration("sim", default="true")}

    @staticmethod
    def node_static_tf(
        tf: List[float] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        parent: Optional[Union[LaunchConfiguration, str]] = None,
        child: Optional[Union[LaunchConfiguration, str]] = None,
        **kwargs,
    ) -> Node:
        label = ["--x", "--y", "--z", "--roll", "--pitch", "--yaw"]
        tf = [str(x) for x in tf]
        return Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen",
            arguments=[item for pair in zip(label, tf) for item in pair]
            + [
                "--frame-id",
                parent,
                "--child-frame-id",
                child,
            ],
            **kwargs,
        )


class RVizMixin:
    @staticmethod
    def configured_rviz_config_path(
        package_name: str,
        config_path: str,
        robot_name: str,
    ) -> str:
        source_path = Path(get_package_share_directory(package_name)) / config_path
        rviz_config = source_path.read_text(encoding="utf-8")
        replacements = (
            (
                r"^(\s*Value:)\s*/[^/]+/robot_description$",
                rf"\1 /{robot_name}/robot_description",
            ),
            (
                r"^(\s*TF Prefix:).*$",
                rf"\1 {robot_name}",
            ),
            (
                r"^(\s*Value:)\s*/[^/]+/force_torque_broadcaster/wrench$",
                rf"\1 /{robot_name}/force_torque_broadcaster/wrench",
            ),
        )
        rendered_config = rviz_config
        replaced_any = False
        for pattern, replacement in replacements:
            rendered_config, count = re.subn(
                pattern,
                replacement,
                rendered_config,
                flags=re.MULTILINE,
            )
            replaced_any = replaced_any or count > 0

        if not replaced_any:
            return str(source_path)

        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            prefix=f"{robot_name}_rviz_",
            suffix=".rviz",
            delete=False,
        ) as temp_config:
            temp_config.write(rendered_config)
            return temp_config.name

    @staticmethod
    def arg_rviz_config_pkg(
        default_value: str = "lbr_description",
    ) -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rviz_config_pkg",
            default_value=default_value,
            description="The RViz configuration file.",
        )

    @staticmethod
    def arg_rviz_config(
        default_value: str = "config/config.rviz",
    ) -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="rviz_config",
            default_value=default_value,
            description="The RViz configuration file.",
        )

    @staticmethod
    def node_rviz(
        rviz_config_path: Optional[Union[LaunchConfiguration, str]] = None,
        rviz_config_pkg: Optional[
            Union[LaunchConfiguration, str]
        ] = LaunchConfiguration("rviz_config_pkg", default="lbr_description"),
        rviz_config: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "rviz_config", default="config/config.rviz"
        ),
        **kwargs,
    ) -> Node:
        config_argument = (
            rviz_config_path
            if rviz_config_path is not None
            else PathJoinSubstitution(
                [
                    FindPackageShare(rviz_config_pkg),
                    rviz_config,
                ]
            )
        )
        return Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d",
                config_argument,
            ],
            **kwargs,
        )
