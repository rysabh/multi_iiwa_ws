import re
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Dict, Optional, Union

import xacro
import yaml
from ament_index_python import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


class LBRROS2ControlMixin:
    class _NoAliasDumper(yaml.SafeDumper):
        def ignore_aliases(self, data):
            return True

    @staticmethod
    def _resolve_value(
        context: LaunchContext,
        value: Optional[Union[LaunchConfiguration, str, bool]],
        default: str = "",
    ) -> str:
        if isinstance(value, LaunchConfiguration):
            return value.perform(context)
        if isinstance(value, bool):
            return "true" if value else "false"
        if value is None:
            return default
        return str(value)

    @staticmethod
    def controller_config_path(
        context: LaunchContext,
        description_package: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "description_package", default="lbr_description"
        ),
        description_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "xacro_name", default=LaunchConfiguration("model", default="iiwa7")
        ),
        robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        sim: Optional[Union[LaunchConfiguration, str, bool]] = LaunchConfiguration(
            "sim", default="false"
        ),
        ctrl_cfg_pkg: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "ctrl_cfg_pkg", default="lbr_ros2_control"
        ),
        ctrl_cfg: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "ctrl_cfg", default="config/lbr_controllers.yaml"
        ),
    ) -> str:
        description_package_value = LBRROS2ControlMixin._resolve_value(
            context, description_package, "lbr_description"
        )
        description_name_value = LBRROS2ControlMixin._resolve_value(
            context, description_name, "iiwa7"
        )
        robot_name_value = LBRROS2ControlMixin._resolve_value(context, robot_name, "lbr")
        sim_value = LBRROS2ControlMixin._resolve_value(context, sim, "false")
        ctrl_cfg_pkg_value = LBRROS2ControlMixin._resolve_value(
            context, ctrl_cfg_pkg, "lbr_ros2_control"
        )
        ctrl_cfg_value = LBRROS2ControlMixin._resolve_value(
            context, ctrl_cfg, "config/lbr_controllers.yaml"
        )

        base_controller_config_path = (
            Path(get_package_share_directory(ctrl_cfg_pkg_value)) / ctrl_cfg_value
        )
        with base_controller_config_path.open("r", encoding="utf-8") as config_file:
            controller_config = yaml.safe_load(config_file)

        description_path = (
            Path(get_package_share_directory(description_package_value))
            / "urdf"
            / description_name_value
            / f"{description_name_value}.xacro"
        )
        description_doc = xacro.process_file(
            str(description_path),
            mappings={
                "robot_name": robot_name_value,
                "sim": sim_value,
                "controllers_path": str(base_controller_config_path),
            },
        )
        description_tree = ET.fromstring(description_doc.toxml())
        ros2_control = description_tree.find(".//ros2_control")
        if ros2_control is None:
            raise RuntimeError(f"Could not find ros2_control in {description_path}")

        joint_names = [joint.attrib["name"] for joint in ros2_control.findall("joint")]
        if not joint_names:
            raise RuntimeError(
                f"Could not infer controller joints from {description_path}"
            )

        chain_tip = "link_ee"
        estimated_ft_sensor = ros2_control.find("./sensor[@name='estimated_ft_sensor']")
        if estimated_ft_sensor is not None:
            chain_tip_param = estimated_ft_sensor.find("./param[@name='chain_tip']")
            if chain_tip_param is not None and chain_tip_param.text:
                chain_tip = chain_tip_param.text.strip()

        frame_id = f"{robot_name_value}/{chain_tip}"

        if "/**/force_torque_broadcaster" in controller_config:
            controller_config["/**/force_torque_broadcaster"]["ros__parameters"][
                "frame_id"
            ] = frame_id
        if "/**/joint_trajectory_controller" in controller_config:
            controller_config["/**/joint_trajectory_controller"]["ros__parameters"][
                "joints"
            ] = joint_names
        if "/**/forward_position_controller" in controller_config:
            controller_config["/**/forward_position_controller"]["ros__parameters"][
                "joints"
            ] = joint_names

        with tempfile.NamedTemporaryFile(
            mode="w",
            encoding="utf-8",
            prefix=f"{robot_name_value}_controllers_",
            suffix=".yaml",
            delete=False,
        ) as generated_config:
            rendered_config = yaml.dump(
                controller_config,
                Dumper=LBRROS2ControlMixin._NoAliasDumper,
                sort_keys=False,
            )
            rendered_config = re.sub(
                r'^(\/\*\*\/[^:]+):',
                r'"\1":',
                rendered_config,
                flags=re.MULTILINE,
            )
            generated_config.write(rendered_config)
            return generated_config.name

    @staticmethod
    def arg_ctrl_cfg_pkg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="ctrl_cfg_pkg",
            default_value="lbr_ros2_control",
            description="Controller configuration package. The package containing the ctrl_cfg.",
        )

    @staticmethod
    def arg_ctrl_cfg() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="ctrl_cfg",
            default_value="config/lbr_controllers.yaml",
            description="Relative path from ctrl_cfg_pkg to the controllers.",
        )

    @staticmethod
    def arg_ctrl() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="ctrl",
            default_value="joint_trajectory_controller",
            description="Desired default controller. One of specified in ctrl_cfg.",
            choices=[
                "joint_trajectory_controller",
                "forward_position_controller",
                "lbr_joint_position_command_controller",
                "lbr_torque_command_controller",
                "lbr_wrench_command_controller",
            ],
        )

    @staticmethod
    def arg_use_sim_time() -> DeclareLaunchArgument:
        return DeclareLaunchArgument(
            name="use_sim_time",
            default_value="false",
            description="Use simulation (Gazebo) clock if true.",
        )

    @staticmethod
    def node_ros2_control(
        robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        controller_config_path: Optional[Union[LaunchConfiguration, str]] = None,
        **kwargs,
    ) -> Node:
        parameters = [
            {"use_sim_time": False},
            controller_config_path
            if controller_config_path is not None
            else PathJoinSubstitution(
                [
                    FindPackageShare(
                        LaunchConfiguration("ctrl_cfg_pkg", default="lbr_ros2_control")
                    ),
                    LaunchConfiguration("ctrl_cfg", default="config/lbr_controllers.yaml"),
                ]
            ),
        ]
        return Node(
            package="controller_manager",
            executable="ros2_control_node",
            parameters=parameters,
            namespace=robot_name,
            remappings=[
                ("~/robot_description", "robot_description"),
            ],
            **kwargs,
        )

    @staticmethod
    def node_controller_spawner(
        robot_name: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        controller: Optional[Union[LaunchConfiguration, str]] = LaunchConfiguration(
            "ctrl"
        ),
        **kwargs,
    ) -> Node:
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                controller,
                "--controller-manager",
                "controller_manager",
            ],
            namespace=robot_name,
            **kwargs,
        )

    @staticmethod
    def node_robot_state_publisher(
        robot_description: Dict[str, str],
        robot_name: Optional[LaunchConfiguration] = LaunchConfiguration(
            "robot_name", default="lbr"
        ),
        use_sim_time: Optional[Union[LaunchConfiguration, bool]] = LaunchConfiguration(
            "use_sim_time", default="false"
        ),
        **kwargs,
    ) -> Node:
        return Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[
                robot_description,
                {"use_sim_time": use_sim_time},
                # robot_state_publisher expects frame_prefix to end with "/"
                # neat hack to add trailing slash, which is required by frame_prefix
                {"frame_prefix": PythonExpression(["'", robot_name, "/'"])},
            ],
            namespace=robot_name,
            **kwargs,
        )
