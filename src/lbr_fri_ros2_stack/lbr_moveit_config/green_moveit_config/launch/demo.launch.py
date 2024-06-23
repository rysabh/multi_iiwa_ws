from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("iiwa7", package_name="green_moveit_config")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    return generate_demo_launch(moveit_config)