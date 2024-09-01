from typing import List, Dict
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Quaternion, Pose
from moveit_msgs.action import MoveGroupSequence
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MotionPlanRequest,
    MotionSequenceItem,
    OrientationConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header
import moveit_motion.diffusion_policy_cam.submodules.cleaned_file_parser as cfp
import moveit_motion.ros_submodules.ros_math as rm

class SequenceMotion(Node):
    def __init__(self, move_groups: List[str]):
        super().__init__("sequence_motion")
        self._action_client = ActionClient(self, MoveGroupSequence, f"/sequence_move_group")

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {self._action_client._action_name}...")
        self._base = "world"
        self._end_effectors = {
            move_group : f"{move_group}_link_tcp" for move_group in move_groups
        }
        print(self._end_effectors)


    @staticmethod
    def data_2_pose(data: List) -> List[Pose]:
        kuka_blue_poses = []

        for pose_data in data:
            pose = Pose()
            pose.position.x = pose_data[0]
            pose.position.y = pose_data[1]
            pose.position.z = pose_data[2]
            pose.orientation.x = pose_data[3]
            pose.orientation.y = pose_data[4]
            pose.orientation.z = pose_data[5]
            pose.orientation.w = pose_data[6]
            kuka_blue_poses.append(pose)
        return kuka_blue_poses
    
    def motion_plan_request(self, target_pose: Pose, move_group: str) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "LIN"
        req.allowed_planning_time = 10.0
        req.group_name = move_group
        req.max_acceleration_scaling_factor = 0.5
        req.max_velocity_scaling_factor = 0.5
        req.num_planning_attempts = 1000

        req.goal_constraints.append(
            Constraints(
                position_constraints=[
                    PositionConstraint(
                        header = Header(frame_id = self._base),
                        link_name = self._end_effectors[move_group],
                        constraint_region = BoundingVolume(
                            primitives = [SolidPrimitive(type=2, dimensions=[0.0001])],
                            primitive_poses = [Pose(position=target_pose.position)],
                        ),
                        weight = 1.0,
                    )
                ],
                orientation_constraints = [
                    OrientationConstraint(
                        header = Header(frame_id = self._base),
                        link_name = self._end_effectors[move_group],
                        orientation = target_pose.orientation,
                        absolute_x_axis_tolerance = 0.001,
                        absolute_y_axis_tolerance = 0.001,
                        absolute_z_axis_tolerance = 0.001,
                        weight = 1.0,
                    )
                ],
            )
        )

        return req
    
    def execute_sequence_motion(self, target_poses_dict: Dict[str, List[Pose]]) -> None:
        
        goal = MoveGroupSequence.Goal() 

        for move_group, target_poses in target_poses_dict.items():
            for target_pose in target_poses:
                motion_request = self.motion_plan_request(target_pose=target_pose, move_group=move_group)
                goal.request.items.append(
                    MotionSequenceItem(
                        blend_radius = 0.00001,
                        req = motion_request,
                    )
                )

        if goal.request.items:
            goal.request.items[-1].blend_radius = 0.0

        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        if future.done():
            goal_handle = future.result()
            if goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                if result_future.done():
                    action_result = result_future.result().result
                    if action_result:
                        self.get_logger().info(f"Trajectory executed and saved")
                        print(action_result)
                    else:
                        self.get_logger().info(f"Failed to get result")
                else:
                    self.get_logger().info(f"Result Future did not complete")
            else:
                self.get_logger().info(f"Goal not accepted")
        else:
            self.get_logger().info(f"Future did not complete")

def main(_robot_name, _file_name):
    rclpy.init()
    move_groups = ["kuka_blue", "kuka_green"]
    sequence_motion = SequenceMotion(move_groups = move_groups)
    path = f'no-sync/edge_3/{_file_name}'
    FPS = 15
    data = cfp.DataParser.from_quat_file(file_path = path, target_fps= FPS, filter=False, window_size=5, polyorder=3)

    gripper_data = data.get_rigid_TxyzQwxyz()['gripper']
    chisel_data = data.get_rigid_TxyzQwxyz()['chisel']

    kuka_blue = []

    for _ in range(len(gripper_data)):
        ros_frame = rm.robodk_2_ros(gripper_data[_])
        kuka_blue.append(ros_frame)
    print(len(gripper_data))                                        #DEBUGGING
    poses_blue = sequence_motion.data_2_pose(kuka_blue)
    print(len(poses_blue))                                       #DEBUGGING

    if _robot_name == "kuka_blue":
        sequence_motion.execute_sequence_motion(target_poses_dict = {"kuka_blue": poses_blue})

    # ---------- kuka_green --------------  
    kuka_green = []

    for _ in range(len(chisel_data)):
        ros_frame = rm.robodk_2_ros(chisel_data[_])
        kuka_green.append(ros_frame)
    print(len(chisel_data))                                        #DEBUGGING
    poses_green = sequence_motion.data_2_pose(kuka_green)
    print(len(poses_green))                                       #DEBUGGING

    if _robot_name == "kuka_green":
        sequence_motion.execute_sequence_motion(target_poses_dict = {"kuka_green": poses_green})


    # # ----------- dual ---------------
    # kuka_blue = []
    # kuka_green = []

    # for _ in range(len(gripper_data)):
    #     chisel_ros_frame = rm.robodk_2_ros(gripper_data[_])
    #     kuka_blue.append(chisel_ros_frame)
    #     gripper_ros_frame = rm.robodk_2_ros(chisel_data[_])
    #     kuka_green.append(gripper_ros_frame)
    # kuka_blue_poses = sequence_motion.data_2_pose(kuka_blue)
    # kuka_green_poses = sequence_motion.data_2_pose(kuka_green)

    # if _robot_name == "dual":
    #     sequence_motion.execute_sequence_motion(target_poses_dict = {"kuka_blue": kuka_blue_poses, "kuka_green": kuka_green_poses})


if __name__ == "__main__":
    import sys

    _robot_name = sys.argv[1] if len(sys.argv) > 1 else "kuka_green"
    _file_name = sys.argv[2] if len(sys.argv) > 2 else "ft_010.csv"

    main(_robot_name = _robot_name, 
         _file_name = _file_name)