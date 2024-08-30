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

class SequenceMotion(Node):
    def __init__(self):
        super().__init__("sequence_motion")
        self._action_client = ActionClient(self, MoveGroupSequence, "/sequence_move_group")

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {self._action_client._action_name}...")

        self._base = "world"
        self._end_effectors = {
            "kuka_green": "kuka_green_link_ee",
            "kuka_blue": "kuka_blue_link_ee",
        }

    def motion_plan_request(self, target_pose: Pose, move_group: str) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "LIN"
        req.allowed_planning_time = 10.0
        req.group_name = move_group
        req.max_acceleration_scaling_factor = 0.1
        req.max_velocity_scaling_factor = 0.1
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
                        blend_radius = 0.01,
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
                    else:
                        self.get_logger().info(f"Failed to get result")
                else:
                    self.get_logger().info(f"Result Future did not complete")
            else:
                self.get_logger().info(f"Goal not accepted")
        else:
            self.get_logger().info(f"Future did not complete")
    

def main(args=None):
    rclpy.init(args=args)
    sequence_motion = SequenceMotion()
    target_poses_dict = {
        "kuka_green": [
            Pose(position=Point(x=1.18, y=1.08, z=1.08), orientation=Quaternion(w=1.0)),
            Pose(position=Point(x=1.24, y=1.11, z=1.02), orientation=Quaternion(w=1.0)),
            Pose(position=Point(x=1.29, y=1.14, z=0.97), orientation=Quaternion(w=1.0)),
        ],
        "kuka_blue": [
            Pose(position = Point(x = 0.0, y = 0.0, z = 1.0), orientation = Quaternion(w = 1.0)),
            Pose(position = Point(x = 0.2, y = 0.2, z = 0.8), orientation = Quaternion(w = 1.0)),
            Pose(position = Point(x = -0.06, y = -0.32, z = 1.0), orientation = Quaternion(w = 1.0)),
        ]
    }
    sequence_motion.execute_sequence_motion(target_poses_dict=target_poses_dict)
    rclpy.shutdown()

if __name__ == "__main__":
    main()