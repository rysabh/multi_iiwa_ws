from typing import List

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
from rclpy.serialization import serialize_message

class SequenceMotion(Node):
    def __init__(self, move_group: str = "kuka_green"):
        super().__init__("sequence_motion")
        self._action_client = ActionClient(self, MoveGroupSequence, "/sequence_move_group")

        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {self._action_client._action_name}...")

        self._base = "world"
        self._end_effector = "kuka_green_link_ee"
        self._move_group_name = move_group

    def motion_plan_request(self, target_pose: Pose) -> MotionPlanRequest:
        req = MotionPlanRequest()
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "LIN"
        req.allowed_planning_time = 10.0
        req.group_name = self._move_group_name
        req.max_acceleration_scaling_factor = 0.1
        req.max_velocity_scaling_factor = 0.1
        req.num_planning_attempts = 1000

        req.goal_constraints.append(
            Constraints(
                position_constraints=[
                    PositionConstraint(
                        header = Header(frame_id = self._base),
                        link_name = self._end_effector,
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
                        link_name = self._end_effector,
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

    def execute_sequence_motion(self, target_poses: List[Pose]) -> None:
        goal = MoveGroupSequence.Goal()
        for target_pose in target_poses:
            goal.request.items.append(
                MotionSequenceItem(
                    blend_radius = 0.01,
                    req = self.motion_plan_request(target_pose=target_pose),
                )
            )
        goal.request.items[-1].blend_radius = 0.0
        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        print()
    #     future.add_done_callback(self.goal_response_callback)

    # def goal_response_callback(self, future):
    #     goal_handle = future.result()
    #     if not goal_handle.accepted:
    #         self.get_logger().info("Goal rejected")
    #         return
    #     self.get_logger().info("Goal accepted")
    #     result_future = goal_handle.get_result_async()
    #     result_future.add_done_callback(self.result_response_callback)

    # def result_response_callback(self, future):
    #     result = future.result().result
    #     if result.error_code.val != result.SUCCESS:
    #         self.get_logger().info(f"Goal failed with error code: {result.error_code.val}")
    #     else:
    #         self.get_logger().info("Goal succeeded")
    #         serialized_traj = serialize_message(result.planned_trajectory)
    #         print(serialized_traj)

def main(args=None):
    rclpy.init(args=args)
    sequence_motion_green = SequenceMotion(move_group="kuka_green")
    target_poses = [
        # Pose(position = Point(x = 0.0, y = 0.0, z = 1.0), orientation = Quaternion(w = 1.0)),
        # Pose(position = Point(x = 0.2, y = 0.2, z = 0.8), orientation = Quaternion(w = 1.0)),
        # Pose(position = Point(x = -0.06, y = -0.32, z = 1.0), orientation = Quaternion(w = 1.0)),
        # Pose(position = Point(x = 0.38, y = -0.33, z = 1.0), orientation = Quaternion(w = 1.0)),

        Pose(position = Point(x = 1.18, y = 1.08, z = 1.08), orientation = Quaternion(w = 1.0)),
        Pose(position = Point(x = 1.24, y = 1.11, z = 1.02), orientation = Quaternion(w = 1.0)),
        Pose(position = Point(x = 1.29, y = 1.14, z = 0.97), orientation = Quaternion(w = 1.0)),
    ]
    sequence_motion_green.execute_sequence_motion(target_poses=target_poses)
    rclpy.shutdown()

if __name__ == "__main__":
    main()