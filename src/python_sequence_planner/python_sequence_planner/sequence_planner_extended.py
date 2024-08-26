from typing import List, Dict
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import Point, Quaternion, Pose
from moveit_msgs.action import MoveGroupSequence
import pandas as pd
from moveit_msgs.msg import MotionSequenceItem, MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume, RobotState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Header

class SequenceMotion(Node):
    def __init__(self):
        super().__init__("sequence_motion")
        self._action_client = ActionClient(self, MoveGroupSequence, "/sequence_move_group")
        while not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for {self._action_client._action_name}...")
        self._end_effector = "kuka_blue_link_ee"

    def motion_plan_request(self, target_pose: Pose) -> MotionPlanRequest:
        # Your existing method code here adapted for the single move group
        req = MotionPlanRequest()
        req.group_name = "kuka_blue"
        req.allowed_planning_time = 10.0
        req.pipeline_id = "pilz_industrial_motion_planner"
        req.planner_id = "LIN"
        start_state = RobotState()
        start_state.joint_state.name = ["kuka_blue_A1", "kuka_blue_A2", "kuka_blue_A3", "kuka_blue_A4", "kuka_blue_A5", "kuka_blue_A6", "kuka_blue_A7"]
        start_state.joint_state.position = [2.37, -0.28, -2.27, -1.56, -0.19, 1.27, 0.21]
        # req.start_state = start_state
        req.max_acceleration_scaling_factor = 0.1
        req.max_velocity_scaling_factor = 0.1
        req.num_planning_attempts = 10
        req.goal_constraints.append(
            Constraints(
                position_constraints=[
                    PositionConstraint(
                        header=Header(frame_id="world"),
                        link_name=self._end_effector,
                        constraint_region=BoundingVolume(
                            primitives=[SolidPrimitive(type=2, dimensions=[0.0001])],
                            primitive_poses=[Pose(position=target_pose.position)],
                        ),
                        weight=1.0,
                    )
                ],
                orientation_constraints=[
                    OrientationConstraint(
                        header=Header(frame_id="world"),
                        link_name=self._end_effector,
                        orientation=target_pose.orientation,
                        absolute_x_axis_tolerance=0.001,
                        absolute_y_axis_tolerance=0.001,
                        absolute_z_axis_tolerance=0.001,
                        weight=1.0,
                    )
                ],
            )
        )
        return req

    def execute_sequence_motion(self, target_poses: List[Pose]) -> None:
        # Your existing method code here adapted for the single move group
        goal = MoveGroupSequence.Goal() 

        for target_pose in target_poses:
                motion_request = self.motion_plan_request(target_pose=target_pose)
                goal.request.items.append(
                    MotionSequenceItem(
                        blend_radius = 0.001,
                        req = motion_request,
                    )
                )

        if goal.request.items:
            goal.request.items[-1].blend_radius = 0.0

        future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        # if future.done():
        #     goal_handle = future.result()
        #     if goal_handle.accepted:
        #         result_future = goal_handle.get_result_async()
        #         rclpy.spin_until_future_complete(self, result_future)
        #         if result_future.done():
        #             action_result = result_future.result().result
        #             if action_result:
        #                 self.get_logger().info(f"Trajectory executed and saved")
        #             else:
        #                 self.get_logger().info(f"Failed to get result")
        #         else:
        #             self.get_logger().info(f"Result Future did not complete")
        #     else:
        #         self.get_logger().info(f"Goal not accepted")
        # else:
        #     self.get_logger().info(f"Future did not complete")
        # pass

def read_poses_from_csv(csv_path: str) -> List[Pose]:
    data = pd.read_csv(csv_path)
    poses = []
    for _, row in data.iterrows():
        pose = Pose(
            position=Point(x=row['Position_X'], y=row['Position_Y'], z=row['Position_Z']),
            orientation=Quaternion(x=row['Orientation_X'], y=row['Orientation_Y'], z=row['Orientation_Z'], w=row['Orientation_W'])  # Adjust as needed for your actual orientation data
        )
        poses.append(pose)
    poses = [poses[0]]
    return poses

def main(args=None):
    rclpy.init(args=args)
    sequence_motion = SequenceMotion()
    try:
        csv_path = '/home/battery/Downloads/fk_results_1_quaternion.csv'  # Update this to the path of your CSV file
        target_poses = read_poses_from_csv(csv_path)
        sequence_motion.execute_sequence_motion(target_poses)
    finally:
        sequence_motion.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
