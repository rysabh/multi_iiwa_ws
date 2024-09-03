from sensor_msgs.msg import JointState

from moveit_msgs.msg import (
    RobotState,
    RobotTrajectory,
    MoveItErrorCodes,
    Constraints,
    JointConstraint,
    BoundingVolume,
    OrientationConstraint,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import numpy as np
from typing import Union



def robodk_2_ros(TxyzQwxyz: list) -> list:
    return [TxyzQwxyz[0], TxyzQwxyz[1], TxyzQwxyz[2], TxyzQwxyz[4], TxyzQwxyz[5], TxyzQwxyz[6], TxyzQwxyz[3]]

def joint_list_2_state(joint_positions: list, joint_names: list) -> JointState:
    joint_state = JointState()
    joint_state.name = joint_names
    joint_state.position = [float(pos) for pos in joint_positions]
    joint_state.velocity = [0.0] * len(joint_positions)
    joint_state.effort = [0.0] * len(joint_positions)
    return joint_state

from sensor_msgs.msg import JointState

def joint_state_2_list(joint_state: JointState, **kwargs) -> list:
    """
    def joint_state_2_list(joint_state: JointState) -> list: print -> enabled
    """
    
    # Sort the joint names and positions together based on the joint names
    sorted_joints = sorted(zip(joint_state.name, joint_state.position))
    
    # Extract the sorted positions
    sorted_positions = [pos for _, pos in sorted_joints]
    
    if kwargs.get("verbose", False):
        #Prepare the XML-like output
        group_name = "YY"
        state_name = "XX"
        
        print(f'<group_state name="{state_name}" group="{group_name}">')
        for joint_name, joint_position in sorted_joints:
            print(f'    <joint name="{joint_name}" value="{joint_position:.3f}"/>')
        print(f'</group_state>\n')
        
        # Prepare the YAML-like output
        print(f'initial_positions:')
        for joint_name, joint_position in sorted_joints:
            print(f'  {joint_name}: {joint_position:.3f}')
    
    return sorted_positions




def joint_2_robot_state(joint_state: JointState) -> RobotState:
    robot_state = RobotState()
    robot_state.joint_state = joint_state
    return robot_state


def robot_2_joint_state(robot_state: RobotState) -> JointState:
    return robot_state.joint_state



def _joint_state_2_constraints(joint_state: JointState) -> list:
    _joint_constraints = []
    for name, position in zip(joint_state.name, joint_state.position):
        joint_constraint = JointConstraint()
        joint_constraint.joint_name = name
        joint_constraint.position = position
        joint_constraint.tolerance_above = 0.001
        joint_constraint.tolerance_below = 0.001
        joint_constraint.weight = 1.0
        _joint_constraints.append(joint_constraint)
    return _joint_constraints

#multiple joint_state to constraints
def joint_states_2_constraints(*joint_states : JointState) -> Constraints:
    constraints = Constraints()
    for joint_state in joint_states:
        _joint_constraints = _joint_state_2_constraints(joint_state)
        constraints.joint_constraints.extend(_joint_constraints)
    return constraints


def _pose_to_constraints(target_pose: Pose, frame_id: str, link_name: str) -> Constraints:
    """Generate Constraints from a Pose."""
    constraints = Constraints(
        position_constraints=[
            PositionConstraint(
                header=Header(frame_id=frame_id),
                link_name=link_name,
                constraint_region=BoundingVolume(
                    primitives=[SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[0.0001])],
                    primitive_poses=[Pose(position=target_pose.position)],
                ),
                weight=1.0,
            )
        ],
        orientation_constraints=[
            OrientationConstraint(
                header=Header(frame_id=frame_id),
                link_name=link_name,
                orientation=target_pose.orientation,
                absolute_x_axis_tolerance=0.001,
                absolute_y_axis_tolerance=0.001,
                absolute_z_axis_tolerance=0.001,
                weight=1.0,
            )
        ],
    )
    return constraints

def poses_to_constraints(poses: list[Pose], frame_id: str, link_name: str) -> Constraints:
    """Generate Constraints from multiple Poses."""
    constraints = Constraints()
    for pose in poses:
        pose_constraints = _pose_to_constraints(pose, frame_id, link_name)
        constraints.position_constraints.extend(pose_constraints.position_constraints)
        constraints.orientation_constraints.extend(pose_constraints.orientation_constraints)
    return constraints


def TxyzQxyzw_2_Pose(TxyzQxyzw: list) -> Pose:
    pose = Pose()
    pose.position.x = TxyzQxyzw[0]
    pose.position.y = TxyzQxyzw[1]
    pose.position.z = TxyzQxyzw[2]
    pose.orientation.x = TxyzQxyzw[3]
    pose.orientation.y = TxyzQxyzw[4]
    pose.orientation.z = TxyzQxyzw[5]
    pose.orientation.w = TxyzQxyzw[6]
    return pose
    
def extend_trajectories(trajectories: list[RobotTrajectory]) -> RobotTrajectory:
    print("Combining Trajectories")
    
    if not trajectories:
        raise ValueError("The list of trajectories is empty.")
    
    # Initialize the combined trajectory with the first trajectory
    combined_trajectory = trajectories[0]
    combined_points = list(combined_trajectory.joint_trajectory.points)
    
    # Iterate over the remaining trajectories
    current_time = combined_points[-1].time_from_start if combined_points else Duration(sec=0, nanosec=0)
    
    for robot_traj in trajectories[1:]:
        joint_trajectory = robot_traj.joint_trajectory
        for point in joint_trajectory.points:
            # Adjust time_from_start for each point in the current trajectory
            point.time_from_start.sec += current_time.sec
            point.time_from_start.nanosec += current_time.nanosec
            if point.time_from_start.nanosec >= 1e9:
                point.time_from_start.sec += 1
                point.time_from_start.nanosec -= int(1e9)
        
        # Append points, excluding the first point to avoid duplicating the end/start point
        combined_points.extend(joint_trajectory.points[1:])
        
        # Update current time for the next trajectory
        current_time = combined_points[-1].time_from_start if combined_points else Duration(sec=0, nanosec=0)
    
    # Create the final combined JointTrajectory message
    combined_joint_trajectory = JointTrajectory()
    combined_joint_trajectory.header = combined_trajectory.joint_trajectory.header
    combined_joint_trajectory.joint_names = combined_trajectory.joint_trajectory.joint_names
    combined_joint_trajectory.points = combined_points
    
    # Create the final combined RobotTrajectory message
    combined_robot_trajectory = RobotTrajectory()
    combined_robot_trajectory.joint_trajectory = combined_joint_trajectory
    
    print("Trajectories Combined")
    return combined_robot_trajectory


def joint_points_2_trajectory(points: Union[np.ndarray, list], 
                              times: Union[np.ndarray, list], 
                              header_frame_id: str, 
                              joint_names: list, 
                              **kwargs) -> RobotTrajectory:
    
    # Initialize a RobotTrajectory message
    trajectory_msg = RobotTrajectory()
    trajectory_msg.joint_trajectory.header.frame_id = header_frame_id
    trajectory_msg.joint_trajectory.joint_names = joint_names

    # Check if times are provided, if not generate them
    if not times:
        sampling_rate = float(kwargs.get("sampling_rate", 1))
        times = [i / sampling_rate for i in range(len(points))]

    # Iterate over the points and times to create JointTrajectoryPoints
    for i in range(len(points)):
        # Create a JointTrajectoryPoint
        trajectory_point = JointTrajectoryPoint()
        
        # Assign joint positions
        trajectory_point.positions = list(points[i])

        # Set the time_from_start
        trajectory_point.time_from_start = Duration(sec=int(times[i]), nanosec=int((times[i] % 1) * 1e9))

        # Append the point to the trajectory message
        trajectory_msg.joint_trajectory.points.append(trajectory_point)

    return trajectory_msg



def interpolate_trajectory_timestamps(trajectory: RobotTrajectory,
                                      timestamps: list[float],
                                      scaling_factor:float = 1.0) -> RobotTrajectory:
    # Interpolate times for each point in the trajectory
    total_waypoints = len(timestamps)
    total_points = len(trajectory.joint_trajectory.points)
    
    _start_time = timestamps[0]
    # subtract the start time from all timestamps
    timestamps = [_t - _start_time for _t in timestamps]

    if total_waypoints < 2 or total_points < 2:
        raise ValueError("There must be at least 2 waypoints and 2 trajectory points for interpolation.")
    
    interpolated_times = []

    for i in range(total_points):
        # Calculate the relative position of the current point within the total trajectory
        relative_position = i / (total_points - 1) * (total_waypoints - 1)
        segment_index = int(relative_position)
        
        if segment_index >= total_waypoints - 1:
            segment_index = total_waypoints - 2
        
        t0 = timestamps[segment_index]
        t1 = timestamps[segment_index + 1]
        
        # Interpolate time based on the relative position within the segment
        alpha = relative_position - segment_index
        interpolated_time = t0 + alpha * (t1 - t0)
        interpolated_times.append(interpolated_time*scaling_factor)

    # Assign the interpolated times to each point in the trajectory
    for i, trajectory_point in enumerate(trajectory.joint_trajectory.points):
        sec = int(interpolated_times[i])
        nanosec = int((interpolated_times[i] % 1) * 1e9)
        trajectory_point.time_from_start = Duration(sec=sec, nanosec=nanosec)

    return trajectory



