from sensor_msgs.msg import JointState

from moveit_msgs.msg import (
    RobotState,
    RobotTrajectory,
    MoveItErrorCodes,
    Constraints,
    JointConstraint,
)
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
from builtin_interfaces.msg import Duration
from trajectory_msgs.msg import JointTrajectory

def robodk_2_ros(TxyzQwxyz: list) -> list:
    return [TxyzQwxyz[0], TxyzQwxyz[1], TxyzQwxyz[2], TxyzQwxyz[4], TxyzQwxyz[5], TxyzQwxyz[6], TxyzQwxyz[3]]

def joint_list_2_state(joint_positions: list, joint_names: list) -> JointState:
    joint_state = JointState()
    joint_state.name = joint_names
    joint_state.position = joint_positions
    return joint_state

def joint_state_2_list(joint_state: JointState) -> list:
    return joint_state.position


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

def TxyzQwxyz_2_Pose(TxyzQxyzw: list) -> Pose:
    pose = Pose()
    pose.position.x = TxyzQxyzw[0]
    pose.position.y = TxyzQxyzw[1]
    pose.position.z = TxyzQxyzw[2]
    pose.orientation.x = TxyzQxyzw[3]
    pose.orientation.y = TxyzQxyzw[4]
    pose.orientation.z = TxyzQxyzw[5]
    pose.orientation.w = TxyzQxyzw[6]
    return pose
    
def combine_trajectories(trajectories: list[RobotTrajectory]) -> RobotTrajectory:
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