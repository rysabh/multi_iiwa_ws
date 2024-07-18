green_traj_points = []
    blue_traj_points = []
    green_spline_trajectory = []
    blue_spline_trajectory = []

    for point in planned_joint_trajectory.points:
        green_waypoint = JointTrajectoryPoint()
        blue_waypoint = JointTrajectoryPoint()
        robot_dof = int(len(point.positions) / 2)
        green_waypoint.positions = point.positions[robot_dof:]
        blue_waypoint.positions = point.positions[:robot_dof]
        green_waypoint.accelerations = point.accelerations[robot_dof:]
        blue_waypoint.accelerations = point.accelerations[:robot_dof]
        green_waypoint.velocities = point.velocities[robot_dof:]
        blue_waypoint.velocities = point.velocities[:robot_dof]
        green_waypoint.effort = point.effort
        blue_waypoint.effort = point.effort
        green_waypoint.time_from_start = point.time_from_start
        blue_waypoint.time_from_start = point.time_from_start
        green_traj_points.append(green_waypoint)
        blue_traj_points.append(blue_waypoint)

    if not total_trajectory.points:
        last_time = 0.0
    else:
        last_point = total_trajectory.points[-1]
        last_time = last_point.time_from_start.sec + last_point.time_from_start.nanosec / 1e9

    for point in trajectory.points:
        new_point = JointTrajectoryPoint()
        new_point.positions = point.positions
        new_point.velocities = point.velocities
        new_point.accelerations = point.accelerations
        new_point.effort = point.effort
        new_point.time_from_start.sec = point.time_from_start.sec + int(last_time)
        new_point.time_from_start.nanosec = point.time_from_start.nanosec + int((last_time - int(last_time)) * 1e9)
        self.total_trajectory.points.append(new_point)

    current_state_dual_inverse_list = dual_inverse_list

    green_joint_traj = JointTrajectory()
    green_joint_traj.header.stamp.sec = 0
    green_joint_traj.header.stamp.nanosec = 0
    green_joint_traj.header.frame_id = ''
    green_joint_traj.joint_names = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']
    green_joint_traj.points = green_traj_points

    blue_joint_traj = JointTrajectory()
    blue_joint_traj.header.stamp.sec = 0
    blue_joint_traj.header.stamp.nanosec = 0
    blue_joint_traj.header.frame_id = ''
    blue_joint_traj.joint_names = ['A1', 'A2', 'A3', 'A4', 'A5', 'A6', 'A7']
    blue_joint_traj.points = blue_traj_points

    save_trajectory(blue_joint_traj, "kuka_blue.dump")
    save_trajectory(green_joint_traj, "kuka_green.dump")
    save_trajectory_to_csv(blue_joint_traj, "kuka_blue.csv")
    save_trajectory_to_csv(green_joint_traj, "kuka_green.csv")