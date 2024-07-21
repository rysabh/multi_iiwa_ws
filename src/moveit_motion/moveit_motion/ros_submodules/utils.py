from geometry_msgs.msg import Pose, Point, Quaternion

import numpy as np

from scipy.spatial.transform import Rotation


def check_same_pose(
        pose_1: Pose, pose_2: Pose
) -> bool:
    x_diff = np.abs(pose_1.position.x - pose_2.position.x)
    y_diff = np.abs(pose_1.position.y - pose_2.position.y)
    z_diff = np.abs(pose_1.position.z - pose_2.position.z)

    pos_diff = np.asarray([x_diff, y_diff, z_diff])

    qx_diff = np.abs(pose_1.orientation.x - pose_2.orientation.x)
    qy_diff = np.abs(pose_1.orientation.y - pose_2.orientation.y)
    qz_diff = np.abs(pose_1.orientation.z - pose_2.orientation.z)
    qw_diff = np.abs(pose_1.orientation.w - pose_2.orientation.w)

    quat_diff = np.asarray([qx_diff, qy_diff, qz_diff, qw_diff])

    return np.sum(np.square(pos_diff)) < 0.001 and np.sum(np.square(quat_diff)) < 0.001


def construct_transform_matrix(rotation: np.ndarray, translation: np.ndarray):
    transform = np.identity(4)
    transform[0:3, 0:3] = rotation
    transform[0:3, [3]] = translation
    
    return transform


def construct_pose(transform: np.ndarray):
    rotation = transform[0:3, 0:3]
    translation = transform[0:3, 3].reshape(1, 3)

    # scipy quat defination: quat = [x, y, z, w]
    quat = Rotation.from_matrix(rotation).as_quat()

    return Pose(
        position=Point(
            x = translation[0][0],
            y = translation[0][1],
            z = translation[0][2],
        ),
        orientation=Quaternion(
            w = quat[3],
            x = quat[0],
            y = quat[1],
            z = quat[2]
        )
    )


def get_offset_pose(base_pose: Pose, offset_dict: dict[str, float]) -> Pose:
    offset_rotation = Rotation.from_euler(
        seq="ZYX", 
        angles=[offset_dict['A'], offset_dict['B'], offset_dict['C']],
        degrees=True
    ).as_matrix()
    offset_translation = np.asarray([offset_dict['X'], offset_dict['Y'], offset_dict['Z']]).reshape(3, 1)

    transform = construct_transform_matrix(offset_rotation, offset_translation)

    base_rotation = Rotation.from_quat(
        [base_pose.orientation.x, base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w]
    ).as_matrix()
    base_translation = np.asarray([base_pose.position.x, base_pose.position.y, base_pose.position.z]).reshape(3, 1)

    base = construct_transform_matrix(base_rotation, base_translation)

    offset_transform = np.matmul(base, transform)

    return construct_pose(offset_transform)
