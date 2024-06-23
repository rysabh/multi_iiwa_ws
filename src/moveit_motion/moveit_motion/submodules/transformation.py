import numpy as np 
from scipy.spatial.transform import Rotation 
import geometry_msgs.msg 
from geometry_msgs.msg import Pose
import math


def create_pose(px, py, pz, ox, oy, oz, ow):
    """ Helper function to create a Pose object """
    pose = geometry_msgs.msg.Pose()
    pose.position.x = px
    pose.position.y = py
    pose.position.z = pz
    pose.orientation.x = ox
    pose.orientation.y = oy
    pose.orientation.z = oz
    pose.orientation.w = ow
    return pose

def R2q(R): 
    # convert rotation matrix to quaternion (scalar first convention)
    Rmat = Rotation.from_matrix(R) 
    q_scalarlast = Rotation.as_quat(Rmat) 
    q = np.empty(4)
    q[0] = q_scalarlast[3] 
    q[1] = q_scalarlast[0]
    q[2] = q_scalarlast[1]
    q[3] = q_scalarlast[2] 
    return q 

def p2T(p): 
    # convert pose to homogeneous transformation matrix 
    x = p[0] 
    y = p[1]
    z = p[2] 
    q = p[3:7] 
    R = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix() # note: rotation library uses scalar last convention 
    T = np.array([[ R[0][0], R[0][1], R[0][2], x],
                   [R[1][0], R[1][1], R[1][2], y],
                   [R[2][0], R[2][1], R[2][2], z],
                  [0, 0, 0, 1]])
    return T 

def T2p(T): 
    R = T[0:3,0:3] 
    q = R2q(R) 
    r = T[0:3,3] 
    p = np.empty(7)
    p[0:3] = r 
    p[3:7] = q  
    return p 

def R2ABC(matrix):
    # convert rotation matrix to A,B,C angles with ZYX order convention 
    sy = np.sqrt(matrix[0, 0] * matrix[0, 0] + matrix[1, 0] * matrix[1, 0])
    
    singular = sy < 1e-6
    
    if not singular:
        theta_x = np.arctan2(matrix[2, 1], matrix[2, 2]) 
        theta_y = np.arctan2(-matrix[2, 0], sy)
        theta_z = np.arctan2(matrix[1, 0], matrix[0, 0])
    else:
        theta_x = np.arctan2(-matrix[1, 2], matrix[1, 1])
        theta_y = np.arctan2(-matrix[2, 0], sy)
        theta_z = 0.0
        
    return np.array([theta_z, theta_y, theta_x]) * 180/np.pi 

def ABC2R(A:np.float64,B:np.float64,C:np.float64): 
    # convert A,B,C angles to rotation matrix using ZYX order convention 
    theta_z = math.radians(A)
    theta_y = math.radians(B)
    theta_x = math.radians(C)
    Rz = np.array([[np.cos(theta_z), -np.sin(theta_z), 0.],
                   [np.sin(theta_z), np.cos(theta_z), 0.],
                   [0., 0., 1.]], dtype=np.float64)
    
    Ry = np.array([[np.cos(theta_y), 0., np.sin(theta_y)],
                   [0., 1., 0.],
                   [-np.sin(theta_y), 0., np.cos(theta_y)]], dtype=np.float64)
    
    Rx = np.array([[1., 0., 0.],
                   [0., np.cos(theta_x), -np.sin(theta_x)],
                   [0., np.sin(theta_x), np.cos(theta_x)]], dtype=np.float64)
    R = np.dot(Rz, np.dot(Ry, Rx)) 
    return R

def p2xa(p): 
    # convert pose to vector of position in mm and A,B,C angles in deg  
    xa = np.empty(6) 
    xa[0] = p[0] * 1000
    xa[1] = p[1] * 1000 
    xa[2] = p[2] * 1000 
    q = p[3:7] 
    R = Rotation.from_quat([q[1], q[2], q[3], q[0]]).as_matrix() # note: rotation library uses scalar last convention 
    ABC = R2ABC(R) 
    xa[3] = ABC[0]
    xa[4] = ABC[1]
    xa[5] = ABC[2] 
    return xa 

def xa2p(xa):
    '''
    convert vector of position in mm and ABC angles in degrees to pose (x,y,z,qw,qx,qy,qz) 
    '''
    p = np.empty(7)
    p[0] = xa[0] / 1000 
    p[1] = xa[1] / 1000 
    p[2] = xa[2] / 1000 
    R = ABC2R(xa[3], xa[4], xa[5]) 
    Rmat = Rotation.from_matrix(R) 
    q = Rotation.as_quat(Rmat) # outputted as scalar last 
    p[3] = q[3]  
    p[4] = q[0]
    p[5] = q[1] 
    p[6] = q[2]    
    return p 

def invT(T): 
    '''
    inverse of homogeneous transformation matrix 
    '''
    R = T[0:3,0:3]
    p = T[0:3,3]
    RT = np.transpose(R) 
    invp = -RT @ p
    invT = np.array([[RT[0,0], RT[0,1], RT[0,2], invp[0]],
                     [RT[1,0], RT[1,1], RT[1,2], invp[1]],
                     [RT[2,0], RT[2,1], RT[2,2], invp[2]], 
                     [0.,0.,0.,1.]])
    return invT


# ------ Transformation ------ # 

def p_to_T(pose):
    '''
    convert pose [x,y,z,qw,qx,qy,qz] to transformation matrix [[R,r],[0,0,0,1]] 
    '''
    r = np.asarray(pose[0:3]).reshape(3,1) 
    q = pose[3:7] 
    R = Rotation.from_quat([q[1],q[2],q[3],q[0]]).as_matrix() # scipy quat convention is [qx,qy,qz,qw]
    T = np.concatenate((R, r),axis=1) 
    T = np.concatenate((T, [[0,0,0,1]]),axis=0)  
    return T  

def T_to_p(T):
    '''
    convert transformation matrix [[R,r],[0,0,0,1]] to pose [x,y,z,qw,qx,qy,qz] 
    '''
    R = T[0:3,0:3]
    r = T[0:3,3] 
    qrev = Rotation.from_matrix(R).as_quat() 
    q = np.array([qrev[3], qrev[0], qrev[1], qrev[2]])
    pose = np.concatenate((r,q),axis=0) 
    return pose 

def pest_to_pabove(pest, d): 
    '''
    output new pose which is distance d above (along local Z) input pose in frame of the input pose 
    '''
    T_Hh = p_to_T(pest) 
    T_hA = np.array([[1,0,0,0], [0,1,0,0], [0,0,1,-d], [0,0,0,1]]) 
    T_HA = np.matmul(T_Hh, T_hA) 
    p_above = T_to_p(T_HA) 
    return p_above 


def pose_to_nparray(pose: Pose):
    """
    Parameters:
        pose (Pose): The Pose message from geometry_msgs.

    Returns:
        numpy.ndarray: A NumPy array with the structure [x, y, z, qw, qx, qy, qz].
    """
    return np.array([
        pose.position.x, pose.position.y, pose.position.z,
        pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z
    ])


def nparray_to_pose(array):
    """
    Parameters:
        array (numpy.ndarray): A NumPy array with the structure [x, y, z, qw, qx, qy, qz].
    
    Returns:
        Pose: The Pose message from geometry_msgs.
    """
    pose = Pose()
    pose.position.x = array[0]
    pose.position.y = array[1]
    pose.position.z = array[2]
    pose.orientation.w = array[3]
    pose.orientation.x = array[4]
    pose.orientation.y = array[5]
    pose.orientation.z = array[6]
    return pose

def calc_target_pose(calib_pose, offset):  # TODO(abhay) : check this function

    calib_pose_np = pose_to_nparray(calib_pose)  # {EEF POSE | ROBOT BASE FRAME} - PERFECTLY ALIGNED
    # ----------------------- #
    offset_ROT_MAT = ABC2R(A=offset['A'], B=offset['B'], C=offset['C'])
    offset_QUAT = R2q(offset_ROT_MAT)  # Follows scalar first convention
    offset_pose_np = [offset['X'], offset['Y'], offset['Z'], offset_QUAT[0], offset_QUAT[1], offset_QUAT[2], offset_QUAT[3]] 
    # ----------------------- #
    calib_pose_T = p_to_T(calib_pose_np)  # {EEF POSE | ROBOT BASE FRAME} - PERFECTLY ALIGNED
    offset_pose_T = p_to_T(offset_pose_np)  # Homogenous matrix representing the offset

    target_pose_T_peg = np.matmul(calib_pose_T, offset_pose_T)
    target_pose_T_peg_np = T_to_p(target_pose_T_peg)
    target_pose_msg = nparray_to_pose(target_pose_T_peg_np)

    return target_pose_msg

def calc_attempt_pose(target_pose, d):

    # NOTE : Local Z translation from the "estiamted/attempted" target pose

    target_pose_np = pose_to_nparray(target_pose)
    
    attempt_pose_np = pest_to_pabove(target_pose_np, d)
    
    attempt_pose_msg = nparray_to_pose(attempt_pose_np)

    return attempt_pose_msg