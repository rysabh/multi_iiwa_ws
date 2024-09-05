import rclpy
from rclpy.node import Node
import numpy as np
from diffusion_interface.srv import DiffusionAction 
from diffusion_interface.msg import PredictedAction

import sys
import os
parent_dir = os.path.abspath(os.path.join('/home/cam/Documents/raj/diffusion_policy_cam', '..'))
print(parent_dir)
sys.path.append(parent_dir)

from diffusion_policy_cam.submodules import robomath_addon as rma
from diffusion_policy_cam.submodules import robomath as rm
from diffusion_policy_cam.diffusion_jupyternotebook.live_infrence_evaluation import main


class DiffusionService(Node):

    def __init__(self, rigid_bodies, action_bodies, obs_bodies, unlabbled_marker, labbled_markers):
        super().__init__('diffusion_service')
        self.srv = self.create_service(DiffusionAction, 
                                       'predicts_action', 
                                       self.predicted_action_callback)
        
        # Initialize the attributes
        self.rigid_bodies = rigid_bodies
        self.action_bodies = action_bodies
        self.obs_bodies = obs_bodies
        self.unlabbled_marker = unlabbled_marker
        self.labbled_markers = labbled_markers

    def extract_rigid_bodies_and_marker_sets(self, mocap_data):
        rigid_bodies = []
        marker_sets = []
        rigid_body_info = {}
        marker_set_info = {}

        # Extract rigid bodies
        for rb in mocap_data.rigid_bodies:
            euler_pose = rma.TxyzQwxyz_2_TxyzRxyz([
                    rb.pose_stamped.pose.position.x,
                    rb.pose_stamped.pose.position.y,
                    rb.pose_stamped.pose.position.z,
                    rb.pose_stamped.pose.orientation.w,
                    rb.pose_stamped.pose.orientation.x,
                    rb.pose_stamped.pose.orientation.y,
                    rb.pose_stamped.pose.orientation.z
                ])
            
            rigid_body_info[rb.id] = euler_pose
            # rigid_bodies.append(rigid_body_info)

        # Extract marker sets
        for ms in mocap_data.marker_sets:
            marker_set_info[ms.id] = [
                    ms.position.x,
                    ms.position.y,
                    ms.position.z
                ]
            # marker_sets.append(marker_set_info)

        return rigid_body_info, marker_set_info

    def gripper_state(self, rigid_body_pose, marker_pose):
        # print("#######################################")
        # print("#######################################")
        # print("#######################################")
        # print("Rigid Body Pose: ", np.array(rigid_body_pose, dtype=float))
        # print(len(rigid_body_pose))
        # print("Marker Pose: ", np.array(marker_pose, dtype=float))
        # print(len(marker_pose))
        # print("#######################################")
        # print("#######################################")
        # print("#######################################")
        new_pose_wrt_gripper = rma.Vxyz_wrt_TxyzRxyz(np.array(marker_pose, dtype=float), np.array(rigid_body_pose, dtype=float))
        # print("New Pose wrt Gripper: ", new_pose_wrt_gripper)
        # print("#######################################")
        # print("#######################################")
        # print("#######################################")
        distance = rm.distance(new_pose_wrt_gripper, [0, 0, 0])
        
        if distance > 30:
            on_off_state = 1
        else:
            on_off_state = -1
            
        # print("Distance: ", distance)
        # print("On Off State: ", on_off_state)           
        
        return on_off_state

    def bodies_and_markers(self, rigid_bodies, marker_sets):
        observation = []
        
        for rb in self.action_bodies:
            # print("RB - ", rb)
            # print("Rigid Body: ", rigid_bodies.keys())
            # print("#######################################")
            # print("#######################################")
            # print("#######################################")
            # print("Ridig Body: ", rb, rigid_bodies[rb])
            observation.extend(rigid_bodies[rb])
            
        for rb in self.obs_bodies:
            # print("RB - ", rb)
            # print("Rigid Body: ", rigid_bodies.keys())
            # print("Marker: ", marker_sets.keys())
            # print("#######################################")
            # print("#######################################")
            # print("#######################################")
            # print("Ridig Body: ", rb, rigid_bodies[rb])
            observation.extend([self.gripper_state(rigid_bodies[rb], marker_sets[self.unlabbled_marker])])
        
        for ms in self.labbled_markers:
            # print("MS - ", ms)
            # print("Marker: ", marker_sets.keys())
            # print("#######################################")
            # print("#######################################")
            # print("#######################################")
            # print("Marker: ", marker_sets[ms])
            observation.extend(marker_sets[ms])
            
        print("Observation: ", observation)
            
        return observation

    def predicted_action_callback(self, request, response):
            
        rigid_bodies, marker_sets = self.extract_rigid_bodies_and_marker_sets(request.observations)
        observation = self.bodies_and_markers(rigid_bodies, marker_sets)
        actions = main(observation)
        observation_slice = observation[:7]

        # # Convert all values to float
        # float_observation_slice = [float(value) for value in observation_slice]
        # actions = []
        # for i in range(8):
        #     float64_array_msg = PredictedAction()
            
        #     # Flatten the list by removing the extra brackets
        #     float64_array_msg.data = float_observation_slice
            
        #     actions.append(float64_array_msg)
        
        # response.actions = actions
        action_list = []
        
        for i in range(len(actions)):
            msg = PredictedAction()
            msg.data = actions[i]
            action_list.append(msg)
            
        response.actions = action_list

        return response
    



def main(args=None):
    rclpy.init(args=args)

    # Specify the initial values for the attributes
    rigid_bodies = [2, 6]  # Example IDs for rigid bodies
    action_bodies = [6]     # Example IDs for action bodies
    obs_bodies = [2]       # Example IDs for observed bodies
    unlabbled_marker = 1510 # Example ID for the unlabeled marker
    labbled_markers = [131075, 393221] # Example IDs for labeled markers

    # Instantiate the DiffusionService node with the specific values
    diffusion_service = DiffusionService(rigid_bodies, action_bodies, obs_bodies, unlabbled_marker, labbled_markers)

    # Keep the node alive to handle incoming requests
    rclpy.spin(diffusion_service)

    # Cleanup once the node is stopped
    diffusion_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()