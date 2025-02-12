import rclpy
from rclpy.node import Node
import numpy as np
from diffusion_interface.srv import DiffusionAction 
from diffusion_interface.msg import PredictedAction
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import sys
import os
import torch
parent_dir = os.path.abspath(os.path.join('/home/cam/multi_iiwa_ws/src/moveit_motion/moveit_motion/diffusion_policy_cam', '..'))
print(parent_dir)
sys.path.append(parent_dir)


# Set Matplotlib backend
import matplotlib
matplotlib.use('TkAgg')

from moveit_motion.diffusion_policy_cam.submodules import robomath_addon as rma
from moveit_motion.diffusion_policy_cam.submodules import robomath as rm

from moveit_motion.diffusion_policy_cam.diffusion_jupyternotebook import live_infrence_evaluation as live


class DiffusionService(Node):

    def __init__(self, rigid_bodies, action_bodies, obs_bodies, unlabbled_marker,
                        labbled_markers, statistics, obs_horizon,
                        pred_horizon, action_horizon, action_dim, 
                        noise_scheduler, num_diffusion_iters,
                        noise_pred_net, device):
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
        self.obs_horizon = obs_horizon
        self.pred_horizon = pred_horizon
        self.action_horizon = action_horizon
        self.action_dim = action_dim
        self.statistics = statistics
        self.noise_scheduler = noise_scheduler
        self.num_diffusion_iters = num_diffusion_iters
        self.noise_pred_net = noise_pred_net
        self.device = device

        
    def extract_rigid_bodies_and_marker_sets(self, mocap_data):
        # rigid_bodies = []
        # marker_sets = []
        rigid_body_info = {}
        marker_set_info = {}

        # Extract rigid bodies
        for rb in mocap_data.rigid_bodies:
            euler_pose = [
                    rb.pose_stamped.pose.position.x,
                    rb.pose_stamped.pose.position.y,
                    rb.pose_stamped.pose.position.z,
                    rb.pose_stamped.pose.orientation.w,
                    rb.pose_stamped.pose.orientation.x,
                    rb.pose_stamped.pose.orientation.y,
                    rb.pose_stamped.pose.orientation.z
                ]
            
            rigid_body_info[rb.id] = euler_pose
            # rigid_bodies.append(rigid_body_info)

        # Extract marker sets
        for ms in mocap_data.marker_sets:
            self.get_logger().info(f"Marker Set: {type(ms.id)}")
            marker_set_info[ms.id] = [
                    ms.position.x,
                    ms.position.y,
                    ms.position.z
                ]
            # marker_sets.append(marker_set_info)

        return rigid_body_info, marker_set_info
    
    def extract_force_positions(self, force_data):
        force_positions = []
        self.get_logger().info(f"Force data: {force_data}")
        x = force_data.fx
        y = force_data.fy
        z = force_data.fz
        
        # # Compute f = sqrt(x*x + y*y + z*z)
        # f = np.sqrt(x**2 + y**2 + z**2)
        
        # # Normalize x, y, z
        # x_f = x / f
        # y_f = y / f
        # z_f = z / f
        
        
        force_positions.append(x)
        force_positions.append(y)
        force_positions.append(z)
        # force_positions.append(f)
        
        return force_positions

    def gripper_state(self, rigid_body_pose, marker_pose):

        new_pose_wrt_gripper = rma.Vxyz_wrt_TxyzRxyz(np.array(marker_pose, dtype=float), np.array(rigid_body_pose, dtype=float))
        distance = rm.distance(new_pose_wrt_gripper, [0, 0, 0])
        
        if distance > 30:
            on_off_state = 1
        else:
            on_off_state = -1
             
        
        return on_off_state


    def bodies_and_markers(self, rigid_bodies, marker_sets):
        observation = []
        
        for rb in self.rigid_bodies:

                
            rigid_body = rma.motive_2_robodk_rigidbody(np.array(rigid_bodies[rb], dtype=float))
            # Multiply the first three elements by 1000
            rigid_body = [val * 1000 for val in rigid_body[:3]] + rigid_body[3:]
            rigid_bodies[rb] = rma.TxyzQwxyz_2_TxyzRxyz(rigid_body)
    
            observation.extend(rigid_bodies[rb])
            
        # for rb in self.obs_bodies:
        #     # print("RB - ", rb)
        #     # print("Rigid Body: ", rigid_bodies.keys())
        #     # print("Marker: ", marker_sets.keys())
        #     # print("#######################################")
        #     # print("#######################################")
        #     # print("#######################################")
        #     # print("Ridig Body: ", rb, rigid_bodies[rb])
        #     observation.extend([self.gripper_state(rigid_bodies[rb], marker_sets[self.unlabbled_marker])])
        
        for ms in self.labbled_markers:
            marker = rma.motive_2_robodk_marker(np.array(marker_sets[int(ms)], dtype=float))
            
            # Multiply the first three elements by 1000
            marker_sets[ms] = [val * 1000 for val in marker]
            
            observation.extend(marker_sets[ms])
            
        # print("Observation: ", observation)
            
        return observation


    def predicted_action_callback(self, request, response):
        
        all_observations = []

        # Iterate over all observations in the request
        for observation in request.observations:
            rigid_bodies, marker_sets = self.extract_rigid_bodies_and_marker_sets(observation.capture_data)
            state_observation = self.bodies_and_markers(rigid_bodies, marker_sets)
            force_observation = self.extract_force_positions(observation.ati_data)
            all_observations.append(state_observation)
            
            
        self.get_logger().info(f'Got observations --- state obs - {len(state_observation)}')
        self.get_logger().info(f'Got observations --- force obs - {len(force_observation)}')
        # self.get_logger().info(f'Got observations --- Total - {len(result)}')

        
        actions = live._pred_traj(all_observations, self.statistics, self.obs_horizon,
                        self.pred_horizon, self.action_horizon, self.action_dim, 
                        self.noise_scheduler, self.num_diffusion_iters,
                        self.noise_pred_net, self.device)
        
        
        # actions = [[0.21505458652973175, 0.20627030730247498, 0.23463064432144165, -1.4847859896086955, 0.5324402244345928, 2.99450595146179, -1.0]]
        action_list = []
        
        for i in range(len(actions)):
            msg = PredictedAction()
            # print("Action: ", actions[i], type(actions[i]))
            # msg.data = actions[i].tolist()
            # self.get_logger().info(f"Actions:  {actions[i]}")
            msg.data = actions[i].astype(float).tolist()
            action_list.append(msg)
            
        response.actions = action_list

        return response
    


def main(args=None):
    rclpy.init(args=args)

    # Specify the initial values for the attributes
    rigid_bodies = [13, 7, 17]  # Example IDs for rigid bodies
    action_bodies = [13, 7]     # Example IDs for action bodies
    obs_bodies = [17]       # Example IDs for observed bodies
    unlabbled_marker =  0000 # Example ID for the unlabeled marker

    edge_1 =  ['10', '14', '7', '6']
    edge_2 = ['6', '3', '26', '40']
    edge_3 = ['40', '39', '37', '36']
    edge_4 = ['10', '19', '29', '36']

    ###### in Diffusion service calss inintilzation below change the edge lable whoch ever edge you are performing the infrance on #######



    # labbled_markers = [40, 39, 37, 36] # Example IDs for labeled markers

    # checkpoint_path = 'no-sync/chkpts/checkpoint_training_34_obs_1_action_30_FX_epoch_199.pth'
    checkpoint_path = 'no-sync/chkpts/state/checkpoint_training_38_obs_1_action_30_epoch_199.pth'
    

    checkpoint = torch.load(checkpoint_path)

    # Parameters corrsponding to
    num_epochs =checkpoint['num_epochs']
    obs_dim = checkpoint['obs_dim']
    action_dim = checkpoint['action_dim']
    # parameters
    pred_horizon = checkpoint['pred_horizon']
    obs_horizon = checkpoint['obs_horizon']
    action_horizon = checkpoint['action_horizon']

    statistics = checkpoint['dataset_stats']
    len_dataloader = checkpoint['len_dataloader']   
    num_diffusion_iters = checkpoint['num_diffusion_iters']
    
    noise_pred_net, noise_scheduler, device, ema, optimizer, lr_scheduler = live._model_initalization(action_dim, obs_dim, 
                                                                                                      obs_horizon, pred_horizon, num_epochs, len_dataloader, 
                                                                                                      num_diffusion_iters)
    
    noise_pred_net.load_state_dict(checkpoint['model_state_dict'])
    optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
    lr_scheduler.load_state_dict(checkpoint['scheduler_state_dict'])
    ema.load_state_dict(checkpoint['ema_state_dict'])

    # Instantiate the DiffusionService node with the specific values
    diffusion_service = DiffusionService(rigid_bodies, action_bodies, obs_bodies, unlabbled_marker, edge_4, 
                                         statistics, obs_horizon, pred_horizon, action_horizon, action_dim,
                                         noise_scheduler, num_diffusion_iters, noise_pred_net, device)

    # Keep the node alive to handle incoming requests
    rclpy.spin(diffusion_service)

    # Cleanup once the node is stopped
    diffusion_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()