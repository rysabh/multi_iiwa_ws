import rclpy
from rclpy.node import Node
import numpy as np
from diffusion_interface.srv import DiffusionAction  # Adjust this import according to your service definition
from diffusion_interface.msg import Observation
from mocap_optitrack_interfaces.srv import GetMotionCaptureData
from ati_sensor_interfaces.srv import GetForceTorque

class DiffusionClient(Node):

    def __init__(self):
        super().__init__('diffusion_client')
        
        self.client_diffusion = self.create_client(DiffusionAction, 'predicts_action')

        while not self.client_diffusion.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service diffusion not available, waiting...')


        self.client_mocap = self.create_client(GetMotionCaptureData, 'get_mocap_data')
        
        while not self.client_mocap.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service mocap not available, waiting again...')
            
            
        self.client_force = self.create_client(GetForceTorque, 'get_force_torque')
        
        while not self.client_force.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service not available, waiting again...')

        
        self.request = DiffusionAction.Request()
        
    def get_mocap_data(self):
        request = GetMotionCaptureData.Request()
        future = self.client_mocap.call_async(request)
        print("Sent request to mocap service")
        print(future)
        return future
    
    def get_force_torque(self):
        request = GetForceTorque.Request()
        future = self.client_force.call_async(request)
        print("Sent request to ati service")
        print(future)
        return future
    
    def send_observation(self, observations):
        self.request.observations = observations
        future = self.client_diffusion.call_async(self.request)
        return future


def main(args=None):
    rclpy.init(args=args)

    diffusion_client = DiffusionClient()
    
    state_observations = diffusion_client.get_mocap_data()
    rclpy.spin_until_future_complete(diffusion_client, state_observations)
    
    force_observations = diffusion_client.get_force_torque()
    rclpy.spin_until_future_complete(diffusion_client, force_observations)
    
    
    if state_observations.done() and force_observations.done():
        try:
            # Wait until the mocap service call completes
            response_mocap = state_observations.result()
            diffusion_client.get_logger().info(f'Mocap Service call completed')

            response_ati = force_observations.result()
            diffusion_client.get_logger().info(f"Ati Service call successful")


            # Send the observation to the diffusion service
            new_test = []
            new_ob = Observation()
            # print(new_ob)
            new_ob.capture_data =  response_mocap.latest_message
            # print("############################################")
            
            # print(response_ati.msg.fx)
            new_ob.ati_data = response_ati.msg
            print(new_ob.ati_data)
            # print("############################################")
            
            # new_test.append(new_ob)
            new_test.append(new_ob)
            print("New test: ", len(new_test))
            # print("New test: ", new_test)
            print("############################################")
            actions = diffusion_client.send_observation(new_test)

            # Wait until the diffusion service call completes
            rclpy.spin_until_future_complete(diffusion_client, actions)
            
            if actions.done():
                diffusion_client.get_logger().info(f'Response: {actions.result().actions}')
                print("Received data: ", actions.result().actions)
                # for value in actions.result().actions:
                #     chisel = value.data[0:6]
                #     print("Chisel: ", chisel)
                #     gripper = value.data[6:12]
                #     print("Gripper: ", gripper)
        
        except KeyboardInterrupt:
            pass
        
        finally:
            diffusion_client.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()