import rclpy
from rclpy.node import Node
from diffusion_interface.srv import DiffusionAction  # Adjust this import according to your service definition
from mocap_optitrack_interfaces.srv import GetMotionCaptureData

class DiffusionClient(Node):

    def __init__(self):
        super().__init__('diffusion_client')
        self.client_diffusion = self.create_client(DiffusionAction, 'predicts_action')

        while not self.client_diffusion.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service diffusion not available, waiting...')


        self.client_mocap = self.create_client(GetMotionCaptureData, 'get_mocap_data')
        while not self.client_mocap.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('service mocap not available, waiting again...')
            
        
        self.request = DiffusionAction.Request()
        
    def get_mocap_data(self):
        request = GetMotionCaptureData.Request()
        future = self.client_mocap.call_async(request)
        print("Sent request to mocap service")
        print(future)
        return future
    
    def send_observation(self, observations):
        self.request.observations = observations
        future = self.client_diffusion.call_async(self.request)
        return future


def main(args=None):
    rclpy.init(args=args)

    diffusion_client = DiffusionClient()
    
    observations = diffusion_client.get_mocap_data()
    
    rclpy.spin_until_future_complete(diffusion_client, observations)
    
    if observations.done():
        try:
            # Wait until the mocap service call completes
            response_mocap = observations.result()
            diffusion_client.get_logger().info(f'Mocap Service call completed')

            # Send the observation to the diffusion service
            actions = diffusion_client.send_observation(response_mocap.latest_message)

            # Wait until the diffusion service call completes
            rclpy.spin_until_future_complete(diffusion_client, actions)
            
            if actions.done():
                diffusion_client.get_logger().info(f'Response: {actions.result().actions}')
                print("Received data: ", actions.result().actions)
        
        except KeyboardInterrupt:
            pass
        
        finally:
            diffusion_client.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()