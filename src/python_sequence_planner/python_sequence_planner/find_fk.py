import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from moveit_msgs.srv import GetPositionFK
import pandas as pd

class FKClient(Node):
    def __init__(self):
        super().__init__('fk_client')
        self.client = self.create_client(GetPositionFK, 'compute_fk')
        self.joint_names = ["kuka_blue_A1", "kuka_blue_A2", "kuka_blue_A3", "kuka_blue_A4", "kuka_blue_A5", "kuka_blue_A6", "kuka_blue_A7"]

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for FK service to become available...")

    def get_fk(self, joint_values, link_name):
        request = GetPositionFK.Request()
        request.robot_state.joint_state = JointState()
        request.robot_state.joint_state.name = self.joint_names
        request.robot_state.joint_state.position = joint_values
        request.fk_link_names = [link_name]

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            pose = future.result().pose_stamped[0].pose
            return pose
        else:
            raise RuntimeError("Failed to get FK result")

    def process_csv(self, csv_file, link_name, output_csv):
        data = pd.read_csv(csv_file)
        results = []
        for index, row in data.iterrows():
            joint_values = row.tolist()
            pose = self.get_fk(joint_values, link_name)
            pose_data = {
                'position_x': pose.position.x,
                'position_y': pose.position.y,
                'position_z': pose.position.z,
                'orientation_x': pose.orientation.x,
                'orientation_y': pose.orientation.y,
                'orientation_z': pose.orientation.z,
                'orientation_w': pose.orientation.w
            }
            results.append(pose_data)
            self.get_logger().info(f"Pose for row {index}: {pose}")
        results_df = pd.DataFrame(results)
        results_df.to_csv(output_csv, index=False)

def main(args=None):
    rclpy.init(args=args)
    fk_client = FKClient()
    try:
        csv_file = '/home/battery/Downloads/ik_results_1_degrees.csv'
        output_csv = '/home/battery/Downloads/fk_results.csv'
        link_name = 'kuka_blue_link_ee'
        fk_client.process_csv(csv_file, link_name, output_csv)
        print("All poses have been calculated and saved to CSV file.")
    except Exception as e:
        print(f"An error occurred: {str(e)}")
    finally:
        fk_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()