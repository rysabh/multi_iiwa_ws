import rclpy
from rclpy.node import Node
from mocap_optitrack_interfaces.msg import RigidBodyArray
from std_msgs.msg import String
from moveit_motion.diffusion_policy_cam.submodules import robomath as rm
from moveit_motion.diffusion_policy_cam.submodules import robomath_addon as rma
from math import pi

class KukaPosePub(Node):
    def __init__(self):
        super().__init__('kuka_pose_pub')
        self.publisher_ = self.create_publisher(String, 'kuka_blue_pose', 10)

        self.subscription = self.create_subscription(RigidBodyArray, 'mocap_rigid_bodies', self.callback, 10)

    def callback(self, msg):
        for body in msg.rigid_bodies:
            if body.id == 16:
                pose_stamped = body.pose_stamped

                pose_list = [
                    pose_stamped.pose.position.x,
                    pose_stamped.pose.position.y,
                    pose_stamped.pose.position.z,
                    pose_stamped.pose.orientation.w,
                    pose_stamped.pose.orientation.x,
                    pose_stamped.pose.orientation.y,
                    pose_stamped.pose.orientation.z
                ]

                try:
                    Base_W_TxyzQwxyz = rma.motive_2_robodk_rigidbody(pose_list)
                    Base_W_TxyzQwxyz = [i * 1000 for i in Base_W_TxyzQwxyz[:3]] + Base_W_TxyzQwxyz[3:]
                    Base_W_TxyzRxyz = rm.Pose_2_KUKA(rma.TxyzQwxyz_2_Pose(Base_W_TxyzQwxyz))

                    pose_msg = String()
                    pose_msg.data = (
                        f"x = {Base_W_TxyzRxyz[0]}\n"
                        f"y = {Base_W_TxyzRxyz[1]}\n"
                        f"z = {Base_W_TxyzRxyz[2]}\n"
                        f"rx = {Base_W_TxyzRxyz[5]}\n"
                        f"ry = {Base_W_TxyzRxyz[4]}\n"
                        f"rz = {Base_W_TxyzRxyz[3]}"
                    )

                    self.publisher_.publish(pose_msg)

                    self.get_logger().info(f"Published pose: \n{pose_msg.data}")

                except Exception as e:
                    self.get_logger().error(f"Error processing pose: {e}")

def main(args=None):
    rclpy.init(args=args)
    kuka_pose_pub = KukaPosePub()
    rclpy.spin(kuka_pose_pub)
    kuka_pose_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
