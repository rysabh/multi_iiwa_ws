import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
import pandas as pd
import tf_transformations
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster

class RvizLink(Node):

    def __init__(self):
        super().__init__('waypoint_follower')
        
        # self.data = pd.read_csv('src/test.csv')
        self.data = pd.read_csv('src/moveit_motion/moveit_motion/diffusion_policy_cam/diffusion_pipline/data_chisel_task/cleaned_traj/cap_010_cleaned.csv')
        self.index = 0
        self.br = TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 100)
        self.timer = self.create_timer(1/10, self.timer_callback)  # Set timer interval to 1/30 seconds

    def timer_callback(self) -> None:
        """
        Timer callback to publish transforms for each waypoint at defined intervals.
        """
        
        current_time = self.get_clock().now().to_msg()

        t1 = TransformStamped()
        t1.header.stamp = current_time
        t1.header.frame_id = 'world'
        t1.child_frame_id = 'kuka_blue_link_0'
        t1.transform.translation = Vector3(x=-0.5, y=-0.5, z=0.009)
        t1.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        t2 = TransformStamped()
        t2.header.stamp = current_time
        t2.header.frame_id = 'world'
        t2.child_frame_id = 'kuka_green_link_0'
        t2.transform.translation = Vector3(x=-0.5, y=0.35, z=0.014)  
        t2.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)

        self.br.sendTransform(t1)
        self.br.sendTransform(t2)

        if self.index < len(self.data):
        # Extract data for the current index
            x1, y1, z1, qx1, qy1, qz1, qw1 = self.data.iloc[self.index, [0, 1, 2, 3, 4, 5, 6]]
            x2, y2, z2, qx2, qy2, qz2, qw2 = self.data.iloc[self.index, [7, 8, 9, 10, 11, 12, 13]]
            # Publish marker at current waypoint
            self.publish_marker('world', [x1, y1, z1], [qx1, qy1, qz1, qw1], current_time, 
                                marker_id=self.index, 
                                marker_type=Marker.SPHERE, 
                                scale=(0.01, 0.01, 0.01),  # Adjusted scale for visibility
                                color=(1.0, 0.0, 0.0, 1.0), 
                                namespace=f'marker{self.index}_1')
            
            self.publish_marker('world', [x2, y2, z2], [qx2, qy2, qz2, qw2], current_time, 
                                marker_id=self.index, 
                                marker_type=Marker.SPHERE, 
                                scale=(0.01, 0.01, 0.01),  # Adjusted scale for visibility
                                color=(1.0, 0.0, 0.0, 1.0), 
                                namespace=f'marker{self.index}_2')

            # Increment the index for the next call
            self.index += 1


    def publish_marker(self, frame_id, translation, rotation, current_time, 
        marker_id=0, marker_type=Marker.CUBE, scale=(0.3, 0.3, 0.3), 
        color=(0.0, 1.0, 0.0, 1.0), namespace='rigid_body'):
        """
        Publishes a marker to the ROS network.
        NOTE: Requires adding marker in rviz and setting the topic to visualization_marker

        Parameters:
        frame_id (str): The frame ID in which the marker is defined.
        translation (List[float]): The translation of the marker [x, y, z].
        rotation (List[float]): The rotation of the marker [x, y, z, w].
        current_time: The current time stamp for the marker.
        marker_id (int): The ID of the marker.
        marker_type (int): The type of the marker (e.g., Marker.CUBE, Marker.SPHERE).
        scale (Tuple[float, float, float]): The scale of the marker [x, y, z].
        color (Tuple[float, float, float, float]): The color and alpha of the marker [r, g, b, a].
        namespace (str): The namespace of the marker.
        """

        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = current_time
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position.x = translation[0]
        marker.pose.position.y = translation[1]
        marker.pose.position.z = translation[2]
        marker.pose.orientation.x = rotation[0]
        marker.pose.orientation.y = rotation[1]
        marker.pose.orientation.z = rotation[2]
        marker.pose.orientation.w = rotation[3]
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        self.get_logger().info(f'Publishing marker: {marker}')
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    way_point_follower = WaypointFollower()
    rclpy.spin(way_point_follower)
    way_point_follower.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()