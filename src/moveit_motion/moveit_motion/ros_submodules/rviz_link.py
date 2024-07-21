import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped, Quaternion, Vector3
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from tf2_ros import TransformBroadcaster
from nav_msgs.msg import Path

class RvizLink(Node):
    def __init__(self):
        super().__init__('rviz_link')
        self.br = TransformBroadcaster(self)
        self.index = 0
        self.callback_function = None

    
    def start_simulation(self, interval: float = 1/10) -> None:
        if self.callback_function is None:
            raise ValueError("Callback function is not defined.")
        
        self.timer = self.create_timer(interval, self.callback_function)

    def pub_marker(self, namespace, frame_id, translation, rotation, 
                    marker_id=0, marker_type=Marker.CUBE, scale=(0.3, 0.3, 0.3), 
                    color=(0.0, 1.0, 0.0, 1.0)):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = namespace
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.position = Vector3(x=translation[0], y=translation[1], z=translation[2])
        marker.pose.orientation = Quaternion(x=rotation[0], y=rotation[1], z=rotation[2], w=rotation[3])
        marker.scale = Vector3(x=scale[0], y=scale[1], z=scale[2])
        marker.color = ColorRGBA(r=color[0], g=color[1], b=color[2], a=color[3])
        self.get_logger().info(f'Publishing marker: {marker}')
        self.marker_pub.publish(marker)
    
    def pub_markers(self, markers: list[list], frame_id: str, 
                    marker_type=Marker.SPHERE, scale=(0.01,0.01,0.01), color=(1.0, 0.0, 0.0, 1.0)) -> None:
        
        for index, TxyzQwxyz in enumerate(markers):
            Txyz = TxyzQwxyz[0:3]
            Qxyzw = [0.0, 0.0, 0.0, 1.0]
            if len(TxyzQwxyz) == 7:
                Qxyzw = TxyzQwxyz[3:]
                
            
        def _callback_function(self):
            self.pub_marker(frame_id = frame_id, namespace='m', translation=Txyz, rotation=Qxyzw,
                            marker_id=index, marker_type=marker_type, scale=scale, color=color)
        
        self.callback_function = _callback_function

    # def pub_path(self, frame_id: str, path: list[list]) -> None:
    #     path_msg = Path()
    #     path_msg.header.stamp = self.get_clock().now().to_msg()
    #     path_msg.header.frame_id = frame_id

    #     for index, TxyzQwxyz in enumerate(path):
    #         pose = PoseStamped()
    #         pose.header.stamp = current_time
    #         pose.header.frame_id = frame_id
    #         pose.pose.position = Point(x=TxyzQwxyz[0], y=TxyzQwxyz[1], z=TxyzQwxyz[2])
    #         pose.pose.orientation = Quaternion(x=TxyzQwxyz[3], y=TxyzQwxyz[4], z=TxyzQwxyz[5], w=TxyzQwxyz[6])
    #         path_msg.poses.append(pose)
    #     self.path_pub.publish(path_msg)

    def pub_frame(self, parent_frame: str, child_frame: str, translation: list, rotation: list) -> None:
        """
        Publishes a transform to the ROS network.

        Parameters:
        parent_frame (str): The parent frame ID in the transform tree.
        child_frame (str): The child frame ID in the transform tree.
        translation (List[float]): The translation of the child frame relative to the parent frame as [x, y, z].
        rotation (List[float]): The rotation of the child frame relative to the parent frame as [x, y, z, w].
        current_time: The current time stamp to set on the transform header.
        """
        _t = TransformStamped()
        _t.header.stamp = self.get_clock().now().to_msg()
        _t.header.frame_id = parent_frame
        _t.child_frame_id = child_frame

        _t.transform.translation.x = translation[0]
        _t.transform.translation.y = translation[1]
        _t.transform.translation.z = translation[2]

        _t.transform.rotation.x = rotation[0]
        _t.transform.rotation.y = rotation[1]
        _t.transform.rotation.z = rotation[2]
        _t.transform.rotation.w = rotation[3]
        self.br.sendTransform(_t)

    def pub_robot(self, parent_frame:str, base_link: str, links: list, translations: list, rotations: list) -> None:
        _t = TransformStamped()
        _t.header.stamp = self.get_clock().now().to_msg()
        _t.header.frame_id = parent_frame
        _t.child_frame_id = base_link
        _t.transform.translation = Vector3(x=-0.5, y=-0.5, z=0.009)
        _t.transform.rotation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.br.sendTransform(_t)

    def create_path_publisher(self, topic: str = "path") -> None:
        self.path_pub = self.create_publisher(Path, topic, 10)

    def create_marker_publisher(self):
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

