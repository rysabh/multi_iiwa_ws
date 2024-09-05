import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory, PlanningScene, PlanningSceneWorld
from std_msgs.msg import String

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('remapping_node')
        # self.publisher_blue = self.create_publisher(JointState, 'joint_states', 10)
        self.publisher_green = self.create_publisher(JointState, 'joint_states', 10)
        # self.publisher_kuka_blue_display_planned_path = self.create_publisher(DisplayTrajectory, 'display_planned_path', 10)
        self.publisher_kuka_green_display_planned_path = self.create_publisher(DisplayTrajectory, 'display_planned_path', 10)
        # self.publisher_kuka_blue_monitored_planning_scene = self.create_publisher(PlanningScene, 'monitored_planning_scene', 10)
        self.publisher_kuka_green_monitored_planning_scene = self.create_publisher(PlanningScene, 'monitored_planning_scene', 10)
        # self.publisher_kuka_blue_planning_scene = self.create_publisher(PlanningScene, 'planning_scene', 10)
        self.publisher_kuka_green_planning_scene = self.create_publisher(PlanningScene, 'planning_scene', 10)
        # self.publisher_kuka_blue_planning_scene_world = self.create_publisher(PlanningSceneWorld, 'planning_scene_world', 10)
        self.publisher_kuka_green_planning_scene_world = self.create_publisher(PlanningSceneWorld, 'planning_scene_world', 10)

        # # # self.subscription_blue = self.create_subscription(JointState, 'kuka_blue/joint_states', self.kuka_blue_joint_state_callback, 10)
        self.subscription_green = self.create_subscription(JointState, 'kuka_green/joint_states', self.kuka_green_joint_state_callback, 10)
        # # self.subscription_display_planned_path = self.create_subscription(DisplayTrajectory, 'kuka_blue/display_planned_path', self.kuka_blue_display_planned_path_callback, 10)
        self.subscription_display_planned_path = self.create_subscription(DisplayTrajectory, 'kuka_green/display_planned_path', self.kuka_green_display_planned_path_callback, 10)
        # # self.subscription_monitored_planning_scene = self.create_subscription(PlanningScene, 'kuka_blue/monitored_planning_scene', self.kuka_blue_monitored_planning_scene_callback, 10)
        self.subscription_monitored_planning_scene = self.create_subscription(PlanningScene, 'kuka_green/monitored_planning_scene', self.kuka_green_monitored_planning_scene_callback, 10)
        # # self.subscription_planning_scene = self.create_subscription(PlanningScene, 'kuka_blue/planning_scene', self.kuka_blue_planning_scene_callback, 10)      
        self.subscription_planning_scene = self.create_subscription(PlanningScene, 'kuka_green/planning_scene', self.kuka_green_planning_scene_callback, 10)      
        # self.subscription_planning_scene_world = self.create_subscription(PlanningSceneWorld, 'kuka_blue/planning_scene_world', self.kuka_blue_planning_scene_world_callback, 10)
        self.subscription_planning_scene_world = self.create_subscription(PlanningSceneWorld, 'kuka_green/planning_scene_world', self.kuka_green_planning_scene_world_callback, 10)


    def kuka_blue_joint_state_callback(self, msg):
        joint_state = JointState()
        joint_state.name = [f"kuka_blue_{name}" for name in msg.name]
        joint_state.position = msg.position
        joint_state.header = msg.header
        joint_state.velocity = msg.velocity
        joint_state.effort = msg.effort

        self.publisher_blue.publish(joint_state)

    def kuka_green_joint_state_callback(self, msg):
        joint_state = JointState()
        joint_state.name = [f"kuka_green_{name}" for name in msg.name]
        joint_state.header = msg.header
        joint_state.position = msg.position
        joint_state.velocity = msg.velocity
        joint_state.effort = msg.effort

        self.publisher_green.publish(joint_state)

    def kuka_blue_display_planned_path_callback(self, msg):
        display_planned_path = DisplayTrajectory()
        display_planned_path.trajectory = msg.trajectory
        display_planned_path.trajectory_start = msg.trajectory_start
        display_planned_path.model_id = msg.model_id

        self.publisher_kuka_blue_display_planned_path.publish(display_planned_path)

    def kuka_green_display_planned_path_callback(self, msg):
        display_planned_path = DisplayTrajectory()
        display_planned_path.trajectory = msg.trajectory
        display_planned_path.trajectory_start = msg.trajectory_start
        display_planned_path.model_id = msg.model_id

        self.publisher_kuka_green_display_planned_path.publish(display_planned_path)

    def kuka_blue_monitored_planning_scene_callback(self, msg):
        monitored_planning_scene = PlanningScene()
        monitored_planning_scene.allowed_collision_matrix = msg.allowed_collision_matrix
        monitored_planning_scene.fixed_frame_transforms = msg.fixed_frame_transforms
        monitored_planning_scene.is_diff = msg.is_diff
        monitored_planning_scene.link_padding = msg.link_padding
        monitored_planning_scene.link_scale = msg.link_scale
        monitored_planning_scene.name = msg.name
        monitored_planning_scene.object_colors = msg.object_colors
        monitored_planning_scene.robot_model_name = msg.robot_model_name
        monitored_planning_scene.robot_state = msg.robot_state
        monitored_planning_scene.world = msg.world

        self.publisher_kuka_blue_monitored_planning_scene.publish(monitored_planning_scene)

    def kuka_green_monitored_planning_scene_callback(self, msg):
        monitored_planning_scene = PlanningScene()
        monitored_planning_scene.allowed_collision_matrix = msg.allowed_collision_matrix
        monitored_planning_scene.fixed_frame_transforms = msg.fixed_frame_transforms
        monitored_planning_scene.is_diff = msg.is_diff
        monitored_planning_scene.link_padding = msg.link_padding
        monitored_planning_scene.link_scale = msg.link_scale
        monitored_planning_scene.name = msg.name
        monitored_planning_scene.object_colors = msg.object_colors
        monitored_planning_scene.robot_model_name = msg.robot_model_name
        monitored_planning_scene.robot_state = msg.robot_state
        monitored_planning_scene.world = msg.world

        self.publisher_kuka_green_monitored_planning_scene.publish(monitored_planning_scene)

    def kuka_blue_planning_scene_callback(self, msg):
        planning_scene = PlanningScene()
        planning_scene.allowed_collision_matrix = msg.allowed_collision_matrix
        planning_scene.fixed_frame_transforms = msg.fixed_frame_transforms
        planning_scene.is_diff = msg.is_diff
        planning_scene.link_padding = msg.link_padding
        planning_scene.link_scale = msg.link_scale
        planning_scene.name = msg.name
        planning_scene.object_colors = msg.object_colors
        planning_scene.robot_model_name = msg.robot_model_name
        planning_scene.robot_state = msg.robot_state
        planning_scene.world = msg.world

        self.publisher_kuka_blue_planning_scene.publish(planning_scene)

    def kuka_green_planning_scene_callback(self, msg):
        planning_scene = PlanningScene()
        planning_scene.allowed_collision_matrix = msg.allowed_collision_matrix
        planning_scene.fixed_frame_transforms = msg.fixed_frame_transforms
        planning_scene.is_diff = msg.is_diff
        planning_scene.link_padding = msg.link_padding
        planning_scene.link_scale = msg.link_scale
        planning_scene.name = msg.name
        planning_scene.object_colors = msg.object_colors
        planning_scene.robot_model_name = msg.robot_model_name
        planning_scene.robot_state = msg.robot_state
        planning_scene.world = msg.world

        self.publisher_kuka_green_planning_scene.publish(planning_scene)

    def kuka_blue_planning_scene_world_callback(self, msg):
        planning_scene_world = PlanningSceneWorld()
        planning_scene_world.collision_objects = msg.collision_objects
        planning_scene_world.octomap = msg.octomap
        
        self.publisher_kuka_blue_planning_scene_world.publish(planning_scene_world)

    def kuka_green_planning_scene_world_callback(self, msg):
        planning_scene_world = PlanningSceneWorld()
        planning_scene_world.collision_objects = msg.collision_objects
        planning_scene_world.octomap = msg.octomap
        
        self.publisher_kuka_green_planning_scene_world.publish(planning_scene_world)


def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()