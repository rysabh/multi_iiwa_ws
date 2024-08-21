#include <rclcpp/rclcpp.hpp>
#include <moveit_msgs/action/move_group_sequence.hpp>
#include <moveit_msgs/msg/motion_sequence_request.hpp>
#include <moveit_msgs/msg/motion_sequence_item.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/kinematic_constraints/utils.h>
#include <rclcpp_action/rclcpp_action.hpp>

class MotionSequenceNode : public rclcpp::Node {
public:
    MotionSequenceNode() : Node("motion_sequence_node") {
        // Define the planning group for Kuka robot
        const std::string PLANNING_GROUP = "kuka_green";

        // Create MoveGroupSequence action client
        using MoveGroupSequence = moveit_msgs::action::MoveGroupSequence;
        using GoalHandleMoveGroupSequence = rclcpp_action::ClientGoalHandle<MoveGroupSequence>;
        auto client = rclcpp_action::create_client<MoveGroupSequence>(this, "/sequence_move_group");

        // Wait for the MoveGroupSequence action server
        if (!client->wait_for_action_server(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Action server /sequence_move_group not available after waiting");
            rclcpp::shutdown();
            return;
        }

        // Create MotionSequenceRequest
        moveit_msgs::msg::MotionSequenceRequest sequence_request;

        // Define a MotionSequenceItem
        moveit_msgs::msg::MotionSequenceItem item;
        item.blend_radius = 0.1;  // Define blending radius for smooth transitions

        item.req.group_name = PLANNING_GROUP;
        item.req.planner_id = "PTP";
        item.req.allowed_planning_time = 5;
        item.req.max_velocity_scaling_factor = 1.0;
        item.req.max_acceleration_scaling_factor = 0.5;

        // First point target position
        geometry_msgs::msg::PoseStamped target_pose;
        target_pose.header.frame_id = "kuka_green_base_link"; // Adjust the frame ID according to your setup
        target_pose.pose.position.x = 0.28; // Adjust positions according to Kuka's reachable workspace
        target_pose.pose.position.y = 0;
        target_pose.pose.position.z = 0.5;
        target_pose.pose.orientation.w = 1.0;

        // Use helper function to generate goal constraints
        item.req.goal_constraints.resize(1);
        item.req.goal_constraints[0] = kinematic_constraints::constructGoalConstraints("kuka_green_link_ee", target_pose); // Adjust link names according to your Kuka configuration

        // Add the item to the sequence
        sequence_request.items.push_back(item);

        // Create action goal
        auto goal_msg = MoveGroupSequence::Goal();
        goal_msg.request = sequence_request;

        // Configure planning options
        goal_msg.planning_options.planning_scene_diff.is_diff = true;
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = true;

        // Define callbacks for the action client
        auto send_goal_options = rclcpp_action::Client<MoveGroupSequence>::SendGoalOptions();
        
        // Corrected goal response callback
        send_goal_options.goal_response_callback = [this](std::shared_ptr<GoalHandleMoveGroupSequence> goal_handle) {
            if (!goal_handle) {
                RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
            } else {
                RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
            }
        };

        send_goal_options.result_callback = [this](const GoalHandleMoveGroupSequence::WrappedResult & result) {
            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Action succeeded!");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Action failed!");
            }
        };

        // Send action goal
        auto goal_handle_future = client->async_send_goal(goal_msg, send_goal_options);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MotionSequenceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
