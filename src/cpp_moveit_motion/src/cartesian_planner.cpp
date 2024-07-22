#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/move
#include <fstream>
#include <vector>
#include <string>

class CartesianPlanner : public rclcpp::Node
{
public:
    CartesianPlanner() : Node("cartesian_planner")
    {
        this->declare_parameter<std::string>("csv_path", "");
        init();
    }

private:
    void init()
    {
        std::string csv_path;
        this->get_parameter("csv_path", csv_path);
        std::vector<geometry_msgs::msg::Pose> waypoints = loadWaypointsFromCSV(csv_path);

        moveit::planning_interface::MoveGroupInterface move_group(this, "manipulator");  // Update with your move group
        move_group.setPlanningTime(10.0);

        // Cartesian Paths
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double jump_threshold = 0.0;
        const double eef_step = 0.01;
        double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
        RCLCPP_INFO(this->get_logger(), "Cartesian path (%.2f%% achieved)", fraction * 100.0);

        // Execute the plan
        move_group.execute(trajectory);
    }

    std::vector<geometry_msgs::msg::Pose> loadWaypointsFromCSV(const std::string &file_path)
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;
        std::ifstream file(file_path);
        std::string line;
        while (std::getline(file, line))
        {
            std::stringstream lineStream(line);
            std::string cell;
            std::vector<double> data;
            while (std::getline(lineStream, cell, ','))
            {
                data.push_back(std::stod(cell));
            }
            geometry_msgs::msg::Pose pose;
            pose.position.x = data[0];
            pose.position.y = data[1];
            pose.position.z = data[2];
            pose.orientation.w = 1.0;  // Example: Simple orientation
            waypoints.push_back(pose);
        }
        return waypoints;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CartesianPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
