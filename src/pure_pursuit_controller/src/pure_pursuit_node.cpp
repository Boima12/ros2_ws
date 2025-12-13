#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <ament_index_cpp/get_package_share_directory.hpp>

struct Point {
    double x;
    double y;
};

class PurePursuitNode : public rclcpp::Node {
public:
    PurePursuitNode() : Node("pure_pursuit_node") {
        // Parameters
        this->declare_parameter("lookahead_distance", 1.5);
        this->declare_parameter("target_speed", 1.0);
        
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        target_speed_ = this->get_parameter("target_speed").as_double();

        // Load Waypoints
        std::string pkg_path = ament_index_cpp::get_package_share_directory("pure_pursuit_controller");
        std::string file_path = pkg_path + "/waypoints.txt";
        loadWaypoints(file_path);

        // Pub/Sub
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Pure Pursuit Node Started. Loaded %zu waypoints.", path_.size());
    }

private:
    void loadWaypoints(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open waypoints file: %s", filename.c_str());
            return;
        }
        double x, y;
        char comma;
        while (file >> x >> comma >> y) {
            path_.push_back({x, y});
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (path_.empty()) return;

        // 1. Get current vehicle state
        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;

        // Get Yaw from Quaternion
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, current_yaw;
        m.getRPY(roll, pitch, current_yaw);

        // 2. Find closest point on path to determine where to start looking
        size_t closest_index = 0;
        double min_dist = 1e9;
        for (size_t i = 0; i < path_.size(); ++i) {
            double dist = std::hypot(path_[i].x - current_x, path_[i].y - current_y);
            if (dist < min_dist) {
                min_dist = dist;
                closest_index = i;
            }
        }

        // 3. Find Lookahead Point
        Point target_point = path_[closest_index];
        for (size_t i = closest_index; i < path_.size(); ++i) {
            double dist = std::hypot(path_[i].x - current_x, path_[i].y - current_y);
            if (dist >= lookahead_distance_) {
                target_point = path_[i];
                break;
            }
        }

        // 4. Calculate curvature / steering angle
        double alpha = atan2(target_point.y - current_y, target_point.x - current_x) - current_yaw;
        double steering_angle = atan(2.0 * 0.3 * sin(alpha) / lookahead_distance_); // 0.3 is wheelbase

        // 5. Publish Command
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = target_speed_;
        cmd.angular.z = steering_angle; // For ackermann sim, we often map angular.z to steering angle
        
        publisher_->publish(cmd);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    std::vector<Point> path_;
    double lookahead_distance_;
    double target_speed_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitNode>());
    rclcpp::shutdown();
    return 0;
}