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
        // Tunable parameters
        this->declare_parameter("lookahead_distance", 2.0); // Increased to 2.0 for smoother steering
        this->declare_parameter("target_speed", 1.5);       
        
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        target_speed_ = this->get_parameter("target_speed").as_double();

        // Load Waypoints
        std::string pkg_path = ament_index_cpp::get_package_share_directory("pure_pursuit_controller");
        std::string file_path = pkg_path + "/waypoints.txt";
        loadWaypoints(file_path);

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
        while (file >> x) {
            if (file.peek() == ',') file.ignore();
            file >> y;
            path_.push_back({x, y});
        }
    }

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (path_.empty()) return;

        double current_x = msg->pose.pose.position.x;
        double current_y = msg->pose.pose.position.y;

        // Convert Quaternion to Yaw
        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, current_yaw;
        m.getRPY(roll, pitch, current_yaw);

        // 1. Find the index of the closest point on the path
        size_t closest_index = 0;
        double min_dist = 1e9;
        for (size_t i = 0; i < path_.size(); ++i) {
            double dist = std::hypot(path_[i].x - current_x, path_[i].y - current_y);
            if (dist < min_dist) {
                min_dist = dist;
                closest_index = i;
            }
        }

        // 2. Find Lookahead Point (Search forward from closest, wrapping around)
        Point target_point = path_[closest_index];
        for (size_t i = 0; i < path_.size(); ++i) {
            size_t idx = (closest_index + i) % path_.size(); // Modulo handles the loop
            double dist = std::hypot(path_[idx].x - current_x, path_[idx].y - current_y);
            if (dist >= lookahead_distance_) {
                target_point = path_[idx];
                break;
            }
        }

        // 3. Calculate Steering Angle
        double dx = target_point.x - current_x;
        double dy = target_point.y - current_y;
        double alpha = atan2(dy, dx) - current_yaw;

        // --- FIX: Normalize Angle to [-pi, pi] ---
        while (alpha > M_PI) alpha -= 2.0 * M_PI;
        while (alpha < -M_PI) alpha += 2.0 * M_PI;

        // Pure Pursuit Formula
        // Wheelbase L = 0.3 (approx)
        double steering_angle = atan(2.0 * 0.3 * sin(alpha) / lookahead_distance_); 

        // 4. Debug Print (Only every 20th message to reduce spam)
        static int debug_counter = 0;
        if (debug_counter++ % 20 == 0) {
            RCLCPP_INFO(this->get_logger(), 
                "Pos: (%.2f, %.2f) Yaw: %.2f | Target: (%.2f, %.2f) | Steer: %.2f",
                current_x, current_y, current_yaw, target_point.x, target_point.y, steering_angle);
        }

        // 5. Publish Command
        auto cmd = geometry_msgs::msg::Twist();
        cmd.linear.x = target_speed_;
        cmd.angular.z = steering_angle;
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