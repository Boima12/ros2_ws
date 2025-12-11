#include <chrono>
#include <memory>
#include <vector>
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "pure_pursuit_math.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PurePursuitNode : public rclcpp::Node
{
public:
  PurePursuitNode() : Node("pure_pursuit_node"), current_progress_(0)
  {
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry", 10, std::bind(&PurePursuitNode::odom_callback, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    timer_ = this->create_wall_timer(50ms, std::bind(&PurePursuitNode::timer_callback, this));

    // Dynamically resolve waypoints file path from package share directory
    auto package_share_dir = ament_index_cpp::get_package_share_directory("pure_pursuit_controller");
    std::string file_path = package_share_dir + "/src/waypoints.txt";
    load_waypoints_from_file(file_path);

    RCLCPP_INFO(this->get_logger(), "Pure Pursuit node khởi động! Chờ odometry...");
  }

private:
  void load_waypoints_from_file(const std::string& filename) {
      std::ifstream file(filename);
      if (!file.is_open()) {
          RCLCPP_ERROR(this->get_logger(), "KHÔNG MỞ ĐƯỢC FILE: %s", filename.c_str());
          return;
      }
      double x, y;
      path_to_follow_.clear();
      while (file >> x >> y) {
          path_to_follow_.push_back({x, y});
      }
      file.close();
      RCLCPP_INFO(this->get_logger(), "ĐÃ NẠP %zu WAYPOINTS – SẴN SÀNG CHẠY HÌNH SỐ 8!", path_to_follow_.size());
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    
    double qx = msg->pose.pose.orientation.x;
    double qy = msg->pose.pose.orientation.y;
    double qz = msg->pose.pose.orientation.z;
    double qw = msg->pose.pose.orientation.w;
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    current_yaw_ = std::atan2(siny_cosp, cosy_cosp);

    // Lazily set initial progress to the closest waypoint once odom is available
    if (!progress_initialized_ && !path_to_follow_.empty()) {
      double best_dist = std::numeric_limits<double>::max();
      size_t best_idx = 0;
      for (size_t i = 0; i < path_to_follow_.size(); ++i) {
        double d = std::hypot(path_to_follow_[i].x - current_x_, path_to_follow_[i].y - current_y_);
        if (d < best_dist) {
          best_dist = d;
          best_idx = i;
        }
      }
      current_progress_ = best_idx;
      progress_initialized_ = true;
    }
  }

  void timer_callback()
  {
    if (path_to_follow_.empty()) return;

    double target_speed = 0.8;

    double steer = algorithm_.calculate_steering(
        current_x_, current_y_, current_yaw_, target_speed, path_to_follow_, current_progress_);

    if (std::isnan(steer)) steer = 0.0;
    double final_steer = steer;

    // RESET VÒNG KHI ĐÃ QUA 90% PATH VÀ GẦN (0,0)
    if (current_progress_ > path_to_follow_.size() * 0.9 &&
        std::hypot(current_x_, current_y_) < 1.8) {
        current_progress_ = 0;
        RCLCPP_INFO(this->get_logger(), "HOÀN THÀNH 1 VÒNG HÌNH SỐ 8! RESET PROGRESS → 0");
    }

    // Debug
    static int count = 0;
    if (++count % 100 == 0) {
        RCLCPP_INFO(this->get_logger(), "Progress: %zu/%zu | Steer: %.3f → %.3f | Pos: (%.2f, %.2f)",
                    current_progress_, path_to_follow_.size(), steer, final_steer, current_x_, current_y_);
    }

    // Convert steering angle to angular velocity for differential drive
    // Use a much larger gain factor since steering angles are small (typically < 0.3 rad)
    // but we need significant angular velocity for turning
    double angular_vel = final_steer * 3.0;  // Amplify steering command significantly
    
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = target_speed;
    msg.angular.z = angular_vel;
    publisher_->publish(msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  PurePursuit algorithm_;
  std::vector<Point> path_to_follow_;
  size_t current_progress_ = 0;
  bool progress_initialized_ = false;
  
  double current_x_ = 0.0;
  double current_y_ = 0.0;
  double current_yaw_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}
