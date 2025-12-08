#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"    
#include "geometry_msgs/msg/twist.hpp"
#include "pure_pursuit_math.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PurePursuitNode : public rclcpp::Node
{
public:
  PurePursuitNode() : Node("pure_pursuit_node")
  {
    // 1. Subscriber Odom
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&PurePursuitNode::odom_callback, this, _1));

    // 2. Publisher Lệnh điều khiển (Dùng Twist)
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/ackermann_cmd", 10);

    // 3. Timer chạy 20Hz
    timer_ = this->create_wall_timer(
      50ms, std::bind(&PurePursuitNode::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "Role 2 Node Started: Using TWIST message!");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_speed_ = msg->twist.twist.linear.x;
  }

  void timer_callback()
  {
    // --- BƯỚC 1: TẠO DỮ LIỆU GIẢ ---
    std::vector<Point> path_to_follow;
    
    // Nếu chưa nhận được đường dẫn thật (biến latest_path_ rỗng) thì tạo điểm giả
    if (latest_path_.poses.empty()) {
        path_to_follow.push_back({2.0, 0.5}); // Điểm giả để test
    } else {
        // Nếu có đường dẫn thật thì sau này sẽ convert ở đây (tạm thời để trống)
    }

    // --- BƯỚC 2: GỌI THUẬT TOÁN ---
    double steer = algorithm_.calculate_steering(0.0, 0.0, 0.0, 0.0, path_to_follow);

    // --- BƯỚC 3: GỬI LỆNH (DẠNG TWIST) ---
    auto msg = geometry_msgs::msg::Twist();
    
    // linear.x = Vận tốc (m/s)
    // angular.z = Góc lái (rad) - Plugin Ackermann sẽ hiểu cái này
    msg.linear.x = 1.0;  
    msg.angular.z = steer; 
    
    publisher_->publish(msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  // Biến lưu đường dẫn
  nav_msgs::msg::Path latest_path_;
  
  PurePursuit algorithm_;
  double current_speed_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}
