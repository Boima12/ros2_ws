#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/twist.hpp"

// Gọi file header (đang nằm cùng thư mục src)
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

    // 2. Publisher: Gửi TWIST vào topic /ackermann_cmd (Cho khớp với lệnh Bridge của bạn)
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/ackermann_cmd", 10);

    // 3. Timer 20Hz
    timer_ = this->create_wall_timer(
      50ms, std::bind(&PurePursuitNode::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "Control Node Started: Sending Twist to /ackermann_cmd");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Cập nhật vận tốc (nếu cần dùng sau này)
    (void)msg;
  }

  void timer_callback()
  {
    // --- BƯỚC 1: TẠO DỮ LIỆU ĐƯỜNG DẪN GIẢ ---
    std::vector<Point> path_to_follow;
    // Điểm mục tiêu: x=5.0, y=0.0 (Chạy thẳng)
    path_to_follow.push_back({5.0, 0.0}); 

    // --- BƯỚC 2: TÍNH TOÁN ---
    double target_speed = 5.0; // Tốc độ đặt 5 m/s

    // Gọi thuật toán từ file header
    // Giả sử xe đang ở (0,0) hướng 0
    double steer = algorithm_.calculate_steering(0.0, 0.0, 0.0, target_speed, path_to_follow);

    // --- BƯỚC 3: GỬI LỆNH (DẠNG TWIST) ---
    auto msg = geometry_msgs::msg::Twist();
    
    msg.linear.x = target_speed;  // Tốc độ
    msg.angular.z = steer;        // Góc lái
    
    publisher_->publish(msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  PurePursuit algorithm_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}
