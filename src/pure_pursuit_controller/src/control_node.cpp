#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class PurePursuitNode : public rclcpp::Node
{
public:
  PurePursuitNode() : Node("pure_pursuit_node")
  {
    // 1. SUBSCRIBER: Nghe vị trí từ /odom
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10, std::bind(&PurePursuitNode::odom_callback, this, _1));

    // 2. PUBLISHER: Gửi lệnh điều khiển xuống /ackermann_cmd
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/ackermann_cmd", 10);

    // Tạo timer chạy 20Hz (gửi lệnh liên tục)
    timer_ = this->create_wall_timer(
      50ms, std::bind(&PurePursuitNode::timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "Role 2 Node Started: Ready to control Ackermann Car!");
  }

private:
  // Xử lý khi nhận được tin nhắn Odom
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    current_speed_ = msg->twist.twist.linear.x;
    // Sau này sẽ lấy tọa độ x, y ở đây để đưa cho thuật toán
  }

  // Hàm chạy liên tục để gửi lệnh điều khiển
  void timer_callback()
  {
    auto msg = ackermann_msgs::msg::AckermannDriveStamped();

    // --- GIẢ LẬP THUẬT TOÁN (TESTING) ---
    // đợi hàm của Hòa
    msg.drive.speed = 1.0;          // Chạy 1 m/s
    msg.drive.steering_angle = 0.5; // Bẻ lái 0.5 rad (khoảng 28 độ)

    // Gửi lệnh đi
    publisher_->publish(msg);

    // In ra màn hình để kiểm tra
    // RCLCPP_INFO(this->get_logger(), "Publishing: Speed=%.2f, Steer=%.2f", msg.drive.speed, msg.drive.steering_angle);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  double current_speed_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}


