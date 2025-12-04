#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "pure_pursuit_math.hpp" // <--- QUAN TRỌNG: Gọi thư viện của Hòa

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

    // 2. Publisher Lệnh điều khiển
    publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
      "/ackermann_cmd", 10);

    // 3. Timer chạy 20Hz
    timer_ = this->create_wall_timer(
      50ms, std::bind(&PurePursuitNode::timer_callback, this));
      
    RCLCPP_INFO(this->get_logger(), "Role 2 Node Started: Ready to control Ackermann Car!");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Cập nhật vị trí xe (Hiện tại chưa dùng tới biến này trong bài test giả lập)
  current_speed_ = msg->twist.twist.linear.x;
  }
  void timer_callback()
  {
    // Khai báo biến chứa đường đi sẽ dùng để tính toán
    std::vector<Point> path_to_follow;
    // --- LOGIC CHỌN ĐƯỜNG ĐI ---
    // Nếu biến global_path_ (lưu đường thật) bị rỗng -> Dùng đường giả để test

    // TẠM THỜI: giữ nguyên dummy path

    // Khi nào có Planner thật mình sửa sau.
    path_to_follow.push_back({2.0, 0.5}); 
    // --- GỌI THUẬT TOÁN CỦA HÒA ---

    double steer = algorithm_.calculate_steering(0.0, 0.0, 0.0, 0.0, path_to_follow);
    // --- GỬI LỆNH ---

    auto msg = ackermann_msgs::msg::AckermannDriveStamped();

    msg.drive.speed = 1.0;
    msg.drive.steering_angle = steer;
    publisher_->publish(msg);
    // In ra màn hình (Log) - Mình comment lại để đỡ rác màn hình, khi nào cần debug thì mở ra

    // RCLCPP_INFO(this->get_logger(), "Steer: %.2f", steer);

  } 

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  PurePursuit algorithm_; // Đối tượng thuật toán
  double current_speed_ = 0.0;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PurePursuitNode>());
  rclcpp::shutdown();
  return 0;
}

