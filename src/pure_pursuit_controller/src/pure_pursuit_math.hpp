#pragma once
#include <cmath>
#include <vector>
#include <algorithm>

// CÁC STRUCT VÀ CLASS CỦA THUẬT TOÁN ĐIỀU KHIỂN

struct Point {
    double x;
    double y;
};

class PurePursuit {
public:
    PurePursuit() {
        // Cấu hình thông số xe Ackermann
        wheelbase_ = 0.33;      
        k_gain_ = 0.5;          
        min_lookahead_ = 1.0;   
        max_steering_angle_ = 0.78; // ~45 độ
    }

    double calculate_lookahead_distance(double current_velocity) {
        double speed = std::abs(current_velocity);
        return (k_gain_ * speed) + min_lookahead_;
    }

    Point find_lookahead_point(double robot_x, double robot_y, const std::vector<Point>& path, double Ld) {
        (void)robot_x; (void)robot_y; (void)Ld; 
        if (path.empty()) return {0.0, 0.0};
        // Trong phiên bản test, chúng ta chỉ lấy điểm đầu tiên trong path làm mục tiêu
        return path[0]; 
    }

    double calculate_steering(double robot_x, double robot_y, double robot_yaw, double current_velocity, const std::vector<Point>& path) {
        double Ld = calculate_lookahead_distance(current_velocity);
        Point target = find_lookahead_point(robot_x, robot_y, path, Ld);

        double dx = target.x - robot_x;
        double dy = target.y - robot_y;
        
        // Chuyển đổi sang hệ tọa độ của xe (robot_yaw)
        double local_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

        double dist_sq = dx*dx + dy*dy;
        // Công thức tính độ cong (curvature)
        double curvature = (2.0 * local_y) / dist_sq;
        
        // Công thức tính góc lái (Steering Angle)
        double steering_angle = std::atan(curvature * wheelbase_);

        // Kẹp góc lái trong giới hạn
        if (steering_angle > max_steering_angle_) steering_angle = max_steering_angle_;
        if (steering_angle < -max_steering_angle_) steering_angle = -max_steering_angle_;

        return steering_angle;
    }

private:
    double wheelbase_;
    double k_gain_;
    double min_lookahead_;
    double max_steering_angle_;
};
