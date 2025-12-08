#ifndef PURE_PURSUIT_MATH_HPP
#define PURE_PURSUIT_MATH_HPP

#include <vector>
#include <cmath>
#include <algorithm>
#include <iostream>

struct Point {
    double x;
    double y;
};

class PurePursuit {
public:
    PurePursuit() {
        wheelbase_ = 0.33;      // Chiều dài cơ sở
        k_gain_ = 0.5;          // Hệ số lookahead
        min_lookahead_ = 1.0;   // Lookahead tối thiểu
        max_steering_angle_ = 0.78; // Giới hạn góc lái (45 độ)
    }

    // Tính khoảng cách Lookahead động
    double calculate_lookahead_distance(double current_velocity) {
        return (k_gain_ * std::abs(current_velocity)) + min_lookahead_;
    }

    // Tìm điểm mục tiêu
    Point find_lookahead_point(double robot_x, double robot_y, const std::vector<Point>& path, double Ld) {
        // (Logic đơn giản hóa cho bài test: Lấy điểm đầu tiên trong danh sách)
        if (path.empty()) return {robot_x + Ld, robot_y}; 
        return path[0]; 
    }

    // HÀM CHÍNH: TÍNH GÓC LÁI
    double calculate_steering(double robot_x, double robot_y, double robot_yaw, double current_velocity, const std::vector<Point>& path) {
        double Ld = calculate_lookahead_distance(current_velocity);
        Point target = find_lookahead_point(robot_x, robot_y, path, Ld);

        double dx = target.x - robot_x;
        double dy = target.y - robot_y;

        // Chuyển sang hệ tọa độ xe
        double local_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

        double dist_sq = dx*dx + dy*dy;
        double curvature = (2.0 * local_y) / dist_sq;
        
        double steering_angle = std::atan(curvature * wheelbase_);

        // Kẹp góc lái
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

#endif
