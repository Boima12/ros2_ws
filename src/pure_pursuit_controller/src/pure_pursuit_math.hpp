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
    // Constructor: Cài đặt các tham số mặc định
    PurePursuit() {
        // Cấu hình dựa trên xe Ackermann (Hỏi lại Hội thông số Wheelbase chính xác nhé)
        wheelbase_ = 0.33;      // L: Chiều dài cơ sở (khoảng cách 2 trục bánh)
        k_gain_ = 0.5;          // k: Hệ số độ nhạy theo vận tốc
        min_lookahead_ = 1.0;   // L_min: Khoảng cách nhìn tối thiểu
        max_steering_angle_ = 0.78; // Giới hạn góc lái (45 độ)
    }

    // --- CÔNG THỨC 1: TÍNH LOOKAHEAD DISTANCE ĐỘNG ---
    // Ld = k * v + Lmin
    double calculate_lookahead_distance(double current_velocity) {
        // Đảm bảo vận tốc luôn dương để tính toán
        double speed = std::abs(current_velocity);
        return (k_gain_ * speed) + min_lookahead_;
    }

    // --- HÀM TÌM ĐIỂM MỤC TIÊU (Lookahead Point) ---
    // Tìm điểm trên đường dẫn cách xe một khoảng Ld
    Point find_lookahead_point(double robot_x, double robot_y, 
                               const std::vector<Point>& path, double Ld) {
        if (path.empty()) return {robot_x, robot_y};

        // Thuật toán đơn giản: Tìm điểm đầu tiên nằm ngoài bán kính Ld
        for (const auto& p : path) {
            double dx = p.x - robot_x;
            double dy = p.y - robot_y;
            double dist = std::sqrt(dx*dx + dy*dy);

            if (dist >= Ld) {
                return p; // Đây là điểm mục tiêu
            }
        }
        return path.back(); // Nếu đi hết đường mà không thấy thì lấy điểm cuối
    }

    // --- CÔNG THỨC 2 & 3: TÍNH GÓC LÁI ---
    double calculate_steering(double robot_x, double robot_y, double robot_yaw, 
                              double current_velocity, const std::vector<Point>& path) {
        
        // Bước 1: Tính khoảng cách Lookahead (Công thức của Hòa)
        double Ld = calculate_lookahead_distance(current_velocity);

        // Bước 2: Tìm điểm mục tiêu thực tế trên đường dẫn
        Point target = find_lookahead_point(robot_x, robot_y, path, Ld);

        // Bước 3: Chuyển đổi tọa độ điểm mục tiêu sang hệ quy chiếu của Xe (Vehicle Frame)
        double dx = target.x - robot_x;
        double dy = target.y - robot_y;

        // Công thức quay tọa độ (Rotation Matrix) để tìm local_y
        // local_y chính là khoảng cách ngang từ mũi xe đến điểm mục tiêu
        double local_y = -std::sin(robot_yaw) * dx + std::cos(robot_yaw) * dy;

        // Bước 4: Tính độ cong Curvature (Công thức của Hòa: k = 2y / Ld^2)
        // Lưu ý: Ld ở mẫu số phải là khoảng cách thực tế đến điểm đó
        double dist_sq = dx*dx + dy*dy; // Ld^2 thực tế
        double curvature = (2.0 * local_y) / dist_sq;

        // Bước 5: Tính góc lái (Công thức của Hòa: delta = atan(k * L))
        double steering_angle = std::atan(curvature * wheelbase_);

        // Kẹp giới hạn góc lái (Safety)
        if (steering_angle > max_steering_angle_) steering_angle = max_steering_angle_;
        if (steering_angle < -max_steering_angle_) steering_angle = -max_steering_angle_;

        return steering_angle;
    }

private:
    double wheelbase_;          // L
    double k_gain_;             // k
    double min_lookahead_;      // Lmin
    double max_steering_angle_;
};

#endif
