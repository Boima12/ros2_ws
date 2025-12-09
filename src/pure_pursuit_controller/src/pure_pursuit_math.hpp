#pragma once
#include <cmath>
#include <vector>
#include <algorithm>
#include <limits>

struct Point {
    double x;
    double y;
};

class PurePursuit {
public:
    PurePursuit() {
        wheelbase_ = 0.33;
        k_gain_ = 0.6;           // Tinh chỉnh cực chuẩn
        min_lookahead_ = 2.5;       // 0.9m là HOÀN HẢO cho hình số 8 nhỏ
        max_steering_angle_ = 0.78;
    }

    double calculate_lookahead_distance(double v) {
        return k_gain_ * std::abs(v) + min_lookahead_;
    }

    Point find_lookahead_point(double x, double y, size_t progress, const std::vector<Point>& path, double Ld) {
        if (path.empty()) return {x, y};

        Point best = path[progress % path.size()];
        double best_dist = std::numeric_limits<double>::max();

        for (size_t i = progress; i < progress + path.size(); ++i) {
            size_t idx = i % path.size();
            double d = std::hypot(path[idx].x - x, path[idx].y - y);
            if (d >= Ld && d < best_dist) {
                best_dist = d;
                best = path[idx];
            }
        }
        return best;
    }

    size_t update_progress(double x, double y, const std::vector<Point>& path) {
        if (path.empty()) return 0;
        double min_d = 1e9;
        size_t idx = 0;
        for (size_t i = 0; i < path.size(); ++i) {
            double d = std::hypot(path[i].x - x, path[i].y - y);
            if (d < min_d) {
                min_d = d;
                idx = i;
            }
        }
        return idx;
    }

    double calculate_steering(double x, double y, double yaw, double v, const std::vector<Point>& path, size_t& progress) {
        progress = update_progress(x, y, path);
        double Ld = calculate_lookahead_distance(v);
        Point target = find_lookahead_point(x, y, progress, path, Ld);

        double dx = target.x - x;
        double dy = target.y - y;
        double local_x =  cos(yaw) * dx + sin(yaw) * dy;
        double local_y = -sin(yaw) * dx + cos(yaw) * dy;

        double alpha = atan2(local_y, local_x);
        double curvature = 2.0 * sin(alpha) / Ld;
        double steer = atan(curvature * wheelbase_);

        steer = std::clamp(steer, -max_steering_angle_, max_steering_angle_);
        return steer;
    }

private:
    double wheelbase_;
    double k_gain_;
    double min_lookahead_;
    double max_steering_angle_;
};

