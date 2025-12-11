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
        wheelbase_ = 0.30;       // Match URDF: front_wheel(0.15) - rear_wheel(-0.15) = 0.30
        k_gain_ = 0.6;           // Speed-dependent lookahead gain
        min_lookahead_ = 1.5;    // Large lookahead for smooth curved tracking
        max_steering_angle_ = 0.6;  // Allow moderate steering for curves
    }

    double calculate_lookahead_distance(double v) {
        return k_gain_ * std::abs(v) + min_lookahead_;
    }

    Point find_lookahead_point(double x, double y, size_t progress, const std::vector<Point>& path, double Ld) {
        if (path.empty()) return {x, y};

        Point best = path[progress % path.size()];
        double best_dist = std::numeric_limits<double>::max();

        // Search forward through the entire path until we find the first point beyond Ld.
        // This keeps the target in front of the vehicle along the path order.
        for (size_t i = 0; i < path.size(); ++i) {
            size_t idx = (progress + i) % path.size();
            double d = std::hypot(path[idx].x - x, path[idx].y - y);
            if (d >= Ld) {
                best = path[idx];
                break;
            }
            if (d < best_dist) {
                best_dist = d;
                best = path[idx];
            }
        }
        return best;
    }

    size_t update_progress(double x, double y, size_t current_progress, const std::vector<Point>& path) {
        if (path.empty()) return 0;

        // Check if we're close enough to the NEXT waypoint to advance
        size_t next_idx = (current_progress + 1) % path.size();
        double dist_to_next = std::hypot(path[next_idx].x - x, path[next_idx].y - y);
        
        // If within 0.5m of next waypoint, advance to it
        if (dist_to_next < 0.5) {
            return next_idx;
        }
        
        // Otherwise stay at current waypoint
        return current_progress;
    }

    double calculate_steering(double x, double y, double yaw, double v, const std::vector<Point>& path, size_t& progress) {
        if (path.empty()) return 0.0;
        
        progress = update_progress(x, y, progress, path);
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

