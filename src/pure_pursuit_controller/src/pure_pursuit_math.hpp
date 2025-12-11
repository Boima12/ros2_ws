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
        k_gain_ = 0.3;           // Reduced from 0.6 for smoother steering
        min_lookahead_ = 0.8;    // Reduced from 2.5 - was way too large for figure-8
        max_steering_angle_ = 0.78;
    }

    double calculate_lookahead_distance(double v) {
        return k_gain_ * std::abs(v) + min_lookahead_;
    }

    Point find_lookahead_point(double x, double y, size_t progress, const std::vector<Point>& path, double Ld) {
        if (path.empty()) return {x, y};

        // Start searching from current progress, not from progress index directly
        Point best = path[progress % path.size()];
        double best_dist = std::numeric_limits<double>::max();

        // Search within a window to avoid jumping to far-away waypoints
        size_t search_window = std::min(size_t(15), path.size());
        for (size_t i = 0; i < search_window; ++i) {
            size_t idx = (progress + i) % path.size();
            double d = std::hypot(path[idx].x - x, path[idx].y - y);
            // Find the first point that's at least Ld away
            if (d >= Ld && d < best_dist) {
                best_dist = d;
                best = path[idx];
            }
        }
        return best;
    }

    size_t update_progress(double x, double y, size_t current_progress, const std::vector<Point>& path) {
        if (path.empty()) return 0;
        
        // Only update progress if we're getting closer to the next waypoint
        size_t next_idx = (current_progress + 1) % path.size();
        size_t curr_idx = current_progress % path.size();
        
        double dist_to_current = std::hypot(path[curr_idx].x - x, path[curr_idx].y - y);
        double dist_to_next = std::hypot(path[next_idx].x - x, path[next_idx].y - y);
        
        // Move to next waypoint when within 0.5m
        if (dist_to_next < 0.5) {
            return (current_progress + 1) % path.size();
        }
        
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

