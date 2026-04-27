#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <cmath>
#include <queue>
#include <climits>



class LaserSub : public rclcpp::Node
{
public:
    LaserSub() : Node("scan_to_cartesian")
    {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10,
            std::bind(&LaserSub::odom_cb, this, std::placeholders::_1));
        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&LaserSub::scan_sub, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/goal_pose", 10,
            std::bind(&LaserSub::goal_cb, this, std::placeholders::_1));

        cartesian_pub_      = this->create_publisher<std_msgs::msg::Float64MultiArray>("/scan_cartesian", 10);
        occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/occ_grid_Devam", 10);
        path_pub_           = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);
        cmd_vel_pub_        = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        grid_.info.resolution = 0.02f;
        grid_.info.width      = 500;
        grid_.info.height     = 500;
        grid_.info.origin.position.x = -5.0;
        grid_.info.origin.position.y = -5.0;
        grid_.info.origin.position.z =  0.0;
        grid_.info.origin.orientation.w = 1.0;

        grid_.data.resize(grid_.info.width * grid_.info.height, -1);
        log_odds_.resize(grid_.info.width * grid_.info.height, 0.0f);

        this->declare_parameter("goal_x", 2.0);
        this->declare_parameter("goal_y", 2.0);
        goal_x_ = this->get_parameter("goal_x").as_double();
        goal_y_ = this->get_parameter("goal_y").as_double();

        param_cb_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & params)
            {
                for (auto & p : params) {
                    if (p.get_name() == "goal_x") goal_x_ = p.as_double();
                    if (p.get_name() == "goal_y") goal_y_ = p.as_double();
                    RCLCPP_INFO(this->get_logger(),
                        "Goal updated: (%.2f, %.2f)", goal_x_, goal_y_);
                }
                prev_idx_     = 0;
                next_idx_     = 1;
                goal_reached_ = false;
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                return result;
            });

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&LaserSub::control_loop, this));
    }

private:

    // ─────────────────────────────────────────────
    // GOAL CALLBACK — RViz2 2D Nav Goal
    // ─────────────────────────────────────────────
    void goal_cb(const geometry_msgs::msg::PoseStamped & msg)
    {
        goal_x_       = msg.pose.position.x;
        goal_y_       = msg.pose.position.y;
        prev_idx_     = 0;
        next_idx_     = 1;
        goal_reached_ = false;
        RCLCPP_INFO(this->get_logger(),
            "New goal from RViz2: (%.2f, %.2f)", goal_x_, goal_y_);
    }

    // ─────────────────────────────────────────────
    // INFLATION — gradient cost BFS
    // ─────────────────────────────────────────────
    std::vector<int8_t> inflate_grid(
        const std::vector<int8_t> & data,
        int W, int H, int radius)
    {
        std::vector<int8_t> inflated = data;
        std::vector<int>    dist(W * H, INT_MAX);
        std::queue<std::pair<int,int>> q;

        int dirs[8][2] = {
            {1,0},{-1,0},{0,1},{0,-1},
            {1,1},{1,-1},{-1,1},{-1,-1}
        };

        // Seed with all lethal cells
        for (int r = 0; r < H; r++) {
            for (int c = 0; c < W; c++) {
                if (data[r * W + c] == 100) {
                    dist[r * W + c] = 0;
                    q.push({c, r});
                }
            }
        }

        // BFS outward — gradient cost based on distance
        while (!q.empty()) {
            auto [c, r] = q.front(); q.pop();
            for (auto & d : dirs) {
                int nc = c + d[0], nr = r + d[1];
                if (nc < 0 || nc >= W || nr < 0 || nr >= H) continue;
                int new_dist = dist[r * W + c] + 1;
                if (new_dist <= radius && new_dist < dist[nr * W + nc]) {
                    dist[nr * W + nc] = new_dist;
                    if (inflated[nr * W + nc] != 100) {
                        // Closer to wall = higher cost, max 80 (not lethal)
                        float ratio   = 1.0f - static_cast<float>(new_dist) / radius;
                        int8_t cost   = static_cast<int8_t>(ratio * 80.0f);
                        inflated[nr * W + nc] = std::max(inflated[nr * W + nc], cost);
                    }
                    q.push({nc, nr});
                }
            }
        }
        return inflated;
    }

    // ─────────────────────────────────────────────
    // RDP PATH SMOOTHING
    // ─────────────────────────────────────────────
    std::vector<geometry_msgs::msg::PoseStamped> rdp_smooth(
        const std::vector<geometry_msgs::msg::PoseStamped> & poses,
        double epsilon)
    {
        if (poses.size() < 3) return poses;

        double x1 = poses.front().pose.position.x;
        double y1 = poses.front().pose.position.y;
        double x2 = poses.back().pose.position.x;
        double y2 = poses.back().pose.position.y;

        double dx       = x2 - x1;
        double dy       = y2 - y1;
        double line_len = std::sqrt(dx*dx + dy*dy);

        double max_dist = 0.0;
        size_t max_idx  = 0;

        for (size_t i = 1; i < poses.size() - 1; i++) {
            double px   = poses[i].pose.position.x;
            double py   = poses[i].pose.position.y;
            double dist = (line_len < 1e-6) ?
                std::sqrt(std::pow(px-x1,2) + std::pow(py-y1,2)) :
                std::abs(dy*px - dx*py + x2*y1 - y2*x1) / line_len;
            if (dist > max_dist) { max_dist = dist; max_idx = i; }
        }

        if (max_dist > epsilon) {
            auto left = rdp_smooth(
                std::vector<geometry_msgs::msg::PoseStamped>(
                    poses.begin(), poses.begin() + max_idx + 1), epsilon);
            auto right = rdp_smooth(
                std::vector<geometry_msgs::msg::PoseStamped>(
                    poses.begin() + max_idx, poses.end()), epsilon);
            left.pop_back();
            left.insert(left.end(), right.begin(), right.end());
            return left;
        }
        return {poses.front(), poses.back()};
    }

    // ─────────────────────────────────────────────
    // CURVATURE COMPUTATION
    // ─────────────────────────────────────────────
    std::vector<double> compute_curvatures(
        const std::vector<geometry_msgs::msg::PoseStamped> & poses)
    {
        size_t n = poses.size();
        std::vector<double> curvatures(n, 0.0);

        for (size_t i = 1; i + 1 < n; i++) {
            double ax = poses[i-1].pose.position.x;
            double ay = poses[i-1].pose.position.y;
            double bx = poses[i].pose.position.x;
            double by = poses[i].pose.position.y;
            double cx = poses[i+1].pose.position.x;
            double cy = poses[i+1].pose.position.y;

            double ab_x = bx-ax, ab_y = by-ay;
            double bc_x = cx-bx, bc_y = cy-by;
            double ac_x = cx-ax, ac_y = cy-ay;

            double cross  = ab_x*bc_y - ab_y*bc_x;
            double len_ab = std::sqrt(ab_x*ab_x + ab_y*ab_y);
            double len_bc = std::sqrt(bc_x*bc_x + bc_y*bc_y);
            double len_ac = std::sqrt(ac_x*ac_x + ac_y*ac_y);
            double denom  = len_ab * len_bc * len_ac;

            curvatures[i] = (denom < 1e-6) ? 0.0 :
                            2.0 * std::abs(cross) / denom;
        }

        if (n > 1) {
            curvatures[0]   = curvatures[1];
            curvatures[n-1] = curvatures[n-2];
        }
        return curvatures;
    }

    // ─────────────────────────────────────────────
    // SPEED FROM CURVATURE
    // ─────────────────────────────────────────────
    double curvature_to_speed(double curvature)
    {
        return std::clamp(
            MAX_SPEED_ / (1.0 + CURV_GAIN_ * curvature),
            MIN_SPEED_, MAX_SPEED_);
    }

    // ─────────────────────────────────────────────
    // CONTROL LOOP — 10 Hz
    // ─────────────────────────────────────────────
    void control_loop()
    {
        if (current_path_.poses.size() < 2) return;

        double dx = posx_ - goal_x_;
        double dy = posy_ - goal_y_;
        if (std::sqrt(dx*dx + dy*dy) < GOAL_TOLERANCE_) {
            if (!goal_reached_) {
                RCLCPP_INFO(this->get_logger(), "Goal reached!");
                goal_reached_ = true;
            }
            stop_robot();
            return;
        }
        goal_reached_ = false;

        size_t path_size = current_path_.poses.size();

        double nx = current_path_.poses[next_idx_].pose.position.x;
        double ny = current_path_.poses[next_idx_].pose.position.y;
        double dist_to_next = std::sqrt(
            std::pow(posx_ - nx, 2) + std::pow(posy_ - ny, 2));

        if (dist_to_next < WAYPOINT_RADIUS_ && next_idx_ < path_size - 1) {
            prev_idx_ = next_idx_;
            next_idx_ = std::min(next_idx_ + 1, path_size - 1);
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "Segment advanced [%zu → %zu]", prev_idx_, next_idx_);
        }

        double prev_x = current_path_.poses[prev_idx_].pose.position.x;
        double prev_y = current_path_.poses[prev_idx_].pose.position.y;
        double next_x = current_path_.poses[next_idx_].pose.position.x;
        double next_y = current_path_.poses[next_idx_].pose.position.y;

        auto   curvatures = compute_curvatures(current_path_.poses);
        double curvature  = curvatures[next_idx_];
        double speed      = curvature_to_speed(curvature);

        double steer = stanley(posx_, posy_, theta_, speed,
                               prev_x, prev_y, next_x, next_y);

        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = speed;
        cmd.angular.z = steer;
        cmd_vel_pub_->publish(cmd);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
            "speed=%.3f  curv=%.3f  steer=%.3f  seg=[%zu→%zu]",
            speed, curvature, steer, prev_idx_, next_idx_);
    }

    // ─────────────────────────────────────────────
    // STANLEY CONTROLLER
    // ─────────────────────────────────────────────
    double stanley(
        double curr_x,       double curr_y,
        double curr_heading, double curr_speed,
        double prev_x,       double prev_y,
        double next_x,       double next_y)
    {
        double a = prev_y - next_y;
        double b = -(prev_x - next_x);
        double c = prev_x * next_y - next_x * prev_y;

        double denom = std::sqrt(a*a + b*b);
        if (denom < 1e-6) return 0.0;

        double cte = (a * curr_x + b * curr_y + c) / denom;

        double path_heading  = std::atan2(next_y - prev_y, next_x - prev_x);
        double heading_error = std::atan2(
            std::sin(path_heading - curr_heading),
            std::cos(path_heading - curr_heading));

        double steering = heading_error +
                          std::atan(STANLEY_K_ * cte / (curr_speed + STANLEY_EPS_));

        return std::clamp(steering, -MAX_STEER_, MAX_STEER_);
    }

    // ─────────────────────────────────────────────
    // STOP ROBOT
    // ─────────────────────────────────────────────
    void stop_robot()
    {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x  = 0.0;
        cmd.angular.z = 0.0;
        cmd_vel_pub_->publish(cmd);
    }

    // ─────────────────────────────────────────────
    // SCAN CALLBACK — mapping + A*
    // ─────────────────────────────────────────────
    void scan_sub(const sensor_msgs::msg::LaserScan & msg)
    {
        int col_robot = static_cast<int>(
            (posx_ - grid_.info.origin.position.x) / grid_.info.resolution);
        int row_robot = static_cast<int>(
            (posy_ - grid_.info.origin.position.y) / grid_.info.resolution);

        if (col_robot < 0 || col_robot >= static_cast<int>(grid_.info.width) ||
            row_robot < 0 || row_robot >= static_cast<int>(grid_.info.height))
            return;

        std::vector<double> xs, ys;

        for (size_t k = 0; k < msg.ranges.size(); k++)
        {
            double r      = msg.ranges[k];
            bool   is_hit = std::isfinite(r) &&
                            r >= msg.range_min &&
                            r <= msg.range_max;

            double r_trace = is_hit ? std::min(r, FREE_TRACE_MAX_) : FREE_TRACE_MAX_;
            double theta   = msg.angle_min + k * msg.angle_increment + theta_;

            double x_trace = posx_ + r_trace * std::cos(theta);
            double y_trace = posy_ + r_trace * std::sin(theta);

            int col_trace = static_cast<int>(
                (x_trace - grid_.info.origin.position.x) / grid_.info.resolution);
            int row_trace = static_cast<int>(
                (y_trace - grid_.info.origin.position.y) / grid_.info.resolution);

            if (col_trace < 0 || col_trace >= static_cast<int>(grid_.info.width) ||
                row_trace < 0 || row_trace >= static_cast<int>(grid_.info.height))
                continue;

            auto cells = line_equation(col_robot, row_robot, col_trace, row_trace);
            for (auto & cell : cells) {
                int c  = cell.first;
                int r_ = cell.second;
                if (c  < 0 || c  >= static_cast<int>(grid_.info.width)  ||
                    r_ < 0 || r_ >= static_cast<int>(grid_.info.height))
                    continue;
                int idx = r_ * grid_.info.width + c;
                log_odds_[idx] = std::clamp(log_odds_[idx] + L_FREE, L_MIN, L_MAX);
            }

            if (is_hit) {
                double x = posx_ + r * std::cos(theta);
                double y = posy_ + r * std::sin(theta);
                xs.push_back(x);
                ys.push_back(y);

                int col_ray = static_cast<int>(
                    (x - grid_.info.origin.position.x) / grid_.info.resolution);
                int row_ray = static_cast<int>(
                    (y - grid_.info.origin.position.y) / grid_.info.resolution);

                if (col_ray >= 0 && col_ray < static_cast<int>(grid_.info.width) &&
                    row_ray >= 0 && row_ray < static_cast<int>(grid_.info.height)) {
                    int idx = row_ray * grid_.info.width + col_ray;
                    log_odds_[idx] = std::clamp(log_odds_[idx] + L_OCC, L_MIN, L_MAX);
                }
            }
        }

        for (size_t i = 0; i < grid_.data.size(); i++) {
            if      (log_odds_[i] >  3.0f) grid_.data[i] = 100;
            else if (log_odds_[i] < -1.0f) grid_.data[i] = 0;
            else                            grid_.data[i] = -1;
        }

        size_t n = xs.size();
        auto pub_msg = std_msgs::msg::Float64MultiArray();
        pub_msg.data.resize(2 * n);
        for (size_t k = 0; k < n; k++) {
            pub_msg.data[2 * k]     = xs[k];
            pub_msg.data[2 * k + 1] = ys[k];
        }
        cartesian_pub_->publish(pub_msg);

        grid_.header.stamp    = this->now();
        grid_.header.frame_id = "odom";
        occupancy_grid_pub_->publish(grid_);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "Valid hits: %zu / %zu", n, msg.ranges.size());

        // ── Inflate + A* ──────────────────────────────────────────────────
        int W = grid_.info.width;
        int H = grid_.info.height;

        auto inflated = inflate_grid(grid_.data, W, H, INFLATION_CELLS_);

        int start_col = static_cast<int>(
            (posx_ - grid_.info.origin.position.x) / grid_.info.resolution);
        int start_row = static_cast<int>(
            (posy_ - grid_.info.origin.position.y) / grid_.info.resolution);
        int goal_col = static_cast<int>(
            (goal_x_ - grid_.info.origin.position.x) / grid_.info.resolution);
        int goal_row = static_cast<int>(
            (goal_y_ - grid_.info.origin.position.y) / grid_.info.resolution);

        auto path_cells = astar(start_col, start_row,
                                goal_col,  goal_row, inflated);

        if (!path_cells.empty()) {
            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp    = this->now();
            path_msg.header.frame_id = "odom";

            for (auto & cell : path_cells) {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path_msg.header;
                pose.pose.position.x = grid_.info.origin.position.x +
                                       (cell.first  + 0.5) * grid_.info.resolution;
                pose.pose.position.y = grid_.info.origin.position.y +
                                       (cell.second + 0.5) * grid_.info.resolution;
                pose.pose.orientation.w = 1.0;
                path_msg.poses.push_back(pose);
            }

            path_msg.poses = rdp_smooth(path_msg.poses, RDP_EPSILON_);
            current_path_  = path_msg;
            prev_idx_      = 0;
            next_idx_      = 1;
            path_pub_->publish(path_msg);
        }
    }

    // ─────────────────────────────────────────────
    // ODOMETRY CALLBACK
    // ─────────────────────────────────────────────
    void odom_cb(const nav_msgs::msg::Odometry & msg)
    {
        posx_ = msg.pose.pose.position.x;
        posy_ = msg.pose.pose.position.y;
        tf2::Quaternion q;
        tf2::fromMsg(msg.pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        theta_ = yaw;
    }

    // ─────────────────────────────────────────────
    // BRESENHAM
    // ─────────────────────────────────────────────
    std::vector<std::pair<int,int>> line_equation(int x0, int y0, int x1, int y1)
    {
        std::vector<std::pair<int,int>> cells;
        int dx = std::abs(x1-x0), dy = std::abs(y1-y0);
        int sx = (x0 < x1) ? 1 : -1, sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy, x = x0, y = y0;
        while (true) {
            cells.push_back({x, y});
            if (x == x1 && y == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 <  dx) { err += dx; y += sy; }
        }
        return cells;
    }

    // ─────────────────────────────────────────────
    // A* — gradient cost aware
    // ─────────────────────────────────────────────
    float heuristic(int col, int row, int goal_col, int goal_row)
    {
        float dc = goal_col - col, dr = goal_row - row;
        return std::sqrt(dc*dc + dr*dr);
    }

    std::vector<std::pair<int,int>> astar(
        int start_col, int start_row,
        int goal_col,  int goal_row,
        const std::vector<int8_t> & map)
    {
        using Node = std::tuple<float,int,int>;
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

        int W = grid_.info.width, H = grid_.info.height;
        std::vector<float> g_cost(W * H, 1e9f);
        std::vector<std::pair<int,int>> came_from(W * H, {-1,-1});
        std::vector<std::pair<int,int>> path;

        int dirs[8][2] = {
            {1,0},{-1,0},{0,1},{0,-1},
            {1,1},{1,-1},{-1,1},{-1,-1}
        };

        g_cost[start_row * W + start_col] = 0.0f;
        open.push({heuristic(start_col, start_row, goal_col, goal_row),
                   start_col, start_row});

        while (!open.empty()) {
            auto [f, col, row] = open.top(); open.pop();
            if (col == goal_col && row == goal_row) break;

            for (auto & d : dirs) {
                int nc = col + d[0], nr = row + d[1];
                if (nc < 0 || nc >= W || nr < 0 || nr >= H) continue;

                int8_t cell_cost = map[nr * W + nc];
                if (cell_cost == 100) continue;  // lethal — never enter

                // Gradient penalty — discourages near-wall cells
                // but doesn't block them if no other option
                float cost_penalty = (cell_cost > 0) ?
                    static_cast<float>(cell_cost) / 80.0f * COST_PENALTY_WEIGHT_ :
                    0.0f;

                float move_cost = (d[0] != 0 && d[1] != 0) ? 1.414f : 1.0f;
                float new_g     = g_cost[row * W + col] + move_cost + cost_penalty;

                if (new_g < g_cost[nr * W + nc]) {
                    g_cost[nr * W + nc]    = new_g;
                    came_from[nr * W + nc] = {col, row};
                    open.push({new_g + heuristic(nc, nr, goal_col, goal_row),
                               nc, nr});
                }
            }
        }

        std::pair<int,int> current = {goal_col, goal_row};
        while (current != std::pair<int,int>{-1,-1}) {
            path.push_back(current);
            current = came_from[current.second * W + current.first];
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    // ── Subscribers / Publishers ──────────────────────────────────────────
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr    laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr   cartesian_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr       occupancy_grid_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr          cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr                                     control_timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

    // ── Robot state ───────────────────────────────────────────────────────
    double posx_ = 0.0, posy_ = 0.0, theta_ = 0.0;

    // ── Goal ──────────────────────────────────────────────────────────────
    double goal_x_ = 2.0, goal_y_ = 2.0;
    bool   goal_reached_ = false;

    // ── Path tracking ─────────────────────────────────────────────────────
    nav_msgs::msg::Path current_path_;
    size_t prev_idx_ = 0;
    size_t next_idx_ = 1;

    // ── Grid ──────────────────────────────────────────────────────────────
    nav_msgs::msg::OccupancyGrid grid_;
    std::vector<float>           log_odds_;

    // ── Mapping constants ─────────────────────────────────────────────────
    static constexpr float  L_OCC            =  3.0f;
    static constexpr float  L_FREE           = -0.10f;
    static constexpr float  L_MIN            = -10.0f;
    static constexpr float  L_MAX            =  20.0f;
    static constexpr double FREE_TRACE_MAX_  =  3.4;

    // ── Inflation ─────────────────────────────────────────────────────────
    static constexpr int   INFLATION_CELLS_      = 10;   // lethal radius
    static constexpr float COST_PENALTY_WEIGHT_  = 2.0f; // gradient cost weight

    // ── Smoothing ─────────────────────────────────────────────────────────
    static constexpr double RDP_EPSILON_     = 0.1;

    // ── Controller constants ──────────────────────────────────────────────
    static constexpr double STANLEY_K_       =  1.0;
    static constexpr double STANLEY_EPS_     =  0.01;
    static constexpr double MAX_STEER_       =  2.0;
    static constexpr double MAX_SPEED_       =  0.15;
    static constexpr double MIN_SPEED_       =  0.04;
    static constexpr double CURV_GAIN_       =  5.0;
    static constexpr double GOAL_TOLERANCE_  =  0.15;
    static constexpr double WAYPOINT_RADIUS_ =  0.10;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserSub>());
    rclcpp::shutdown();
}