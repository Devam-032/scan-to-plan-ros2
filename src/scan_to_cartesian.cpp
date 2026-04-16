#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
#include <queue>

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

        cartesian_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/scan_cartesian", 10);
        occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/occ_grid_Devam", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/planned_path", 10);

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
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        });
    }

private:
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
            double r = msg.ranges[k];
            bool is_hit = std::isfinite(r) &&
                          r >= msg.range_min &&
                          r <= msg.range_max;

            double r_trace = is_hit ? std::min(r, FREE_TRACE_MAX_) : FREE_TRACE_MAX_;

            double theta = msg.angle_min + k * msg.angle_increment + theta_;

            double x_trace = posx_ + r_trace * std::cos(theta);
            double y_trace = posy_ + r_trace * std::sin(theta);

            int col_trace = static_cast<int>(
                (x_trace - grid_.info.origin.position.x) / grid_.info.resolution);
            int row_trace = static_cast<int>(
                (y_trace - grid_.info.origin.position.y) / grid_.info.resolution);

            if (col_trace < 0 || col_trace >= static_cast<int>(grid_.info.width) ||
                row_trace < 0 || row_trace >= static_cast<int>(grid_.info.height))
                continue;


            auto cells = line_equation(col_robot, row_robot,
                                       col_trace, row_trace);
            for (auto & cell : cells)
            {
                int c  = cell.first;
                int r_ = cell.second;
                if (c  < 0 || c  >= static_cast<int>(grid_.info.width)  ||
                    r_ < 0 || r_ >= static_cast<int>(grid_.info.height))
                    continue;
                int idx = r_ * grid_.info.width + c;
                log_odds_[idx] = std::clamp(
                    log_odds_[idx] + L_FREE, L_MIN, L_MAX);
            }


            if (is_hit)
            {
                double x = posx_ + r * std::cos(theta);
                double y = posy_ + r * std::sin(theta);
                xs.push_back(x);
                ys.push_back(y);

                int col_ray = static_cast<int>(
                    (x - grid_.info.origin.position.x) / grid_.info.resolution);
                int row_ray = static_cast<int>(
                    (y - grid_.info.origin.position.y) / grid_.info.resolution);

                if (col_ray >= 0 && col_ray < static_cast<int>(grid_.info.width) && row_ray >= 0 && row_ray < static_cast<int>(grid_.info.height))
                {
                    int idx = row_ray * grid_.info.width + col_ray;
                    log_odds_[idx] = std::clamp(
                        log_odds_[idx] + L_OCC, L_MIN, L_MAX);
                }
            }
        }
        for (size_t i = 0; i < grid_.data.size(); i++) 
        {
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

        // Robot's current cell = start
        int start_col = static_cast<int>(
            (posx_ - grid_.info.origin.position.x) / grid_.info.resolution);
        int start_row = static_cast<int>(
            (posy_ - grid_.info.origin.position.y) / grid_.info.resolution);

        // Goal cell — hardcoded for now, you'll make this dynamic later
        int goal_col = static_cast<int>(
            (goal_x_ - grid_.info.origin.position.x) / grid_.info.resolution);
        int goal_row = static_cast<int>(
            (goal_y_ - grid_.info.origin.position.y) / grid_.info.resolution);

        auto path_cells = astar(start_col, start_row, goal_col, goal_row);

        if (!path_cells.empty())
        {
            nav_msgs::msg::Path path_msg;
            path_msg.header.stamp    = this->now();
            path_msg.header.frame_id = "odom";

            for (auto & cell : path_cells)
            {
                geometry_msgs::msg::PoseStamped pose;
                pose.header = path_msg.header;
                // Convert cell back to world coordinates
                pose.pose.position.x = grid_.info.origin.position.x +
                                    (cell.first  + 0.5) * grid_.info.resolution;
                pose.pose.position.y = grid_.info.origin.position.y +
                                    (cell.second + 0.5) * grid_.info.resolution;
                pose.pose.orientation.w = 1.0;
                path_msg.poses.push_back(pose);
            }
            path_pub_->publish(path_msg);
        }
    }

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

    std::vector<std::pair<int,int>> line_equation(int x0, int y0, int x1, int y1)
    {
        std::vector<std::pair<int,int>> cells;
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;
        int x = x0, y = y0;

        while (true) {
            cells.push_back({x, y});
            if (x == x1 && y == y1) break;
            int e2 = 2 * err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 <  dx) { err += dx; y += sy; }
        }
        return cells;
    }

    float heuristic(int col, int row, int goal_col, int goal_row)
    {
        float dc = goal_col - col;
        float dr = goal_row - row;
        return std::sqrt(dc*dc + dr*dr);
    }

    std::vector<std::pair<int,int>> astar(
        int start_col, int start_row,
        int goal_col,  int goal_row)
    {
        using Node = std::tuple<float,int,int>;
        std::priority_queue<Node, std::vector<Node>, std::greater<Node>> open;

        int W = grid_.info.width;
        int H = grid_.info.height;

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

        while (!open.empty())
        {
            auto [f, col, row] = open.top();
            open.pop();

            if (col == goal_col && row == goal_row) break;

            for (auto & d : dirs)
            {
                int nc = col + d[0];
                int nr = row + d[1];

                if (nc < 0 || nc >= W || nr < 0 || nr >= H) continue;

                // ← key difference from your 5x5 example:
                // obstacle check uses grid_.data == 100, not grid[nr][nc] == 1
                if (grid_.data[nr * W + nc] == 100) continue;

                float move_cost = (d[0] != 0 && d[1] != 0) ? 1.414f : 1.0f;
                float new_g = g_cost[row * W + col] + move_cost;

                if (new_g < g_cost[nr * W + nc]) {
                    g_cost[nr * W + nc] = new_g;
                    came_from[nr * W + nc] = {col, row};
                    float h = heuristic(nc, nr, goal_col, goal_row);
                    open.push({new_g + h, nc, nr});
                }
            }
        }

        // Reconstruct
        std::pair<int,int> current = {goal_col, goal_row};
        while (current != std::pair<int,int>{-1,-1})
        {
            path.push_back(current);
            current = came_from[current.second * W + current.first];
        }
        std::reverse(path.begin(), path.end());
        return path;
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cartesian_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr     occupancy_grid_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;
    

    double posx_ = 0.0, posy_ = 0.0, theta_ = 0.0;
    double goal_x_ = 2.0;
    double goal_y_ = 2.0;

    nav_msgs::msg::OccupancyGrid grid_;
    std::vector<float>           log_odds_;

    static constexpr float  L_OCC          =  3.0f;
    static constexpr float  L_FREE         = -0.10f;
    static constexpr float  L_MIN          = -10.0f;
    static constexpr float  L_MAX          =  20.0f;
    static constexpr double FREE_TRACE_MAX_ =  3.4;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserSub>());
    rclcpp::shutdown();
}