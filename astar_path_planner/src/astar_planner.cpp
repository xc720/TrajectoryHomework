#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <vector>
#include <queue>
#include <cmath>
#include <iostream>
#include <algorithm>
#include "Trajectory.cpp"
#include "BSplineTrajectory.cpp"
#include <chrono>  // C++11 chrono库

struct Node {
    int x, y;        // 节点所在的网格坐标
    double g_cost;   // 从起点到当前节点的代价
    double h_cost;   // 从当前节点到终点的估计代价
    std::shared_ptr<Node> parent;    // 父节点，用于回溯路径

    Node(int x_, int y_, double g, double h, std::shared_ptr<Node> parent_ = nullptr)
        : x(x_), y(y_), g_cost(g), h_cost(h), parent(std::move(parent_)) {}

    double f() const {
        return g_cost + h_cost;
    } // 总代价值
};

struct cmp {
    bool operator()(std::shared_ptr<Node> a, std::shared_ptr<Node> b) {
        return a->f() > b->f();
    }
};

struct GridMap {
    int width;
    int height;
    double map_min;
    double map_max;
    double grid_resolution;            // 分辨率
    std::vector<std::vector<int>> grid; // 0: 空闲, 1: 障碍物

    GridMap(int w, int h, double map_min_, double map_max_, double res)
        : width(w), height(h), map_min(map_min_), map_max(map_max_),
          grid_resolution(res), grid(w, std::vector<int>(h, 0)) {}

    // 将一片圆形区域标记为障碍物
    void markObstacle(double cx, double cy, double radius) {
        int grid_cx = std::round((cx - map_min) / grid_resolution);
        int grid_cy = std::round((cy - map_min) / grid_resolution);
        int grid_radius = std::round(radius / grid_resolution);

        // Step 1: 将圆形区域标记为占用
        for (int x = -grid_radius; x <= grid_radius; ++x) {
            for (int y = -grid_radius; y <= grid_radius; ++y) {
                int nx = grid_cx + x;
                int ny = grid_cy + y;
                if (nx >= 0 && nx < width && ny >= 0 && ny < height) {
                    // 如果在圆形内，则标记为障碍物
                    if (std::sqrt(x * x + y * y) <= grid_radius) {
                        grid[nx][ny] = 1;
                    }
                }
            }
        }
        // finish
    }
};

class AStarPlanner {
public:
    AStarPlanner(int width, int height,
                 double m_min, double m_max, double res)
        : width_(width), height_(height),
          map_min_(m_min), map_max_(m_max),
          grid_resolution_(res),
          grid_map_(width, height, m_min, m_max, res),
          num_of_obs_(0) {
    }

    void setObstacle(double cx, double cy, double radius) {
        num_of_obs_++;
        grid_map_.markObstacle(cx, cy, radius);
    }

    void printGridMap() {
        for (int i = 0; i < width_; i++) {
            for (int j = 0; j < height_; j++) {
                std::cout << grid_map_.grid[i][j] << " ";
            }
            std::cout << std::endl;
        }
        std::cout << "num of obstacles: " << num_of_obs_ << std::endl;
    }

    // A* 主函数
    std::vector<Eigen::Vector2d> findPath(const Eigen::Vector2d& start,
                                          const Eigen::Vector2d& goal) {
        // 如果地图中没有障碍物，则直接返回空
        if (num_of_obs_ == 0) {
            std::cerr << "[AStar] No obstacles in map, skip searching.\n";
            return {};
        }

        auto gridStart = worldToGrid(start);
        auto gridGoal  = worldToGrid(goal);

        // 开放列表(优先队列)和闭集
        std::priority_queue<std::shared_ptr<Node>,
                            std::vector<std::shared_ptr<Node>>,
                            cmp> open_list;
        std::vector<std::vector<bool>> closed_list(width_, std::vector<bool>(height_, false));

        // 起点加入开放列表
        auto start_node = std::make_shared<Node>(
            Node(gridStart.first, gridStart.second,
                 0.0, heuristic(gridStart, gridGoal)));
        open_list.push(start_node);

        // Step 3： 实现 A* 算法，搜索结束调用 reconstructPath 返回路径
        while (!open_list.empty()) {
            auto current = open_list.top();
            open_list.pop();

            // 如果已经访问过，则跳过
            if (closed_list[current->x][current->y]) {
                continue;
            }
            closed_list[current->x][current->y] = true;

            // 检查是否到达终点
            if (current->x == gridGoal.first && current->y == gridGoal.second) {
                // 回溯得到路径
                return reconstructPath(current);
            }

            // 获取邻居
            auto neighbors = getNeighbors(*current);
            for (auto& nb : neighbors) {
                if (!closed_list[nb.x][nb.y]) {
                    nb.h_cost = heuristic({nb.x, nb.y}, gridGoal);
                    nb.parent = current;
                    open_list.push(std::make_shared<Node>(nb));
                }
            }
        }

        // 如果没有找到路径，返回空
        return {};
    }

    void reset() {
        num_of_obs_ = 0;
        grid_map_.grid = std::vector<std::vector<int>>(width_, std::vector<int>(height_, 0));
    }

    // 膨胀障碍物，避免机器人擦边碰撞
    void inflateObstacles(int inflate_radius) {
        std::vector<std::vector<int>> inflated = grid_map_.grid;
        for (int x = 0; x < width_; x++) {
            for (int y = 0; y < height_; y++) {
                if (grid_map_.grid[x][y] == 1) {
                    // 以(x,y)为中心，对周围 inflate_radius 范围全部标障碍
                    for (int dx = -inflate_radius; dx <= inflate_radius; dx++) {
                        for (int dy = -inflate_radius; dy <= inflate_radius; dy++) {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (inMapRange(nx, ny)) {
                                inflated[nx][ny] = 1;
                            }
                        }
                    }
                }
            }
        }
        grid_map_.grid = inflated;
    }

    // world -> grid
    std::pair<int, int> worldToGrid(const Eigen::Vector2d& pos) const {
        int gx = std::round((pos.x() - map_min_) / grid_resolution_);
        int gy = std::round((pos.y() - map_min_) / grid_resolution_);
        return {gx, gy};
    }

    // grid -> world
    Eigen::Vector2d gridToWorld(int gx, int gy) const {
        double wx = gx * grid_resolution_ + map_min_;
        double wy = gy * grid_resolution_ + map_min_;
        return Eigen::Vector2d(wx, wy);
    }

    int getWidth()  const { return width_;  }
    int getHeight() const { return height_; }

    // 获取某格子的占用值(0或1)，越界则返回1(当作障碍)
    int getGridValue(int x, int y) const {
        if (!inMapRange(x, y)) return 1;
        return grid_map_.grid[x][y];
    }

private:
    int width_, height_;
    double map_min_, map_max_, grid_resolution_;
    GridMap grid_map_;
    int num_of_obs_;

    bool inMapRange(int x, int y) const {
        return (x >= 0 && x < width_ &&
                y >= 0 && y < height_);
    }

    // 计算启发式代价（使用欧几里得距离）
    double heuristic(const std::pair<int,int>& from, const std::pair<int,int>& to) const {
        return std::sqrt(std::pow(from.first - to.first, 2) +
                         std::pow(from.second - to.second, 2));
    }

    // 计算节点间距离（支持八连通）
    double distance(const Node& a, const Node& b) const {
        return std::sqrt(std::pow(a.x - b.x, 2) +
                         std::pow(a.y - b.y, 2));
    }

    // 获取当前节点的所有邻居节点（八连通）
    std::vector<Node> getNeighbors(const Node& current) const {
        std::vector<Node> neighbors;
        static std::vector<std::pair<int,int>> directions = {
            {1, 0}, {0, 1}, {-1, 0}, {0, -1},
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
        };

        for (auto& dir : directions) {
            int nx = current.x + dir.first;
            int ny = current.y + dir.second;
            if (inMapRange(nx, ny)) {
                if (grid_map_.grid[nx][ny] == 0) {
                    double g_cost = current.g_cost
                                  + distance(current, Node(nx, ny, 0, 0));
                    neighbors.emplace_back(nx, ny, g_cost, 0);
                }
            }
        }
        return neighbors;
    }

    // 回溯路径
    std::vector<Eigen::Vector2d> reconstructPath(std::shared_ptr<Node> node) const {
        std::vector<Eigen::Vector2d> path;
        while (node) {
            path.push_back(gridToWorld(node->x, node->y));
            node = node->parent;
        }
        std::reverse(path.begin(), path.end());
        return path;
    }
};


typedef std::vector<Eigen::Vector2d> Path;

// 对离散路径做插值
Path interpolatePath(const Path& raw_path, double step_size) {
    Path interpolated_path;
    for (size_t i = 0; i + 1 < raw_path.size(); ++i) {
        Eigen::Vector2d start = raw_path[i];
        Eigen::Vector2d end   = raw_path[i + 1];
        double dist = (end - start).norm();
        int num_points = std::ceil(dist / step_size);
        for (int j = 0; j <= num_points; ++j) {
            double t = (double)j / (double)num_points;
            Eigen::Vector2d point = (1.0 - t)*start + t*end;
            interpolated_path.push_back(point);
        }
    }
    if (!raw_path.empty()) {
        interpolated_path.push_back(raw_path.back());
    }
    return interpolated_path;
}

// 简易碰撞检查
bool checkPathCollision(const Path& path, const AStarPlanner& planner) {
    for (auto& pt : path) {
        auto grid_coord = planner.worldToGrid(pt);
        int x = grid_coord.first;
        int y = grid_coord.second;
        // 若越界或在障碍物上，则视为碰撞
        if (x < 0 || x >= planner.getWidth() ||
            y < 0 || y >= planner.getHeight() ||
            planner.getGridValue(x, y) == 1) {
            return true;
        }
    }
    return false;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "astar_planner_all_in_one");
    ros::NodeHandle nh;

    // 地图相关参数
    double map_min, map_max, grid_resolution;
    double start_x, start_y, goal_x, goal_y;
    nh.param("astar_planner/map_min",         map_min,        -5.0);
    nh.param("astar_planner/map_max",         map_max,         5.0);
    nh.param("astar_planner/grid_resolution", grid_resolution, 0.1);
    nh.param("astar_planner/start_x",         start_x,        -4.5);
    nh.param("astar_planner/start_y",         start_y,        -4.5);
    nh.param("astar_planner/goal_x",          goal_x,          4.5);
    nh.param("astar_planner/goal_y",          goal_y,          4.5);

    int grid_width  = std::round((map_max - map_min) / grid_resolution);
    int grid_height = grid_width;
    AStarPlanner planner(grid_width, grid_height, map_min, map_max, grid_resolution);

    // b样条轨迹优化器
    BSplineTrajectoryGenerator bspline_generator;

    // 障碍物订阅
    ros::Subscriber obs_sub = nh.subscribe<visualization_msgs::MarkerArray>(
        "obstacles", 1,
        [&planner](const visualization_msgs::MarkerArray::ConstPtr& msg){
            for (auto & marker : msg->markers) {
                double r = marker.scale.x * 0.5;
                planner.setObstacle(marker.pose.position.x, marker.pose.position.y, r);
            }
        }
    );

    // 发布路径
    ros::Publisher path_pub      = nh.advertise<nav_msgs::Path>("path",      1);
    ros::Publisher raw_path_pub  = nh.advertise<nav_msgs::Path>("raw_path",  1);

    ros::Rate loop_rate(10);

    // 起点和终点
    Eigen::Vector2d start(start_x, start_y);
    Eigen::Vector2d goal(goal_x,  goal_y);

    // 标记是否已经对障碍物执行过一次膨胀
    bool inflated_once = false;

    while (ros::ok()) {
        ros::spinOnce();

        // 第一次运行时，先对障碍物执行膨胀，避免贴边碰撞
        if (!inflated_once) {
            int inflate_radius_in_grid = 1; // 根据机器人尺寸配置
            planner.inflateObstacles(inflate_radius_in_grid);
            inflated_once = true;
        }

        // 计时 A* 开始
        auto astar_start_time = std::chrono::steady_clock::now();

        // 调用 A* 搜索
        std::vector<Eigen::Vector2d> raw_path = planner.findPath(start, goal);

        // 计时 A* 结束
        auto astar_end_time = std::chrono::steady_clock::now();
        double astar_time_ms = std::chrono::duration<double, std::milli>(
                                  astar_end_time - astar_start_time).count();
        ROS_INFO("[Timing] A* search took %.3f ms.", astar_time_ms);


        if (raw_path.empty()) {
            ROS_WARN("No valid path found. Will try again...");
            loop_rate.sleep();
            continue;
        }

        // 路径碰撞检查
        if (checkPathCollision(raw_path, planner)) {
            ROS_ERROR("Planned path in collision. Will replan...");
            loop_rate.sleep();
            continue;
        }

        // 对 A* 的路径做插值
        Path interpolated = interpolatePath(raw_path, 0.1);

        // 计时 B样条开始
        auto bspline_start_time = std::chrono::steady_clock::now();

        // 用 B 样条生成平滑轨迹
        Path bspline_traj = bspline_generator.generateTrajectory(interpolated, 3, 100);

        auto bspline_end_time = std::chrono::steady_clock::now();
        double bspline_time_ms = std::chrono::duration<double, std::milli>(
                                     bspline_end_time - bspline_start_time).count();
        // 计时 B样条结束
        ROS_INFO("[Timing] B-spline generation took %.3f ms.", bspline_time_ms);


        // 发布 A* 的离散路径（可视化）
        nav_msgs::Path raw_path_msg;
        raw_path_msg.header.frame_id = "map";
        raw_path_msg.header.stamp    = ros::Time::now();
        for (auto& pt : raw_path) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = pt.x();
            pose.pose.position.y = pt.y();
            raw_path_msg.poses.push_back(pose);
        }
        raw_path_pub.publish(raw_path_msg);

        // 发布优化过的路径（B样条）用于实际跟踪
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp    = ros::Time::now();
        for (auto& pt : bspline_traj) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position.x = pt.x();
            pose.pose.position.y = pt.y();
            pose.pose.position.z = 0.0; // 平面路径，z 设置为 0
            path_msg.poses.push_back(pose);
        }
        path_pub.publish(path_msg);

        loop_rate.sleep();
    }

    return 0;
}
