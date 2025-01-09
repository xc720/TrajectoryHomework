#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

// 定义路径格式
typedef std::vector<Eigen::Vector2d> Path;

// Step 4: 轨迹生成类
class TrajectoryGenerator {
    // 轨迹规划的目标是根据A*算法给出的无碰撞路径，计算轨迹航点（控制点），从而生成一条以时间参数化的平滑轨迹，可以用于控制移动机器人跟踪
    // 本次作业中，我们要求生成一条分段多项式轨迹，即每段轨迹均为一个多项式函数
    // 你可以选择使用多项式、B样条、贝塞尔曲线、MINCO等多种轨迹基函数实现
    // 每段轨迹的连接处需要满足一定的连续性条件，如位置连续、速度连续、加速度连续等，这将构成轨迹优化的主要约束条件
    // 轨迹的初始和终止状态为到达指定位置，速度、加速度等状态均为0
    // 优化目标可以是最小化轨迹的加加速度（jerk）或加加加速度（snap），请自行选择合适的优化目标及多项式阶数
    // 本次作业对轨迹段的时间选择不做进一步要求，可以自行选择固定时间或自定义策略生成时间分配
    // 可以任意选用求解器，如qpOASES、OSQP-Eigen等，也可以自行实现闭式解法
public:
    TrajectoryGenerator() = default;

    // 生成分段多项式轨迹
    Path generateTrajectory(const Path& waypoints, int resolution) {
        if (waypoints.size() < 2) {
            throw std::invalid_argument("At least two waypoints are required to generate a trajectory.");
        }

        // 计算每段时间分配（均匀分布）
        std::vector<double> times = computeTimeAllocation(waypoints);

        // 生成轨迹
        Path trajectory;
        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            Path segment = generatePolynomialSegment(waypoints[i], waypoints[i + 1], times[i], resolution);
            trajectory.insert(trajectory.end(), segment.begin(), segment.end());
        }

        return trajectory;
    }

private:
    // 计算每段轨迹的时间分配
    std::vector<double> computeTimeAllocation(const Path& waypoints) {
        std::vector<double> times;
        double totalDistance = 0.0;

        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            totalDistance += (waypoints[i + 1] - waypoints[i]).norm();
        }

        for (size_t i = 0; i < waypoints.size() - 1; ++i) {
            double segmentDistance = (waypoints[i + 1] - waypoints[i]).norm();
            times.push_back(segmentDistance / totalDistance); // 根据比例分配时间
        }

        return times;
    }

    // 生成分段多项式轨迹
    Path generatePolynomialSegment(const Eigen::Vector2d& start, const Eigen::Vector2d& end, double time, int resolution) {
        // 初始条件：位置、速度、加速度为0
        Eigen::MatrixXd A(6, 6);
        Eigen::VectorXd bx(6), by(6);

        A << 0, 0, 0, 0, 0, 1,
             pow(time, 5), pow(time, 4), pow(time, 3), pow(time, 2), time, 1,
             0, 0, 0, 0, 1, 0,
             5 * pow(time, 4), 4 * pow(time, 3), 3 * pow(time, 2), 2 * time, 1, 0,
             0, 0, 0, 2, 0, 0,
             20 * pow(time, 3), 12 * pow(time, 2), 6 * time, 2, 0, 0;

        bx << start.x(), end.x(), 0, 0, 0, 0;
        by << start.y(), end.y(), 0, 0, 0, 0;

        Eigen::VectorXd cx = A.colPivHouseholderQr().solve(bx);
        Eigen::VectorXd cy = A.colPivHouseholderQr().solve(by);

        // 生成轨迹点
        Path segment;
        for (int i = 0; i <= resolution; ++i) {
            double t = time * i / resolution;
            double px = cx(0) * pow(t, 5) + cx(1) * pow(t, 4) + cx(2) * pow(t, 3) + cx(3) * pow(t, 2) + cx(4) * t + cx(5);
            double py = cy(0) * pow(t, 5) + cy(1) * pow(t, 4) + cy(2) * pow(t, 3) + cy(3) * pow(t, 2) + cy(4) * t + cy(5);
            segment.emplace_back(px, py);
        }

        return segment;
    }
};

