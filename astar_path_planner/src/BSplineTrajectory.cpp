#include <vector>
#include <Eigen/Dense>
#include <stdexcept>
#include <cmath>

typedef std::vector<Eigen::Vector2d> Path;

class BSplineTrajectoryGenerator {
public:
    BSplineTrajectoryGenerator() = default;

    Path generateTrajectory(const Path& waypoints, int degree, int resolution) {
//        ROS_INFO("[BSpline] Enter generateTrajectory with %zu waypoints", waypoints.size());
        if (waypoints.size() < degree + 1) {
            throw std::invalid_argument("Waypoints must be at least degree + 1.");
        }

        std::vector<double> knots = generateKnots(waypoints.size(), degree);
        Path controlPoints = optimizeControlPoints(waypoints, knots, degree);
        return evaluateBSpline(controlPoints, knots, degree, resolution);
    }

private:
    std::vector<double> generateKnots(size_t n, int degree) {
        size_t totalKnots = n + degree + 1;
        std::vector<double> knots(totalKnots);
        for (size_t i = 0; i < totalKnots; ++i) {
            if (i <= (size_t)degree) {
                knots[i] = 0.0;
            } else if (i >= totalKnots - degree - 1) {
                knots[i] = 1.0;
            } else {
                knots[i] = static_cast<double>(i - degree)
                         / static_cast<double>(totalKnots - 2 * degree - 1);
            }
        }
        return knots;
    }

    Path optimizeControlPoints(const Path& waypoints,
                               const std::vector<double>& knots,
                               int degree) {
        if (waypoints.size() < 3) {
            return waypoints;
        }

        Path controlPoints = waypoints;
        int num_iterations = 3;
        for (int iter = 0; iter < num_iterations; ++iter) {
            for (size_t i = 1; i + 1 < controlPoints.size(); ++i) {
                controlPoints[i] = 0.25 * controlPoints[i - 1]
                                 + 0.50 * controlPoints[i]
                                 + 0.25 * controlPoints[i + 1];
            }
        }

        return controlPoints;
    }

    Path evaluateBSpline(const Path& controlPoints,
                         const std::vector<double>& knots,
                         int degree,
                         int resolution) {
        Path trajectory;
        double t_min = knots.front();
        double t_max = knots.back();
        for (int i = 0; i <= resolution; ++i) {
            double t = t_min + i * (t_max - t_min) / (double)resolution;
            Eigen::Vector2d point = deBoor(degree, controlPoints, knots, t);
            trajectory.push_back(point);
        }
        return trajectory;
    }

    Eigen::Vector2d deBoor(int degree,
                           const Path& controlPoints,
                           const std::vector<double>& knots,
                           double t) {
        int k = findKnotIndex(knots, t, degree);
        std::vector<Eigen::Vector2d> d(degree + 1);
        for (int j = 0; j <= degree; ++j) {
            d[j] = controlPoints[k - degree + j];
        }

        for (int r = 1; r <= degree; ++r) {
            for (int j = degree; j >= r; --j) {
                double alpha = (t - knots[k - degree + j])
                             / (knots[k + 1 + j - r]
                              - knots[k - degree + j]);
                d[j] = (1.0 - alpha) * d[j - 1] + alpha * d[j];
            }
        }
        return d[degree];
    }

    int findKnotIndex(const std::vector<double>& knots, double t, int degree) {
        if (std::fabs(t - knots.back()) < 1e-9) {
            return (int)knots.size() - degree - 2;
        }

        for (int i = degree; i < (int)knots.size() - degree - 1; ++i) {
            if (t >= knots[i] && t < knots[i + 1]) {
                return i;
            }
        }
        return -1;
    }
};
