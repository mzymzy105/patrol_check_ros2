#include "nav2_custom_planner/trajectory_processor.hpp"
#include <algorithm>
#include <limits>

namespace nav2_custom_planner
{

void TrajectoryProcessor::configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr node,
    double max_linear_speed,
    double max_angular_speed,
    double max_curvature)
{
    node_ = node;
    max_linear_speed_ = max_linear_speed;
    max_angular_speed_ = max_angular_speed;
    max_curvature_ = max_curvature;
}

double TrajectoryProcessor::distance(
    const geometry_msgs::msg::PoseStamped &a,
    const geometry_msgs::msg::PoseStamped &b) const
{
    double dx = b.pose.position.x - a.pose.position.x;
    double dy = b.pose.position.y - a.pose.position.y;
    return std::hypot(dx, dy);
}

double TrajectoryProcessor::angleDiff(double a, double b) const
{
    double d = a - b;
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return d;
}


/**
 * @brief 使用弧长参数化的自然三次样条插值将稀疏路径点转换为平滑密集曲线
 * @param poses 输入/输出: 稀疏路径点(输入) → 密集平滑轨迹点(输出)
 *
 * @details 算法原理:
 *          1. 计算沿路径的累积弧长t作为参数(天然单调递增)
 *          2. 对x(t)和y(t)分别构建三次样条:
 *             x(t) = ax + bx·Δt + cx·Δt² + dx·Δt³
 *             y(t) = ay + by·Δt + cy·Δt² + dy·Δt³
 *          3. 使用Thomas算法(追赶法)求解三对角线性方程组,时间复杂度O(n)
 *          4. 按弧长2cm间距均匀重采样生成密集轨迹点
 *
 * @note 输入要求:
 *       - 至少3个路径点(否则无法构建样条)
 *       - 弧长参数化保证参数单调递增,无需x坐标单调
 *       - 假设平面运动(z=0)
 */
void TrajectoryProcessor::cubicSplineInterpolation(
    std::vector<geometry_msgs::msg::PoseStamped> &poses)
{
    if (poses.size() < 3) return;

    size_t n = poses.size();

    // ========== 第一阶段: 基于弧长参数化 ==========
    std::vector<double> t(n, 0.0);
    for (size_t i = 1; i < n; ++i) {
        double dx = poses[i].pose.position.x - poses[i - 1].pose.position.x;
        double dy = poses[i].pose.position.y - poses[i - 1].pose.position.y;
        t[i] = t[i - 1] + std::hypot(dx, dy);
    }
    double total_length = t[n - 1];
    if (total_length < 1e-6) return;

    // ========== 第二阶段: 对x(t)和y(t)分别做三次样条插值 ==========
    std::vector<double> xs(n), ys(n);
    for (size_t i = 0; i < n; ++i) {
        xs[i] = poses[i].pose.position.x;
        ys[i] = poses[i].pose.position.y;
    }

    auto solveSpline = [&](const std::vector<double> &vals,
                           std::vector<double> &a_out,
                           std::vector<double> &b_out,
                           std::vector<double> &c_out,
                           std::vector<double> &d_out) {
        a_out.resize(n);
        b_out.resize(n);
        c_out.resize(n, 0.0);
        d_out.resize(n);
        for (size_t i = 0; i < n; ++i) a_out[i] = vals[i];

        std::vector<double> h(n - 1);
        for (size_t i = 0; i < n - 1; ++i) h[i] = t[i + 1] - t[i];

        std::vector<double> alpha(n, 0.0);
        for (size_t i = 1; i < n - 1; ++i) {
            alpha[i] = (3.0 / h[i]) * (a_out[i + 1] - a_out[i]) -
                       (3.0 / h[i - 1]) * (a_out[i] - a_out[i - 1]);
        }

        std::vector<double> l(n, 1.0), mu(n, 0.0), z(n, 0.0);
        for (size_t i = 1; i < n - 1; ++i) {
            l[i] = 2.0 * (t[i + 1] - t[i - 1]) - h[i - 1] * mu[i - 1];
            mu[i] = h[i] / l[i];
            z[i] = (alpha[i] - h[i - 1] * z[i - 1]) / l[i];
        }

        for (int i = n - 2; i >= 0; --i) {
            c_out[i] = z[i] - mu[i] * c_out[i + 1];
            b_out[i] = (a_out[i + 1] - a_out[i]) / h[i] - h[i] * (c_out[i + 1] + 2.0 * c_out[i]) / 3.0;
            d_out[i] = (c_out[i + 1] - c_out[i]) / (3.0 * h[i]);
        }
    };

    std::vector<double> ax, bx, cx, dx;
    std::vector<double> ay, by, cy, dy;
    solveSpline(xs, ax, bx, cx, dx);
    solveSpline(ys, ay, by, cy, dy);

    // ========== 第三阶段: 均匀重采样 ==========
    double interval = 0.02;
    int num_samples = std::max(static_cast<int>(total_length / interval), 2);

    std::vector<geometry_msgs::msg::PoseStamped> interpolated;
    for (int i = 0; i < num_samples; ++i) {
        double s = total_length * static_cast<double>(i) / (num_samples - 1);

        size_t seg = 0;
        for (size_t j = 0; j < n - 1; ++j) {
            if (s >= t[j] && s <= t[j + 1]) {
                seg = j;
                break;
            }
        }
        if (s > t[n - 1]) seg = n - 2;

        double dt = s - t[seg];
        double ix = ax[seg] + bx[seg] * dt + cx[seg] * dt * dt + dx[seg] * dt * dt * dt;
        double iy = ay[seg] + by[seg] * dt + cy[seg] * dt * dt + dy[seg] * dt * dt * dt;

        geometry_msgs::msg::PoseStamped pt;
        pt.header = poses[0].header;
        pt.pose.position.x = ix;
        pt.pose.position.y = iy;
        pt.pose.position.z = 0.0;
        interpolated.push_back(pt);
    }

    poses = interpolated;
}

void TrajectoryProcessor::computeCurvature(
    std::vector<TrajectoryPoint> &traj_points,
    const std::vector<geometry_msgs::msg::PoseStamped> &poses)
{
    traj_points.clear();// 清空结果向量
    if (poses.size() < 3) {  //少于3个点,无法计算曲率
        for (const auto &p : poses) {
            TrajectoryPoint tp;
            tp.x = p.pose.position.x;
            tp.y = p.pose.position.y;
            tp.theta = 0.0;
            tp.curvature = 0.0;
            tp.velocity = 0.0;
            traj_points.push_back(tp);
        }
        return;
    }

    for (size_t i = 0; i < poses.size(); ++i) {
        TrajectoryPoint tp;
        tp.x = poses[i].pose.position.x;
        tp.y = poses[i].pose.position.y;

        if (i == 0 || i == poses.size() - 1) {
            //首尾曲率简化为0
            tp.curvature = 0.0;
        } else {
            double x0 = poses[i - 1].pose.position.x;
            double y0 = poses[i - 1].pose.position.y;
            double x1 = poses[i].pose.position.x;
            double y1 = poses[i].pose.position.y;
            double x2 = poses[i + 1].pose.position.x;
            double y2 = poses[i + 1].pose.position.y;

            double a = std::hypot(x1 - x0, y1 - y0);
            double b = std::hypot(x2 - x1, y2 - y1);
            double c = std::hypot(x2 - x0, y2 - y0);

            double s = (a + b + c) / 2.0;
            double area = std::sqrt(std::max(0.0, s * (s - a) * (s - b) * (s - c)));
            if (a * b * c > 1e-10) {
                tp.curvature = (4.0 * area) / (a * b * c);
            } else {
                tp.curvature = 0.0;
            }
        }

        if (tp.curvature > max_curvature_) {
            tp.curvature = max_curvature_;
        }

        traj_points.push_back(tp);
    }
}

void TrajectoryProcessor::assignOrientation(
    std::vector<geometry_msgs::msg::PoseStamped> &poses)
{
    for (size_t i = 0; i < poses.size(); ++i) {
        double theta = 0.0;
        if (i < poses.size() - 1) {
            double dx = poses[i + 1].pose.position.x - poses[i].pose.position.x;
            double dy = poses[i + 1].pose.position.y - poses[i].pose.position.y;
            theta = std::atan2(dy, dx);
        } else if (i > 0) {
            double dx = poses[i].pose.position.x - poses[i - 1].pose.position.x;
            double dy = poses[i].pose.position.y - poses[i - 1].pose.position.y;
            theta = std::atan2(dy, dx);
        }
        poses[i].pose.orientation.z = std::sin(theta / 2.0);
        poses[i].pose.orientation.w = std::cos(theta / 2.0);
    }
}

void TrajectoryProcessor::computeVelocityProfile(
    std::vector<geometry_msgs::msg::PoseStamped> &poses,
    const std::vector<TrajectoryPoint> &traj_points)
{
    if (poses.empty()) return;

    for (size_t i = 0; i < poses.size(); ++i) {
        double curvature = (i < traj_points.size()) ? traj_points[i].curvature : 0.0;

        double velocity = max_linear_speed_;
        if (curvature > 1e-6) {
            double curvature_velocity = max_angular_speed_ / curvature;
            velocity = std::min(velocity, curvature_velocity);
        }

        if (i == 0 || i == poses.size() - 1) {
            velocity = std::min(velocity, max_linear_speed_ * 0.3);
        }

        velocity = std::max(velocity, 0.05);
        velocity = std::min(velocity, max_linear_speed_);

        poses[i].pose.position.z = velocity;
    }
}

nav_msgs::msg::Path TrajectoryProcessor::process(const nav_msgs::msg::Path &raw_path)
{
    if (raw_path.poses.empty()) {
        return raw_path;
    }

    std::vector<geometry_msgs::msg::PoseStamped> poses = raw_path.poses;

    cubicSplineInterpolation(poses);

    std::vector<TrajectoryPoint> traj_points;
    computeCurvature(traj_points, poses);

    assignOrientation(poses);

    computeVelocityProfile(poses, traj_points);

    nav_msgs::msg::Path processed_path;
    processed_path.header = raw_path.header;
    processed_path.poses = poses;

    RCLCPP_INFO(node_->get_logger(),
                "轨迹处理完成: 原始路径 %zu 点 -> 处理后 %zu 点",
                raw_path.poses.size(), processed_path.poses.size());

    return processed_path;
}

}  // namespace nav2_custom_planner