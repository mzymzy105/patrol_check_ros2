#ifndef NAV2_CUSTOM_PLANNER__TRAJECTORY_PROCESSOR_HPP_
#define NAV2_CUSTOM_PLANNER__TRAJECTORY_PROCESSOR_HPP_

#include <vector>
#include <cmath>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace nav2_custom_planner
{

struct TrajectoryPoint {
    double x, y;
    double theta;
    double curvature;
    double velocity;
};

class TrajectoryProcessor
{
public:
    TrajectoryProcessor() = default;
    ~TrajectoryProcessor() = default;

    void configure(rclcpp_lifecycle::LifecycleNode::SharedPtr node,
                   double max_linear_speed = 0.26,
                   double max_angular_speed = 1.0,
                   double max_curvature = 5.0);

    nav_msgs::msg::Path process(const nav_msgs::msg::Path &raw_path);

private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
    double max_linear_speed_;
    double max_angular_speed_;
    double max_curvature_;

    // void lineOfSightPruning(std::vector<geometry_msgs::msg::PoseStamped> &poses,
    //                         const nav_msgs::msg::Path &raw_path);

    void cubicSplineInterpolation(std::vector<geometry_msgs::msg::PoseStamped> &poses);

    void computeCurvature(std::vector<TrajectoryPoint> &traj_points,
                          const std::vector<geometry_msgs::msg::PoseStamped> &poses);

    void assignOrientation(std::vector<geometry_msgs::msg::PoseStamped> &poses);

    void computeVelocityProfile(std::vector<geometry_msgs::msg::PoseStamped> &poses,
                                const std::vector<TrajectoryPoint> &traj_points);

    double distance(const geometry_msgs::msg::PoseStamped &a,
                    const geometry_msgs::msg::PoseStamped &b) const;

    double angleDiff(double a, double b) const;
};

}  // namespace nav2_custom_planner

#endif  // NAV2_CUSTOM_PLANNER__TRAJECTORY_PROCESSOR_HPP_