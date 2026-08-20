#ifndef NAV2_CUSTOM_CONTROLLER__NAV2_CUSTOM_CONTROLLER_HPP_
#define NAV2_CUSTOM_CONTROLLER__NAV2_CUSTOM_CONTROLLER_HPP_

#include <memory>
#include <vector>
#include <string>
#include <random>
#include "nav2_core/controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_util/robot_utils.hpp"

namespace nav2_custom_controller{

struct TrajectoryPoint {
    double x, y, theta;
};

class CustomController : public nav2_core::Controller{
public:
    CustomController() = default;
    ~CustomController() override = default;
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
    ) override;
    void cleanup() override;
    void activate() override;
    void deactivate() override;
    geometry_msgs::msg::TwistStamped
    computeVelocityCommands(const geometry_msgs::msg::PoseStamped &pose,
                        const geometry_msgs::msg::Twist &velocity,
                        nav2_core::GoalChecker *goal_checker) override;
    void setPlan(const nav_msgs::msg::Path &path) override;
    void setSpeedLimit(const double &speed_limit,const bool &percentage) override;

protected:
    std::string plugin_name_;
    std::shared_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    nav2_util::LifecycleNode::SharedPtr node_;
    nav2_costmap_2d::Costmap2D *costmap_;
    nav_msgs::msg::Path global_plan_;

    double max_angular_speed_;
    double max_linear_speed_;

    int prediction_horizon_;
    double dt_;
    int num_samples_;
    double weight_path_;
    double weight_heading_;
    double weight_smooth_;

    std::mt19937 rng_;

    TrajectoryPoint propagate(const TrajectoryPoint &state, double v, double omega) const;
    double evaluateCost(const std::vector<TrajectoryPoint> &traj,
                        const std::vector<TrajectoryPoint> &ref,
                        const std::vector<double> &v_seq,
                        const std::vector<double> &omega_seq) const;
    std::vector<TrajectoryPoint> extractReferenceTrajectory(
        const TrajectoryPoint &current_state) const;
    double angleDiff(double a, double b) const;
};
}
#endif