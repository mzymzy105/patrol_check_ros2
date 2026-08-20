#ifndef NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_
#define NAV2_CUSTOM_PLANNER__NAV2_CUSTOM_PLANNER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <unordered_map>
#include <cmath>
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_core/global_planner.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_custom_planner/trajectory_processor.hpp"

namespace nav2_custom_planner{

struct AStarNode {
    unsigned int x, y;
    double g_cost;
    double h_cost;
    AStarNode* parent;
    bool closed;

    double f_cost() const { return g_cost + h_cost; }

    struct Compare {
        bool operator()(const AStarNode* a, const AStarNode* b) const {
            return a->f_cost() > b->f_cost();
        }
    };
};

class CustomPlanner:public nav2_core::GlobalPlanner{
public:
    CustomPlanner() = default;
    ~CustomPlanner() = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;
    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path
    createPlan(const geometry_msgs::msg::PoseStamped &start,const geometry_msgs::msg::PoseStamped &goal) override;

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    nav2_util::LifecycleNode::SharedPtr node_;
    nav2_costmap_2d::Costmap2D *costmap_;
    std::string global_frame_,name_;
    double tolerance_;
    bool use_8connected_;
    bool smooth_path_;
    double cost_weight_;

    std::vector<AStarNode> node_pool_;

    TrajectoryProcessor trajectory_processor_;
    bool enable_trajectory_processing_;
    double max_linear_speed_;
    double max_angular_speed_;
    double max_curvature_;

    unsigned int hash(unsigned int x, unsigned int y) const;
    AStarNode* getNode(unsigned int x, unsigned int y);
    void resetNodePool();

    double heuristic(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) const;
    double movementCost(unsigned int x1, unsigned int y1, unsigned int x2, unsigned int y2) const;
    bool isValidCell(unsigned int x, unsigned int y) const;

    nav_msgs::msg::Path reconstructPath(AStarNode* goal_node,
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) const;
    void smoothPath(nav_msgs::msg::Path &path) const;
    bool isLineFree(const geometry_msgs::msg::PoseStamped &a,
                    const geometry_msgs::msg::PoseStamped &b) const;
};
}
#endif