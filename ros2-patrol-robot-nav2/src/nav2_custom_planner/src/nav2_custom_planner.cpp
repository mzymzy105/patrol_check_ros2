#include "nav2_util/node_utils.hpp"
#include <cmath>
#include <memory>
#include <string>
#include <limits>

#include "nav2_core/exceptions.hpp"
#include "nav2_custom_planner/nav2_custom_planner.hpp"

namespace nav2_custom_planner{

void CustomPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent ,std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros){
    tf_buffer = tf;
    node_ = parent.lock();
    name_ = name;
    costmap_ = costmap_ros->getCostmap();
    global_frame_ = costmap_ros->getGlobalFrameID();

    nav2_util::declare_parameter_if_not_declared(
        node_,name_+".tolerance",rclcpp::ParameterValue(0.5));
    node_->get_parameter(name_+".tolerance",tolerance_);
    nav2_util::declare_parameter_if_not_declared(
        node_,name_+".use_8connected",rclcpp::ParameterValue(true));
    node_->get_parameter(name_+".use_8connected",use_8connected_);
    nav2_util::declare_parameter_if_not_declared(
        node_,name_+".smooth_path",rclcpp::ParameterValue(true));
    node_->get_parameter(name_+".smooth_path",smooth_path_);
    nav2_util::declare_parameter_if_not_declared(
        node_,name_+".cost_weight",rclcpp::ParameterValue(1.0));
    node_->get_parameter(name_+".cost_weight",cost_weight_);

    nav2_util::declare_parameter_if_not_declared(
        node_,name_+".enable_trajectory_processing",rclcpp::ParameterValue(true));
    node_->get_parameter(name_+".enable_trajectory_processing",enable_trajectory_processing_);
    nav2_util::declare_parameter_if_not_declared(
        node_,name_+".max_linear_speed",rclcpp::ParameterValue(0.26));
    node_->get_parameter(name_+".max_linear_speed",max_linear_speed_);
    nav2_util::declare_parameter_if_not_declared(
        node_,name_+".max_angular_speed",rclcpp::ParameterValue(1.0));
    node_->get_parameter(name_+".max_angular_speed",max_angular_speed_);
    nav2_util::declare_parameter_if_not_declared(
        node_,name_+".max_curvature",rclcpp::ParameterValue(5.0));
    node_->get_parameter(name_+".max_curvature",max_curvature_);

    trajectory_processor_.configure(
        node_, max_linear_speed_, max_angular_speed_, max_curvature_);

    unsigned int size_x = costmap_->getSizeInCellsX();
    unsigned int size_y = costmap_->getSizeInCellsY();
    node_pool_.resize(size_x * size_y);
    for (unsigned int i = 0; i < size_x * size_y; ++i) {
        node_pool_[i].closed = false;
        node_pool_[i].parent = nullptr;
    }

    RCLCPP_INFO(node_->get_logger(),"A*规划器 %s 已配置: 8connected=%d smooth=%d",
                name_.c_str(),use_8connected_,smooth_path_);
}

void CustomPlanner::cleanup(){
    RCLCPP_INFO(node_->get_logger(),"正在清理A*规划器插件%s",name_.c_str());
}

void CustomPlanner::activate(){
    RCLCPP_INFO(node_->get_logger(),"正在激活A*规划器插件%s",name_.c_str());
}

void CustomPlanner::deactivate(){
    RCLCPP_INFO(node_->get_logger(),"正在停用A*规划器插件%s",name_.c_str());
}

unsigned int CustomPlanner::hash(unsigned int x, unsigned int y) const {
    return y * costmap_->getSizeInCellsX() + x;
}

AStarNode* CustomPlanner::getNode(unsigned int x, unsigned int y) {
    return &node_pool_[hash(x, y)];
}

void CustomPlanner::resetNodePool() {
    for (auto &node : node_pool_) {
        node.closed = false;
        node.parent = nullptr;
        node.g_cost = std::numeric_limits<double>::max();
    }
}

/**
 * @brief A*启发式函数,估算从当前节点到目标节点的代价
 * @param x1 当前节点的x坐标(网格坐标)
 * @param y1 当前节点的y坐标(网格坐标)
 * @param x2 目标节点的x坐标(网格坐标)
 * @param y2 目标节点的y坐标(网格坐标)
 * @return 启发式估计代价
 * 
 * @details 根据搜索模式选择不同的距离度量:
 *          - 8连通模式: 使用八向距离(Octile Distance),允许斜向移动
 *            公式: max(dx,dy) + (√2-1)*min(dx,dy) = (dx+dy) + (√2-2)*min(dx,dy)
 *          - 4连通模式: 使用曼哈顿距离(Manhattan Distance),仅允许上下左右移动
 *            公式: dx + dy
 *          该启发式函数满足可采纳性(admissible)和一致性(consistent),
 *          保证A*算法能找到最优路径
 */
double CustomPlanner::heuristic(unsigned int x1, unsigned int y1,
                                 unsigned int x2, unsigned int y2) const {
    double dx = std::abs(static_cast<double>(x1) - static_cast<double>(x2));
    double dy = std::abs(static_cast<double>(y1) - static_cast<double>(y2));
    if (use_8connected_) {
        return (dx + dy) + (std::sqrt(2.0) - 2.0) * std::min(dx, dy);
    }
    return dx + dy;
}

double CustomPlanner::movementCost(unsigned int x1, unsigned int y1,
                                    unsigned int x2, unsigned int y2) const {
    double base_cost = (x1 != x2 && y1 != y2) ? std::sqrt(2.0) : 1.0;
    unsigned char cell_cost = costmap_->getCost(x2, y2);
    double cost_factor = 1.0 + cost_weight_ * (static_cast<double>(cell_cost) / 255.0);
    return base_cost * cost_factor;
}

bool CustomPlanner::isValidCell(unsigned int x, unsigned int y) const {
    if (x >= costmap_->getSizeInCellsX() || y >= costmap_->getSizeInCellsY()) {
        return false;
    }
    unsigned char cost = costmap_->getCost(x, y);
    return cost < nav2_costmap_2d::LETHAL_OBSTACLE;
}

bool CustomPlanner::isLineFree(const geometry_msgs::msg::PoseStamped &a,
                                const geometry_msgs::msg::PoseStamped &b) const {
    double dx = b.pose.position.x - a.pose.position.x;
    double dy = b.pose.position.y - a.pose.position.y;
    double dist = std::hypot(dx, dy);
    double resolution = costmap_->getResolution();  //网格分辨率
    int steps = static_cast<int>(dist / (resolution * 0.5));
    for (int i = 1; i < steps; ++i) {
        double t = static_cast<double>(i) / steps;
        double wx = a.pose.position.x + t * dx;
        double wy = a.pose.position.y + t * dy;
        unsigned int mx, my;
        if (costmap_->worldToMap(wx, wy, mx, my)) {
            if (costmap_->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                return false;
            }
        }
    }
    return true;
}

nav_msgs::msg::Path CustomPlanner::reconstructPath(AStarNode* goal_node,
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal) const {
    nav_msgs::msg::Path path;
    path.header.stamp = node_->now();
    path.header.frame_id = global_frame_;

    std::vector<geometry_msgs::msg::PoseStamped> poses;
    AStarNode* current = goal_node;
    while (current != nullptr) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.stamp = node_->now();
        pose.header.frame_id = global_frame_;
        double wx, wy;
        costmap_->mapToWorld(current->x, current->y, wx, wy);
        pose.pose.position.x = wx;
        pose.pose.position.y = wy;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.0;
        poses.push_back(pose);
        current = current->parent;
    }
    std::reverse(poses.begin(), poses.end());

    path.poses.push_back(start);
    for (auto &p : poses) {
        path.poses.push_back(p);
    }
    path.poses.push_back(goal);
    return path;
}

void CustomPlanner::smoothPath(nav_msgs::msg::Path &path) const {
    if (path.poses.size() <= 3) return;

    std::vector<geometry_msgs::msg::PoseStamped> smoothed;
    smoothed.push_back(path.poses.front());

    size_t current_idx = 0;
    while (current_idx < path.poses.size() - 1) {
        size_t furthest = current_idx + 1;
        for (size_t i = path.poses.size() - 1; i > current_idx + 1; --i) {
            if (isLineFree(path.poses[current_idx], path.poses[i])) {
                furthest = i;
                break;
            }
        }
        smoothed.push_back(path.poses[furthest]);
        current_idx = furthest;
    }

    path.poses = smoothed;
}

nav_msgs::msg::Path
CustomPlanner::createPlan(const geometry_msgs::msg::PoseStamped &start,
                           const geometry_msgs::msg::PoseStamped &goal){
    nav_msgs::msg::Path global_path;
    global_path.poses.clear();
    global_path.header.stamp = node_->now();
    global_path.header.frame_id = global_frame_;

    if (start.header.frame_id != global_frame_ || goal.header.frame_id != global_frame_){
        RCLCPP_ERROR(node_->get_logger(),"A*规划器仅接受来自%s坐标系的位姿",global_frame_.c_str());
        return global_path;
    }

    unsigned int sx, sy, gx, gy;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) ||
        !costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy)) {
        RCLCPP_ERROR(node_->get_logger(),"起点或目标点超出代价地图范围");
        return global_path;
    }

    if (!isValidCell(sx, sy)) {
        RCLCPP_ERROR(node_->get_logger(),"起点位于障碍物或未知区域");
        return global_path;
    }
    if (!isValidCell(gx, gy)) {
        RCLCPP_ERROR(node_->get_logger(),"目标点位于障碍物或未知区域");
        return global_path;
    }

    RCLCPP_INFO(node_->get_logger(),"A*规划: (%d,%d)->(%d,%d)",sx,sy,gx,gy);

    resetNodePool();

    std::priority_queue<AStarNode*, std::vector<AStarNode*>, AStarNode::Compare> open_list;

    AStarNode* start_node = getNode(sx, sy);
    start_node->x = sx;
    start_node->y = sy;
    start_node->g_cost = 0.0;
    start_node->h_cost = heuristic(sx, sy, gx, gy);
    start_node->parent = nullptr;
    open_list.push(start_node);

    const int dirs_8[8][2] = {{1,0},{0,1},{-1,0},{0,-1},{1,1},{1,-1},{-1,1},{-1,-1}};
    const int dirs_4[4][2] = {{1,0},{0,1},{-1,0},{0,-1}};
    int dir_count = use_8connected_ ? 8 : 4;
    const int (*dirs)[2] = use_8connected_ ? dirs_8 : dirs_4;

    bool found = false;
    AStarNode* goal_node = nullptr;

    while (!open_list.empty()) {
        AStarNode* current = open_list.top();
        open_list.pop();

        if (current->closed) continue;
        current->closed = true;

        if (current->x == gx && current->y == gy) {
            found = true;
            goal_node = current;
            break;
        }

        for (int d = 0; d < dir_count; ++d) {
            int nx = static_cast<int>(current->x) + dirs[d][0];
            int ny = static_cast<int>(current->y) + dirs[d][1];

            // 检查邻居是否超出代价地图范围
            if (nx < 0 || ny < 0 ||
                nx >= static_cast<int>(costmap_->getSizeInCellsX()) ||
                ny >= static_cast<int>(costmap_->getSizeInCellsY())) {
                continue;
            }

            unsigned int ux = static_cast<unsigned int>(nx);
            unsigned int uy = static_cast<unsigned int>(ny);

            if (!isValidCell(ux, uy)) continue;

            AStarNode* neighbor = getNode(ux, uy);
            if (neighbor->closed) continue;

            double move_cost = movementCost(current->x, current->y, ux, uy);
            if (move_cost >= std::numeric_limits<double>::max()) continue;

            double new_g = current->g_cost + move_cost;
            if (new_g < neighbor->g_cost) {
                neighbor->g_cost = new_g;
                neighbor->h_cost = heuristic(ux, uy, gx, gy);
                neighbor->parent = current;
                neighbor->x = ux;
                neighbor->y = uy;
                open_list.push(neighbor);
            }
        }
    }

    if (!found) {
        RCLCPP_WARN(node_->get_logger(),"A*规划失败: 无法找到从起点到目标点的路径");
        throw nav2_core::PlannerException("A*无法创建目标规划");
    }

    global_path = reconstructPath(goal_node, start, goal);

    if (smooth_path_) {
        smoothPath(global_path);
    }

    if (enable_trajectory_processing_) {
        global_path = trajectory_processor_.process(global_path);
    }

    RCLCPP_INFO(node_->get_logger(),"A*规划成功: 路径包含 %zu 个点",global_path.poses.size());
    return global_path;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_planner::CustomPlanner,nav2_core::GlobalPlanner)