#include "nav2_custom_controller/custom_controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <limits>

namespace nav2_custom_controller{

void CustomController::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros){
    node_ = parent.lock();
    costmap_ros_ = costmap_ros;
    tf_ = tf;
    plugin_name_ = name;
    costmap_ = costmap_ros_->getCostmap();

    nav2_util::declare_parameter_if_not_declared(
        node_, plugin_name_ + ".max_linear_speed",rclcpp::ParameterValue(0.26));
    node_->get_parameter(plugin_name_+".max_linear_speed",max_linear_speed_);
    nav2_util::declare_parameter_if_not_declared(
        node_,plugin_name_+".max_angular_speed",rclcpp::ParameterValue(1.0));
    node_->get_parameter(plugin_name_+".max_angular_speed",max_angular_speed_);

    nav2_util::declare_parameter_if_not_declared(
        node_,plugin_name_+".prediction_horizon",rclcpp::ParameterValue(10));
    node_->get_parameter(plugin_name_+".prediction_horizon",prediction_horizon_);
    nav2_util::declare_parameter_if_not_declared(
        node_,plugin_name_+".dt",rclcpp::ParameterValue(0.1));
    node_->get_parameter(plugin_name_+".dt",dt_);
    nav2_util::declare_parameter_if_not_declared(
        node_,plugin_name_+".num_samples",rclcpp::ParameterValue(150));
    node_->get_parameter(plugin_name_+".num_samples",num_samples_);
    nav2_util::declare_parameter_if_not_declared(
        node_,plugin_name_+".weight_path",rclcpp::ParameterValue(10.0));
    node_->get_parameter(plugin_name_+".weight_path",weight_path_);
    nav2_util::declare_parameter_if_not_declared(
        node_,plugin_name_+".weight_heading",rclcpp::ParameterValue(1.0));
    node_->get_parameter(plugin_name_+".weight_heading",weight_heading_);
    nav2_util::declare_parameter_if_not_declared(
        node_,plugin_name_+".weight_smooth",rclcpp::ParameterValue(0.5));
    node_->get_parameter(plugin_name_+".weight_smooth",weight_smooth_);

    std::random_device rd;
    rng_ = std::mt19937(rd());

    RCLCPP_INFO(node_->get_logger(),"MPC控制器 %s 已配置: horizon=%d dt=%.2f samples=%d",
                plugin_name_.c_str(),prediction_horizon_,dt_,num_samples_);
}

void CustomController::cleanup(){
    RCLCPP_INFO(node_->get_logger(),"正在清理MPC控制器插件%s",plugin_name_.c_str());
}

void CustomController::activate(){
    RCLCPP_INFO(node_->get_logger(),"正在激活MPC控制器插件%s",plugin_name_.c_str());
}

void CustomController::deactivate(){
    RCLCPP_INFO(node_->get_logger(),"正在停用MPC控制器插件%s",plugin_name_.c_str());
}

/**
 * @brief 状态传播函数: 根据当前状态和控制输入预测下一时刻的状态
 * @param state 当前轨迹点状态(包含x, y, theta)
 * @param v 线速度控制输入(m/s)
 * @param omega 角速度控制输入(rad/s)
 * @return 下一时刻的轨迹点状态
 *
 * @details 基于差速驱动机器人运动学模型的离散化传播:
 *          - 位置更新: x_{k+1} = x_k + v·cos(θ_k)·dt
 *                     y_{k+1} = y_k + v·sin(θ_k)·dt
 *          - 朝向更新: θ_{k+1} = θ_k + ω·dt
 *          - 角度归一化: 将θ限制在[-π, π]范围内,避免角度溢出
 *
 *          该函数是MPC预测模型的核心,通过多次调用可生成完整的预测轨迹
 */
TrajectoryPoint CustomController::propagate(const TrajectoryPoint &state,
                                             double v, double omega) const {
    TrajectoryPoint next;
    // 位置更新: 假设机器人在dt时间内以恒定速度v沿当前朝向θ移动
    next.x = state.x + v * std::cos(state.theta) * dt_;
    next.y = state.y + v * std::sin(state.theta) * dt_;
    // 朝向更新: 角速度积分
    next.theta = state.theta + omega * dt_;
    // 角度归一化: 将theta限制在[-π, π]范围内,防止角度值无限增长
    next.theta = std::atan2(std::sin(next.theta), std::cos(next.theta));
    return next;
}

double CustomController::angleDiff(double a, double b) const {
    double d = a - b;
    while (d > M_PI) d -= 2.0 * M_PI;
    while (d < -M_PI) d += 2.0 * M_PI;
    return d;
}

/**
 * @brief 评估预测轨迹的代价函数
 * @param traj 预测轨迹点序列,由propagate函数生成
 * @param ref 参考轨迹点序列,从全局路径中提取
 * @param v_seq 线速度控制输入序列
 * @param omega_seq 角速度控制输入序列
 * @return 总代价值,越小表示轨迹质量越好
 * 
 * @details 代价函数由三部分组成:
 *          1. 路径跟踪代价: 衡量预测轨迹与参考轨迹的位置偏差
 *          2. 朝向跟踪代价: 衡量预测轨迹与参考轨迹的朝向偏差
 *          3. 平滑性代价: 惩罚速度突变,保证控制输入的平滑性
 *          总代价 J = Σ[w_path*(Δx²+Δy²) + w_heading*Δθ² + w_smooth*(Δv²+Δω²)]
 */
double CustomController::evaluateCost(const std::vector<TrajectoryPoint> &traj,
                                       const std::vector<TrajectoryPoint> &ref,
                                       const std::vector<double> &v_seq,
                                       const std::vector<double> &omega_seq) const {
    double cost = 0.0;
    
    for (size_t k = 0; k < traj.size(); ++k) {
        double dx = traj[k].x - ref[k].x;
        double dy = traj[k].y - ref[k].y;
        cost += weight_path_ * (dx * dx + dy * dy);
        
        double dtheta = angleDiff(traj[k].theta, ref[k].theta);
        cost += weight_heading_ * dtheta * dtheta;
        
        if (k > 0) {
            double dv = v_seq[k] - v_seq[k-1];
            double dw = omega_seq[k] - omega_seq[k-1];
            cost += weight_smooth_ * (dv * dv + dw * dw);
        }
    }
    
    return cost;
}

std::vector<TrajectoryPoint> CustomController::extractReferenceTrajectory(
        const TrajectoryPoint &current_state) const {
    std::vector<TrajectoryPoint> ref;
    if (global_plan_.poses.empty()) return ref;

    using nav2_util::geometry_utils::euclidean_distance;
    size_t nearest_idx = 0;
    double min_dist = std::numeric_limits<double>::max();
    //寻找目前距离小车最近的规划点
    for (size_t i = 0; i < global_plan_.poses.size(); ++i) {
        double dist = std::hypot(
            global_plan_.poses[i].pose.position.x - current_state.x,
            global_plan_.poses[i].pose.position.y - current_state.y);
        if (dist < min_dist) { min_dist = dist; nearest_idx = i; }
    }

    double ref_distance = max_linear_speed_ * dt_;
    for (int k = 0; k < prediction_horizon_; ++k) {
        double target_dist = (k + 1) * ref_distance;
        double accumulated = 0.0;
        size_t idx = nearest_idx;
        for (size_t i = nearest_idx; i + 1 < global_plan_.poses.size(); ++i) {
            double seg = std::hypot(
                global_plan_.poses[i+1].pose.position.x - global_plan_.poses[i].pose.position.x,
                global_plan_.poses[i+1].pose.position.y - global_plan_.poses[i].pose.position.y);
            if (accumulated + seg >= target_dist) {
                double ratio = (target_dist - accumulated) / seg;
                TrajectoryPoint pt;
                pt.x = global_plan_.poses[i].pose.position.x + ratio *
                    (global_plan_.poses[i+1].pose.position.x - global_plan_.poses[i].pose.position.x);
                pt.y = global_plan_.poses[i].pose.position.y + ratio *
                    (global_plan_.poses[i+1].pose.position.y - global_plan_.poses[i].pose.position.y);
                pt.theta = std::atan2(
                    global_plan_.poses[i+1].pose.position.y - global_plan_.poses[i].pose.position.y,
                    global_plan_.poses[i+1].pose.position.x - global_plan_.poses[i].pose.position.x);
                ref.push_back(pt);
                idx = i;
                break;
            }
            accumulated += seg;
            idx = i + 1;
        }
        if (idx + 1 >= global_plan_.poses.size()) {
            TrajectoryPoint pt;
            pt.x = global_plan_.poses.back().pose.position.x;
            pt.y = global_plan_.poses.back().pose.position.y;
            pt.theta = (ref.empty() ? 0.0 : ref.back().theta);
            ref.push_back(pt);
        }
    }
    return ref;
}

geometry_msgs::msg::TwistStamped CustomController::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &pose,
        const geometry_msgs::msg::Twist &, nav2_core::GoalChecker *){
    if (global_plan_.poses.empty()){
        throw nav2_core::PlannerException("收到长度为零的路径");
    }

    geometry_msgs::msg::PoseStamped pose_in_globalframe;
    if(!nav2_util::transformPoseInTargetFrame(pose,pose_in_globalframe,*tf_,global_plan_.header.frame_id,0.1)) {
        throw nav2_core::PlannerException("无法将机器人姿态转换为全局计划的坐标系");
    }

    TrajectoryPoint current_state;
    current_state.x = pose_in_globalframe.pose.position.x;
    current_state.y = pose_in_globalframe.pose.position.y;
    current_state.theta = tf2::getYaw(pose_in_globalframe.pose.orientation);

    auto ref_traj = extractReferenceTrajectory(current_state);
    if (ref_traj.size() < 2) {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = pose_in_globalframe.header.frame_id;
        cmd_vel.header.stamp = node_->get_clock()->now();
        return cmd_vel;
    }

    std::uniform_real_distribution<double> v_dist(0.0, max_linear_speed_);
    std::uniform_real_distribution<double> omega_dist(-max_angular_speed_, max_angular_speed_);

    double best_v = 0.0, best_omega = 0.0, best_cost = std::numeric_limits<double>::max();

    for (int s = 0; s < num_samples_; ++s) {
        std::vector<double> v_seq(prediction_horizon_);
        std::vector<double> omega_seq(prediction_horizon_);
        for (int k = 0; k < prediction_horizon_; ++k) {
            v_seq[k] = v_dist(rng_);
            omega_seq[k] = omega_dist(rng_);
        }

        std::vector<TrajectoryPoint> traj;
        TrajectoryPoint st = current_state;
        for (int k = 0; k < prediction_horizon_; ++k) {
            st = propagate(st, v_seq[k], omega_seq[k]);
            traj.push_back(st);
        }

        double cost = evaluateCost(traj, ref_traj, v_seq, omega_seq);
        if (cost < best_cost) {
            best_cost = cost;
            best_v = v_seq[0];
            best_omega = omega_seq[0];
        }
    }

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.frame_id = pose_in_globalframe.header.frame_id;
    cmd_vel.header.stamp = node_->get_clock()->now();
    cmd_vel.twist.linear.x = best_v;
    cmd_vel.twist.angular.z = best_omega;

    RCLCPP_INFO(node_->get_logger(),"MPC控制器 %s: v=%.3f omega=%.3f cost=%.2f",
                plugin_name_.c_str(),best_v,best_omega,best_cost);
    return cmd_vel;
}

void CustomController::setSpeedLimit(const double &speed_limt,const bool &percentage){
    (void)percentage;
    (void)speed_limt;
}

void CustomController::setPlan(const nav_msgs::msg::Path &path){
    global_plan_ = path;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_custom_controller::CustomController,nav2_core::Controller)