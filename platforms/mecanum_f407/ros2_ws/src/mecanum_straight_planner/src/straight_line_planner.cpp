
#include <algorithm>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav2_core/planner_exceptions.hpp"

#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "nav2_costmap_2d/cost_values.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2/utils.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "pluginlib/class_list_macros.hpp"

namespace mecanum_straight_planner
{

class StraightLinePlanner : public nav2_core::GlobalPlanner
{
public:
    StraightLinePlanner() = default;
    ~StraightLinePlanner() override = default;

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> /*tf*/,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
    ) override
    {
        node_ = parent.lock();
        if (!node_) {
            throw std::runtime_error("Planner parent node expired");
        }

        name_ = name;
        costmap_ = costmap_ros->getCostmap();
        global_frame_ = costmap_ros->getGlobalFrameID();

        // 直线插值间隔，单位：米
        const auto res_key = name_ + ".interpolation_resolution";
        if (!node_->has_parameter(res_key)) {
            node_->declare_parameter<double>(res_key, 0.02);
        }
        node_->get_parameter(res_key, resolution_);

        // 第一阶段限制：只接受位于车体前方的目标
        const auto forward_key = name_ + ".forward_only";
        if (!node_->has_parameter(forward_key)) {
            node_->declare_parameter<bool>(forward_key, true);
        }
        node_->get_parameter(forward_key, forward_only_);

        // 目标允许偏离车体正前方的横向距离
        const auto lateral_key = name_ + ".lateral_tolerance";
        if (!node_->has_parameter(lateral_key)) {
            node_->declare_parameter<double>(lateral_key, 0.10);
        }
        node_->get_parameter(lateral_key, lateral_tolerance_);

        if (resolution_ <= 0.0 || lateral_tolerance_ < 0.0) {
            throw std::runtime_error("Invalid planner parameters");
        }

        RCLCPP_INFO(
            node_->get_logger(),
            "StraightLinePlanner configured: resolution=%.3f "
            "forward_only=%s lateral_tolerance=%.3f",
            resolution_,
            forward_only_ ? "true" : "false",
            lateral_tolerance_);
    }

    void activate() override
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "StraightLinePlanner activated");
    }

    void deactivate() override
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "StraightLinePlanner deactivated");
    }

    void cleanup() override
    {
        RCLCPP_INFO(
            node_->get_logger(),
            "StraightLinePlanner cleanup");

        costmap_ = nullptr;
        node_.reset();
    }


    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal,
        std::function<bool()> cancel_checker) override
    {
        if (!node_ || !costmap_) {
            throw nav2_core::PlannerException(
                "StraightLinePlanner is not configured");
        }

        if (start.header.frame_id != global_frame_ ||
            goal.header.frame_id != global_frame_)
        {
            throw nav2_core::PlannerTFError(
                "Start and goal must be in global frame");
        }

        const double x0 = start.pose.position.x;
        const double y0 = start.pose.position.y;

        const double x1 = goal.pose.position.x;
        const double y1 = goal.pose.position.y;

        const double dx = x1 - x0;
        const double dy = y1 - y0;

        const double distance = std::hypot(dx, dy);

        // 路径点间隔不能大于半个地图栅格
        // 防止采样距离过大导致漏检障碍
        const double step_size = std::min(
            resolution_,
            costmap_->getResolution() * 0.5);

        const int segments = std::max(
            1,
            static_cast<int>(
                std::ceil(distance / step_size)));

        // 直线方向
        const double yaw = (distance > 1e-6)
            ? std::atan2(dy, dx)
            : tf2::getYaw(start.pose.orientation);

        tf2::Quaternion q;
        q.setRPY(0.0, 0.0, yaw);
        q.normalize();

        nav_msgs::msg::Path path;
        path.header.frame_id = global_frame_;
        path.header.stamp = node_->now();
        path.poses.reserve(segments + 1);

        for (int i = 0; i <= segments; ++i)
        {
            if (cancel_checker && cancel_checker()) {
                throw nav2_core::PlannerCancelled(
                    "Planning cancelled");
            }

            const double t =
                static_cast<double>(i) / segments;

            // 核心：两点直线插值
            const double x = x0 + t * dx;
            const double y = y0 + t * dy;

            unsigned int mx = 0;
            unsigned int my = 0;

            if (!costmap_->worldToMap(x, y, mx, my)) {
                throw nav2_core::NoValidPathCouldBeFound(
                    "Straight path outside map");
            }

            const unsigned char cost =
                costmap_->getCost(mx, my);

            // 不允许直接穿过障碍或未知区域
            if (cost >=
                nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
            {
                throw nav2_core::NoValidPathCouldBeFound(
                    "Straight path blocked");
            }

            geometry_msgs::msg::PoseStamped p;
            p.header = path.header;

            p.pose.position.x = x;
            p.pose.position.y = y;
            p.pose.position.z =
                start.pose.position.z +
                t * (goal.pose.position.z -
                    start.pose.position.z);

            p.pose.orientation = tf2::toMsg(q);

            path.poses.push_back(p);
        }

        // 最终目标必须与传入的终点一致
        path.poses.back().pose = goal.pose;

        RCLCPP_INFO(
            node_->get_logger(),
            "StraightLinePlanner: A(%.3f, %.3f)"
            " -> B(%.3f, %.3f), length=%.3f, points=%zu",
            x0, y0, x1, y1,
            distance,
            path.poses.size());

        return path;
    }


private:
    rclcpp_lifecycle::LifecycleNode::SharedPtr node_;

    nav2_costmap_2d::Costmap2D * costmap_{nullptr};

    std::string name_;
    std::string global_frame_;

    double resolution_{0.02};
    double lateral_tolerance_{0.10};
    bool forward_only_{true};
};

}  // namespace mecanum_straight_planner

PLUGINLIB_EXPORT_CLASS(
    mecanum_straight_planner::StraightLinePlanner,
    nav2_core::GlobalPlanner)
