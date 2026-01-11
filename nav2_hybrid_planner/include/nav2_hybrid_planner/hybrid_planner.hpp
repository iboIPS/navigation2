#ifndef NAV2_HYBRID_PLANNER__HYBRID_PLANNER_HPP_
#define NAV2_HYBRID_PLANNER__HYBRID_PLANNER_HPP_

#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "nav2_core/global_planner.hpp"
#include <nav_msgs/msg/path.hpp>
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

namespace nav2_hybrid_planner
{
struct Cell;
struct CellHash;
struct node;
struct NodeCompare;

class HybridPlanner : public nav2_core::GlobalPlanner
{
public:
    HybridPlanner() = default;
    ~HybridPlanner() = default;

    // Plugin configure 
    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_rs
    ) override;

    void cleanup() override;
    void activate() override;
    void deactivate() override;

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped & start,
        const geometry_msgs::msg::PoseStamped & goal,
        std::function<bool()> cancel_checker
    ) override;

private:
    inline bool isFree(int x, int y) const;
    inline double heuristic(const Cell & a, const Cell & b);
    std::vector<std::vector<double>> computeDistanceTransform();

    rcl_interfaces::msg::SetParametersResult onParamChange(
        const std::vector<rclcpp::Parameter> & param);

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

    std::shared_ptr<tf2_ros::Buffer> tf_;
    nav2_util::LifecycleNode::SharedPtr node_;
    nav2_costmap_2d::Costmap2D * costmap_;
    std::string global_farme_, name_;
    
    double w_length_;
    double w_safety_;
    double l_ref_;
    double d_min_;
    double c_max_;
    bool travel_unknown_;

};
} // namespace nav2_hybrid_plabber
#endif //NAV2_HYBRID_PLANNER__HYBRID_PLANNER_HPP_