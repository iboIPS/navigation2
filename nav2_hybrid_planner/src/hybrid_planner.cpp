#include "nav2_hybrid_planner/hybrid_planner.hpp"

#include <cmath>
#include <memory>
#include <string>
#include "nav2_util/node_utils.hpp"
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <limits>

namespace nav2_hybrid_planner
{
/*************************
 *  Helper Struct & utils
 *************************/
struct Cell
{
    int x;
    int y;

    bool operator==(const Cell & other) const
    {
        return x == other.x && y == other.y;
    }
};

struct CellHash
{
    std::size_t operator()(const Cell & c) const
    {
        return std::hash<int>()(c.x << 16 ^ c.y);
    }
};

struct Node
{
    double f;
    int tie;
    Cell cell;
};

struct NodeCompare
{
    bool operator()(const Node & a, const Node & b)
    {
        return a.f > b.f; // min-heap
    }
};

inline double HybridPlanner::heuristic(const Cell & a, const Cell & b)
{
    return std::hypot(a.x - b.x, a.y - b.y);
}

inline bool HybridPlanner::isFree(int x, int y) const
{
    unsigned char c = costmap_->getCost(x, y);
    
    if (travel_unknown_) {
        return c == nav2_costmap_2d::FREE_SPACE || c == nav2_costmap_2d::NO_INFORMATION;
    } else {
        return c == nav2_costmap_2d::FREE_SPACE;
    }
}

/********************
 * Lifecycle methods
 *******************/
void HybridPlanner::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
) {
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ = costmap_ros->getCostmap();
    global_farme_ = costmap_ros->getGlobalFrameID();

    // Parameter initialization
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".weight_length", rclcpp::ParameterValue(1.0)
    );
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".weight_safety", rclcpp::ParameterValue(2.0)
    );
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".length_reference", rclcpp::ParameterValue(1.0)
    );
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".minimum_safety_distance", rclcpp::ParameterValue(1.5)
    );
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".maximum_safety_penalty", rclcpp::ParameterValue(1000.0)
    );
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".travel_unknown", rclcpp::ParameterValue(true)
    );

    node_->get_parameter(name_ + ".weight_length", w_length_);
    node_->get_parameter(name_ + ".weight_safety", w_safety_);
    node_->get_parameter(name_ + ".length_reference", l_ref_);
    node_->get_parameter(name_ + ".minimum_safety_distance", d_min_);
    node_->get_parameter(name_ + ".maximum_safety_penalty", c_max_);
    node_->get_parameter(name_ + ".travel_unknown", travel_unknown_);

    param_cb_handle_ = node_->add_on_set_parameters_callback(
        std::bind(&HybridPlanner::onParamChange, this, std::placeholders::_1)
    );

    RCLCPP_INFO(
        node_->get_logger(), "Configuring plugin %s of type NavfnPlanner",
        name_.c_str()
    );
}

void HybridPlanner::cleanup()
{
    RCLCPP_INFO(
        node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
        name_.c_str()
    );
}

void HybridPlanner::activate()
{
    RCLCPP_INFO(
        node_->get_logger(), "Activate plugin %s of type NavfnPlanner",
        name_.c_str()
    );
}

void HybridPlanner::deactivate()
{
    RCLCPP_INFO(
        node_->get_logger(), "Deactivate plugin %s of type NavfnPlanner",
        name_.c_str()
    );
}

/*********************
 * Distance Transform
 ********************/
std::vector<std::vector<double>> HybridPlanner::computeDistanceTransform()
{
  const int size_x = static_cast<int>(costmap_->getSizeInCellsX());
  const int size_y = static_cast<int>(costmap_->getSizeInCellsY());

  std::vector<std::vector<double>> dist(
    size_y, std::vector<double>(size_x, std::numeric_limits<double>::infinity()));

  std::queue<Cell> q;

  for (int y = 0; y < size_y; ++y) {
    for (int x = 0; x < size_x; ++x) {
      if (!isFree(x, y)) {  // obstacle / inflated obstacle
        dist[y][x] = 0.0;
        q.push({x, y});
      }
    }
  }

  const int dx[4] = {-1, 1, 0, 0};
  const int dy[4] = {0, 0, -1, 1};

  while (!q.empty()) {
    const Cell c = q.front();
    q.pop();

    for (int i = 0; i < 4; ++i) {
      const int nx = c.x + dx[i];
      const int ny = c.y + dy[i];

      if (nx < 0 || ny < 0 || nx >= size_x || ny >= size_y) {
        continue;
      }
      if (!isFree(nx, ny)) {
        continue;
      }

      const double nd = dist[c.y][c.x] + 1.0;
      if (nd < dist[ny][nx]) {
        dist[ny][nx] = nd;
        q.push({nx, ny});
      }
    }
  }

  const double fallback = static_cast<double>(size_x + size_y);
  for (int y = 0; y < size_y; ++y) {
    for (int x = 0; x < size_x; ++x) {
      if (!std::isfinite(dist[y][x])) {
        dist[y][x] = fallback;
      }
    }
  }

  return dist;
}

inline double safetyCost(
    const Cell & c,
    const std::vector<std::vector<double>> & dist_map,
    double d_min,
    double c_max
) 
{
    double d = dist_map[c.y][c.x];
    if (d <= d_min) {
        return c_max;
    }

    return 1.0 / d;
}

/******************
 * CreatePlan (A*)
 *****************/
nav_msgs::msg::Path HybridPlanner::createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancel_checker
)
{
    nav_msgs::msg::Path path;
    path.header.frame_id = global_farme_;
    path.header.stamp = node_->now();
    auto t0 = node_->now();

    unsigned int sx, sy, gx, gy;
    if (!costmap_->worldToMap(start.pose.position.x, start.pose.position.y, sx, sy) || 
        (!costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, gx, gy))) {
            RCLCPP_WARN(node_->get_logger(), "Start or goal out of bounds");
            return path;
        }

    Cell start_c{static_cast<int>(sx), static_cast<int>(sy)};
    Cell goal_c{static_cast<int>(gx), static_cast<int>(gy)};
    
    // =========================
    // Distance Transform timing
    // =========================
    auto t_dt0 = node_->now();
    auto distance_map = computeDistanceTransform();
    auto t_dt1 = node_->now();
    RCLCPP_WARN(node_->get_logger(),
        "DT took %.3f s",
        (t_dt1 - t_dt0).seconds()
    );
    
    std::priority_queue<Node, std::vector<Node>, NodeCompare> open;
    std::unordered_map<Cell, Cell, CellHash> came_from;
    std::unordered_map<Cell, double, CellHash> g_score;
    std::unordered_set<Cell, CellHash> visited;

    int counter = 0;
    g_score[start_c] = 0.0;
    open.push({heuristic(start_c, goal_c), counter++, start_c});

    const int dx[8] = {-1, 1, 0, 0, -1, -1, 1, 1};
    const int dy[8] = {0, 0, -1, 1, -1, 1, -1, 1};

    // =========================
    // A* timing START
    // =========================
    auto t_astar0 = node_->now();

    while (!open.empty())
    {
        if (cancel_checker && cancel_checker()) {
            RCLCPP_WARN(node_->get_logger(), "Planning cancelled");
            return nav_msgs::msg::Path();
        }

        Node current = open.top();
        open.pop();

        Cell c = current.cell;
        if (visited.count(c)) {
            continue;
        }
        visited.insert(c);

        if (c == goal_c) {
            Cell cur = c;
            while (came_from.count(cur)) {
                geometry_msgs::msg::PoseStamped pose;
                double wx, wy;
                costmap_->mapToWorld(cur.x, cur.y, wx, wy);
                pose.pose.position.x = wx;
                pose.pose.position.y = wy;
                pose.pose.orientation.w = 1.0;
                path.poses.push_back(pose);
                cur = came_from[cur];
            }
            std::reverse(path.poses.begin(), path.poses.end());
            path.poses.back().pose.orientation = goal.pose.orientation;
            return path;
        }

        for (int i = 0; i < 8; i++) {
            Cell n{c.x + dx[i], c.y + dy[i]};
            if (n.x < 0 || n.y < 0 ||
                n.x >= static_cast<int>(costmap_->getSizeInCellsX()) ||
                n.y >= static_cast<int>(costmap_->getSizeInCellsY()) ||
                !isFree(n.x, n.y)) {
                continue;
            }

            double step = std::hypot(dx[i], dy[i]);
            double length_cost = step / std::max(l_ref_, 1e-6);
            double safety = safetyCost(n, distance_map, d_min_, c_max_);
            double hybrid_cost = w_length_ * length_cost + w_safety_ * safety;

            double tentative_g = g_score[c] + hybrid_cost;
            if (!g_score.count(n) || tentative_g < g_score[n]) {
                g_score[n] = tentative_g;
                double f = tentative_g + heuristic(n, goal_c);
                open.push({f, counter++, n});
                came_from[n] = c;
            }
        }
    }

    // =========================
    // A* timing END (no path)
    // =========================
    auto t_astar1 = node_->now();
    auto t1 = node_->now();

    RCLCPP_WARN(node_->get_logger(),
        "[HybridPlanner] A* %.3f s | TOTAL %.3f s (NO PATH)",
        (t_astar1 - t_astar0).seconds(),
        (t1 - t0).seconds()
    );

    RCLCPP_WARN(node_->get_logger(), "No path found");
    return path;
}

rcl_interfaces::msg::SetParametersResult
HybridPlanner::onParamChange(const std::vector<rclcpp::Parameter> & parameters)
{
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto & param : parameters)
    {
        const std::string & full = param.get_name();

        // Strip "<plugin>."
        const auto pos = full.find('.');
        if (pos == std::string::npos) {
            continue;
        }

        const std::string key = full.substr(pos + 1);

        if (key == "weight_length") {
            w_length_ = param.as_double();
        }
        else if (key == "weight_safety") {
            w_safety_ = param.as_double();
        }
        else if (key == "length_reference") {
            l_ref_ = std::max(param.as_double(), 1e-6);
        }
        else if (key == "minimum_safety_distance") {
            d_min_ = std::max(param.as_double(), 0.0);
        }
        else if (key == "maximum_safety_penalty") {
            c_max_ = std::max(param.as_double(), 0.0);
        }
        else if (key == "travel_unknown") {
            travel_unknown_ = param.as_bool();
        }
    }

    RCLCPP_WARN(
        node_->get_logger(),
        "[HybridPlanner] Params updated: w_length=%.2f w_safety=%.2f l_ref=%.2f d_min=%.2f c_max=%.2f",
        w_length_, w_safety_, l_ref_, d_min_, c_max_
    );
    RCLCPP_INFO(
        node_->get_logger(),
        "[HybridPlanner] travel_unknown=%s",
        travel_unknown_ ? "true" : "false"
    );

    return result;
}


} // namespace nav2_hybrid_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_hybrid_planner::HybridPlanner, nav2_core::GlobalPlanner)