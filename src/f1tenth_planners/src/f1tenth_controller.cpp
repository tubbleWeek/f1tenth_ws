#include <algorithm>

#include "nav2_core/exceptions.hpp"
#include "nav2_util/node_utils.hpp"
#include "f1tenth_planners/f1tenth_controller.hpp"
#include "pluginlib/class_list_macros.hpp"

using std::hypot;
using std::min;
using std::max;
using std::abs;
using nav2_util::declare_parameter_if_not_declared;

namespace nav2_controller {
    void PurePursuitController::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> & tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> & costmap_ros)
    {
        node_ = parent;
        auto node = node_.lock();
        costmap_ros_ = costmap_ros;
        tf_ = tf;
        plugin_name_ = name;
        logger_ = node->get_logger();
        clock_ = node->get_clock();
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".desired_linear_vel", rclcpp::ParameterValue(
            0.2));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".lookahead_dist",
            rclcpp::ParameterValue(0.4));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".max_angular_vel", rclcpp::ParameterValue(
            1.0));
        declare_parameter_if_not_declared(
            node, plugin_name_ + ".transform_tolerance", rclcpp::ParameterValue(
            0.1));
        node->get_parameter(plugin_name_ + ".desired_linear_vel", desired_linear_vel_);
        node->get_parameter(plugin_name_ + ".lookahead_dist", lookahead_dist_);
        node->get_parameter(plugin_name_ + ".max_angular_vel", max_angular_vel_);
        double transform_tolerance;
        node->get_parameter(plugin_name_ + ".transform_tolerance", transform_tolerance);
        transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
    }

      void PurePursuitController::cleanup()
        {
            RCLCPP_INFO(
            logger_, "CleaningUp plugin %s of type PurePursuit",
            name_.c_str());
        }


  void PurePursuitController::activate()
  {
    RCLCPP_INFO(
      logger_, "Activating plugin %s of type PurePursuit",
      name_.c_str());
  }

    void PurePursuitController::deactivate()
    {
        RCLCPP_INFO(
        logger_, "Deactivating plugin %s of type PurePursuit",
        name_.c_str());
    }

}