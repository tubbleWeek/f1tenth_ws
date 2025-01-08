#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "f1tenth_planners/f1tenth_planner.h"


    
void BasicPlanner::configure(
    rclcpp_lifecycle::LifecycleNode::SharedPtr parent, 
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf, 
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) 
{
    return;


}

void BasicPlanner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}


void BasicPlanner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void BasicPlanner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path BasicPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{

}