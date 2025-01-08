#ifndef NAV2_BASED_PLANNER_H_
#define NAV2_BASED_PLANNER_H_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"


class BasicPlanner : nav2_core::GlobalPlanner {
    public:
        BasicPlanner() = default;
        ~BasicPlanner() = default;
        
        // nav2_core::GlobalPlanner::c
        // plugin configure
        void configure(
            rclcpp_lifecycle::LifecycleNode::SharedPtr parent, 
            std::string name, std::shared_ptr<tf2_ros::Buffer> tf, 
            std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
        ) override;
    
        void cleanup() override;

        void activate() override;

        void deactivate() override;

        // This method creates path for given start and goal pose
        nav_msgs::msg::Path createPlan(
            const geometry_msgs::msg::PoseStamped &start, 
            const geometry_msgs::msg::PoseStamped &goal) override;
            
    private:
    // TF buffer
    std::shared_ptr<tf2_ros::Buffer> tf_;

    // node ptr
    nav2_util::LifecycleNode::SharedPtr node_;

    // Global Costmap
    nav2_costmap_2d::Costmap2D * costmap_;

    // The global frame of the costmap
    std::string global_frame_, name_;

    double interpolation_resolution_;
};

#endif