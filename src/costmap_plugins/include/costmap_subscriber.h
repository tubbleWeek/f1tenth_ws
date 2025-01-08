#ifndef COSTMAP_H_
#define COSTMAP_H_

#include <string>
#include <vector>
#include <memory>

#include "nav2_costmap_2d/costmap_2d_ros.hpp"

class CostmapSubscriber : public rclcpp::Node {
    public:
        CostmapSubscriber() : Node("CostmapSubscriber") {

        }

    private:
        void costmap_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
};

#endif