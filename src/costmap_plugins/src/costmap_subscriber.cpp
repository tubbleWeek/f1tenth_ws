#include "costmap_subscriber.h"

CostmapSubscriber::CostmapSubscriber() : Node("costmap_subscriber") {
    std::string costmap_topic;
    this->get_parameter("costmap_topic", costmap_topic);
    subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        costmap_topic, 10, 
        std::bind(&CostmapSubscriber::costmap_callback, this, std::placeholders::_1)
    );
}

