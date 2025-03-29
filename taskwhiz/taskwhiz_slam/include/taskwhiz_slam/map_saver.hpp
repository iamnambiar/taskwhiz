#ifndef TASKWHIZ_SLAM__MAP_SAVER_HPP
#define TASKWHIZ_SLAM__MAP_SAVER_HPP

#include <string>
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/save_map.hpp"

namespace map_saver
{
    class MapSaver : public rclcpp::Node
    {
    public:
        MapSaver();

    private:
        rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr save_map_client_;
        rclcpp::TimerBase::SharedPtr save_timer_;
        std::string_view map_url_;

        void save_map();
    };
}   // namespace map_saver

#endif  // TASKWHIZ_SLAM__MAP_SAVER_HPP