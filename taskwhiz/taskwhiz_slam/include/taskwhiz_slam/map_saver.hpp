#ifndef TASKWHIZ_SLAM__MAP_SAVER_HPP
#define TASKWHIZ_SLAM__MAP_SAVER_HPP

#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace taskwhiz_slam
{
    class MapSaver : public nav2_util::LifecycleNode
    {
    public:
        MapSaver();

        // Lifecycle transitions
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state);
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state);

    private:
        void save_map();

        std::string map_url_;
        rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr save_map_client_;
        rclcpp::TimerBase::SharedPtr save_timer_;

    };
}   // namespace taskwhiz_slam

#endif  // TASKWHIZ_SLAM__MAP_SAVER_HPP