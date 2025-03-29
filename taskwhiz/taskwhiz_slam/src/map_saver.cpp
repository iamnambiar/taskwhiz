#include "taskwhiz_slam/map_saver.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/save_map.hpp"

namespace map_saver
{
    MapSaver::MapSaver() : rclcpp::Node("taskwhiz_map_saver_node")
    {
        this->declare_parameter("map_save_path", "");
        map_url_ = this->get_parameter("map_save_path").as_string();
        save_map_client_ = this->create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");
        save_timer_ = this->create_wall_timer(std::chrono::seconds(30),
                                              std::bind(&MapSaver::save_map, this));
    }

    void MapSaver::save_map()
    {
        auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
        request->map_url = map_url_;

        while (!save_map_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the service to be available...");
        }
        auto result = save_map_client_->async_send_request(request);
        if (result.get()->result)
        {
            RCLCPP_INFO(this->get_logger(), "Map saved successfully at: %s", request->map_url.c_str());
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to save the map.");
        }
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<map_saver::MapSaver>());
    rclcpp::shutdown();
    return 0;
}