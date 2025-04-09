#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_msgs/srv/save_map.hpp"
#include "taskwhiz_slam/map_saver.hpp"
#include "nav2_util/lifecycle_node.hpp"

namespace taskwhiz_slam
{
    MapSaver::MapSaver() : nav2_util::LifecycleNode("taskwhiz_map_saver_node")
    {
        this->declare_parameter("map_save_path", "");
    }

    nav2_util::CallbackReturn MapSaver::on_configure(const rclcpp_lifecycle::State &)
    {
        this->get_parameter("map_save_path", map_url_);
        save_map_client_ = this->create_client<nav2_msgs::srv::SaveMap>("/map_saver/save_map");
        save_timer_ = this->create_wall_timer(std::chrono::seconds(30),
                                              std::bind(&MapSaver::save_map, this));
        save_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "MapSaver configured.");
        return nav2_util::CallbackReturn::SUCCESS;
    }
    nav2_util::CallbackReturn MapSaver::on_activate(const rclcpp_lifecycle::State &)
    {
        createBond();
        save_timer_->reset();
        RCLCPP_INFO(this->get_logger(), "MapSaver activated.");
        return nav2_util::CallbackReturn::SUCCESS;
    }
    nav2_util::CallbackReturn MapSaver::on_deactivate(const rclcpp_lifecycle::State &)
    {
        destroyBond();
        save_timer_->cancel();
        RCLCPP_INFO(this->get_logger(), "MapSaver deactivated.");
        return nav2_util::CallbackReturn::SUCCESS;
    }
    nav2_util::CallbackReturn MapSaver::on_cleanup(const rclcpp_lifecycle::State &)
    {
        save_timer_.reset();
        save_map_client_.reset();
        RCLCPP_INFO(this->get_logger(), "MapSaver cleaned up.");
        return nav2_util::CallbackReturn::SUCCESS;
    }
    nav2_util::CallbackReturn MapSaver::on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "MapSaver shutdown.");
        return nav2_util::CallbackReturn::SUCCESS;
    }
    void MapSaver::save_map()
    {
        if (!save_map_client_->wait_for_service(std::chrono::milliseconds(100)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for the service to be available...");
            return;
        }
        auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
        request->map_url = map_url_;
        request->map_topic = "map";
        request->image_format = "pgm";
        request->map_mode = "trinary";
        request->occupied_thresh = 0.65f;
        request->free_thresh = 0.25f;
        auto result_future = save_map_client_->async_send_request(
            request,
            [this, request](const rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedFuture future) {
                if (future.get()->result) {
                    RCLCPP_INFO(this->get_logger(), "Map saved successfully at %s", request->map_url.c_str());
                }
                else {
                    RCLCPP_ERROR(this->get_logger(), "Failed to save map at %s", request->map_url.c_str());
                }
            }
        );
    }
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<taskwhiz_slam::MapSaver>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    rclcpp::shutdown();
    return 0;
}