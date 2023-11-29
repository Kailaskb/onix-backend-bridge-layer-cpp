#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "onix_backend_messages/msg/system_config.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<onix_backend_messages::msg::SystemConfig>(
            "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const onix_backend_messages::msg::SystemConfig::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(),
                    "Received - mac_address: %s, time: %s, controller_voltage: %s, total_time: %s, controller_temp: %s, ssid: %s, rssi: %s",
                    msg->mac_address.c_str(), msg->time.c_str(), msg->controller_voltage.c_str(),
                    msg->total_time.c_str(), msg->controller_temp.c_str(), msg->ssid.c_str(),
                    msg->rssi.c_str());
    }

    rclcpp::Subscription<onix_backend_messages::msg::SystemConfig>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
