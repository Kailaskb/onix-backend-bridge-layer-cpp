#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "onix_backend_messages/msg/system_config.hpp"
#include "onix_backend_messages/msg/status_data.hpp"

class StatusDataPublisher : public rclcpp::Node
{
public:
    StatusDataPublisher()
        : Node("status_data_publisher")
    {
        // Create a subscription for SystemConfig messages
        subscription_ = this->create_subscription<onix_backend_messages::msg::SystemConfig>(
            "topic", 10, std::bind(&StatusDataPublisher::topic_callback, this, std::placeholders::_1));

        // Create a publisher for StatusData messages
        publisher_ = this->create_publisher<onix_backend_messages::msg::StatusData>("status_data", 10);

        // Initialize the StatusData message
        status_data_msg_ = std::make_shared<onix_backend_messages::msg::StatusData>();
    }

private:
    // Callback function for the subscription to SystemConfig
    void topic_callback(const onix_backend_messages::msg::SystemConfig::SharedPtr msg)
    {
        // Store received values in the StatusData message
        status_data_msg_->mac_address = msg->mac_address;
        status_data_msg_->time = msg->time;
        status_data_msg_->controller_voltage = msg->controller_voltage;
        status_data_msg_->total_time = std::stoll(msg->total_time);
        status_data_msg_->controller_temp = msg->controller_temp;
        status_data_msg_->ssid = msg->ssid;
        status_data_msg_->rssi = std::stoll(msg->rssi);

        // Add other fields as needed

        // Log the received values (optional)
        RCLCPP_INFO(this->get_logger(),
                    "Received - mac_address: %s, time: %s, controller_voltage: %ld, total_time: %ld, controller_temp: %.2f, ssid: %s, rssi: %ld",
                    msg->mac_address.c_str(), msg->time.c_str(), status_data_msg_->controller_voltage,
                    status_data_msg_->total_time, status_data_msg_->controller_temp, msg->ssid.c_str(),
                    status_data_msg_->rssi);

        // Publish the received StatusData message
        publisher_->publish(*status_data_msg_);
    }

    rclcpp::Subscription<onix_backend_messages::msg::SystemConfig>::SharedPtr subscription_;
    rclcpp::Publisher<onix_backend_messages::msg::StatusData>::SharedPtr publisher_;
    std::shared_ptr<onix_backend_messages::msg::StatusData> status_data_msg_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StatusDataPublisher>());
    rclcpp::shutdown();
    return 0;
}
