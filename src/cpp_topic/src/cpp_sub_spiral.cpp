#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "onix_backend_messages/msg/system_config.hpp"
#include "onix_backend_messages/msg/status_data.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
    MinimalSubscriber()
        : Node("minimal_subscriber")
    {
        // Create a subscription to SystemConfig messages
        subscription_ = this->create_subscription<onix_backend_messages::msg::SystemConfig>(
            "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));

        // Initialize the StatusData message
        status_data_msg_ = std::make_shared<onix_backend_messages::msg::StatusData>();

        // Create a timer to periodically update the status data
        timer_ = this->create_wall_timer(std::chrono::seconds(0.33),
                                         std::bind(&MinimalSubscriber::timer_callback, this));
    }

private:
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

        // Log the received values using RCLCPP_INFO
        RCLCPP_INFO(this->get_logger(),
                    "Received StatusData - mac_address: %s, time: %s, controller_voltage: %ld, total_time: %s, controller_temp: %.2f, ssid: %s, rssi: %s",
                    status_data_msg_->mac_address.c_str(), status_data_msg_->time.c_str(),
                    status_data_msg_->controller_voltage, status_data_msg_->total_time,
                    status_data_msg_->controller_temp, status_data_msg_->ssid.c_str(),
                    status_data_msg_->rssi);

        // You can save or process the data as needed
    }

    // Timer callback function to update the status data
    void timer_callback()
    {
        // Update the status data (for example, increment a counter)
        status_data_msg_->total_time += 1;

        // Log the updated status data
        RCLCPP_INFO(this->get_logger(),
                    "Updated StatusData - mac_address: %s, time: %s, controller_voltage: %ld, total_time: %ld, controller_temp: %.2f, ssid: %s, rssi: %ld",
                    status_data_msg_->mac_address.c_str(), status_data_msg_->time.c_str(),
                    status_data_msg_->controller_voltage, status_data_msg_->total_time,
                    status_data_msg_->controller_temp, status_data_msg_->ssid.c_str(),
                    status_data_msg_->rssi);

        // You can publish the updated status data or perform other actions as needed
    }

    rclcpp::Subscription<onix_backend_messages::msg::SystemConfig>::SharedPtr subscription_;
    std::shared_ptr<onix_backend_messages::msg::StatusData> status_data_msg_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
