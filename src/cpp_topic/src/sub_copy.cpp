#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "onix_backend_messages/msg/system_config.hpp"
#include "onix_backend_messages/msg/location_informations.hpp"
#include "onix_backend_messages/msg/status_data.hpp"
// Define a global variable to store the received data
struct CombinedData {
    std::string mac_address;
    std::string time;
    int64_t controller_voltage;
    int64_t total_time;
    float controller_temp;
    std::string ssid;
    int64_t rssi;
    double x;
    double y;
    double angle;
    float confidence;
    std::string current_station;
    std::string last_station;
};

// Initialize the global variable and a mutex to protect it
CombinedData g_combined_data = {};
std::mutex g_data_mutex;

class CombinedSubscriber : public rclcpp::Node {
public:
    CombinedSubscriber()
        : Node("minimal_publisher") {
        // Create subscriptions to SystemConfig and LocationInformations messages
        system_config_subscription_ = this->create_subscription<onix_backend_messages::msg::SystemConfig>(
            "topic", 10, std::bind(&CombinedSubscriber::system_config_callback, this, std::placeholders::_1));

        location_subscription_ = this->create_subscription<onix_backend_messages::msg::LocationInformations>(
            "motion", 10, std::bind(&CombinedSubscriber::location_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<onix_backend_messages::msg::StatusData>("status_data", 10);
        // Create a timer to periodically update the global data
        timer_ = this->create_wall_timer(std::chrono::duration<double>(0.33), std::bind(&CombinedSubscriber::timer_callback, this));
    }

private:
    void system_config_callback(const onix_backend_messages::msg::SystemConfig::SharedPtr msg)
    {
        // Lock the mutex before updating the global variable
        std::lock_guard<std::mutex> lock(g_data_mutex);

        // Store received SystemConfig values in the global variable
        g_combined_data.mac_address = msg->mac_address;
        g_combined_data.time = msg->time;
        g_combined_data.controller_voltage = msg->controller_voltage;
        g_combined_data.total_time = std::stoll(msg->total_time);
        g_combined_data.controller_temp = msg->controller_temp;
        g_combined_data.ssid = msg->ssid;
        g_combined_data.rssi = std::stoll(msg->rssi);
    }

    void location_callback(const onix_backend_messages::msg::LocationInformations::SharedPtr msg)
    {
        // Lock the mutex before updating the global variable
        std::lock_guard<std::mutex> lock(g_data_mutex);

        // Store received LocationInformations values in the global variable
        g_combined_data.x = msg->x;
        g_combined_data.y = msg->y;
        g_combined_data.angle = msg->angle;
        g_combined_data.confidence = msg->confidence;
        g_combined_data.current_station = msg->current_station;
        g_combined_data.last_station = msg->last_station;
    }

    // Timer callback function to periodically update the global data
    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(g_data_mutex);
        
        onix_backend_messages::msg::StatusData status_data_msg;
        status_data_msg.mac_address = g_combined_data.mac_address;
        status_data_msg.time = g_combined_data.time;
        status_data_msg.controller_voltage = g_combined_data.controller_voltage;
        status_data_msg.total_time = g_combined_data.total_time;
        status_data_msg.controller_temp = g_combined_data.controller_temp;
        status_data_msg.ssid = g_combined_data.ssid;
        status_data_msg.rssi = g_combined_data.rssi;
        status_data_msg.x = g_combined_data.x;
        status_data_msg.y = g_combined_data.y;
        status_data_msg.angle = g_combined_data.angle;
        status_data_msg.confidence = g_combined_data.confidence;
        status_data_msg.current_station = g_combined_data.current_station;
        status_data_msg.last_station = g_combined_data.last_station;

        // Add other fields as needed

        
        // Publish the combined StatusData message
        publisher_->publish(status_data_msg);
        // Lock the mutex before updating the global variable
        

        // Log the combined data to the console
        RCLCPP_INFO(get_logger(),
                    "Updated CombinedData - mac_address: %s, time: %s, controller_voltage: %ld, total_time: %ld, controller_temp: %.2f, ssid: %s, rssi: %ld, x: %f, y: %f, angle: %f, confidence: %f, current_station: %s, last_station: %s",
                    g_combined_data.mac_address.c_str(), g_combined_data.time.c_str(),
                    g_combined_data.controller_voltage, g_combined_data.total_time,
                    g_combined_data.controller_temp, g_combined_data.ssid.c_str(),
                    g_combined_data.rssi, g_combined_data.x, g_combined_data.y,
                    g_combined_data.angle, g_combined_data.confidence,
                    g_combined_data.current_station.c_str(), g_combined_data.last_station.c_str());
    }

    rclcpp::Subscription<onix_backend_messages::msg::SystemConfig>::SharedPtr system_config_subscription_;
    rclcpp::Subscription<onix_backend_messages::msg::LocationInformations>::SharedPtr location_subscription_;
    rclcpp::Publisher<onix_backend_messages::msg::StatusData>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // Create an instance of the CombinedSubscriber
    auto combined_subscriber = std::make_shared<CombinedSubscriber>();

    // Spin the node
    rclcpp::spin(combined_subscriber);

    rclcpp::shutdown();
    return 0;
}
