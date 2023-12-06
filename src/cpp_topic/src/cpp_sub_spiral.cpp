#include <memory>
#include <mutex>
#include "rclcpp/rclcpp.hpp"
#include "onix_backend_messages/msg/system_config.hpp"
#include "onix_backend_messages/msg/location_informations.hpp"
#include "onix_backend_messages/msg/motion_informations.hpp"
#include "onix_backend_messages/msg/status_data.hpp"
#include <vector>

using namespace std::chrono_literals;
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
    float vx;
    float vy;
    float w;
    float steer;
    float spin;
    float r_vx;
    float r_vy;
    float r_w;
    float r_steer;
    float r_spin;
    std::vector<float> steer_angles;
    std::vector<float> r_steer_angles;
    bool is_stop;
    float odo;
    float today_odo;
    int64_t odom_time;
    float total_run_time;
    float odom_controller_temp;
    float odom_controller_voltage;
};

// Initialize the global variable and a mutex to protect it
CombinedData g_combined_data = {};
std::mutex g_data_mutex;
using namespace std::chrono_literals;
class CombinedSubscriber : public rclcpp::Node {
public:
    CombinedSubscriber()
        : Node("combined_subscriber") {
        // Create subscriptions to SystemConfig and LocationInformations messages
        system_config_subscription_ = this->create_subscription<onix_backend_messages::msg::SystemConfig>(
            "system_config", 10, std::bind(&CombinedSubscriber::system_config_callback, this, std::placeholders::_1));

        location_subscription_ = this->create_subscription<onix_backend_messages::msg::LocationInformations>(
            "location", 10, std::bind(&CombinedSubscriber::location_callback, this, std::placeholders::_1));

        motion_subscription_ = this->create_subscription<onix_backend_messages::msg::MotionInformations>(
            "motion", 10, std::bind(&CombinedSubscriber::motion_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<onix_backend_messages::msg::StatusData>("status_data", 10);

        // Create a timer to periodically update the global data
        timer_ = this->create_wall_timer(33ms, std::bind(&CombinedSubscriber::timer_callback, this));
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
        g_combined_data.rssi = msg->rssi;
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

    void motion_callback(const onix_backend_messages::msg::MotionInformations::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(g_data_mutex);

        g_combined_data.vx = msg->vx;
        g_combined_data.vy = msg->vy;
        g_combined_data.w = msg->w;
        g_combined_data.steer = msg->steer;
        g_combined_data.spin = msg->spin;
        g_combined_data.r_vx = msg->r_vx;
        g_combined_data.r_vy = msg->r_vy;
        g_combined_data.r_w = msg->r_w;
        g_combined_data.r_steer = msg->r_steer;
        g_combined_data.r_spin = msg->r_spin;
        g_combined_data.steer_angles = msg->steer_angles;
        g_combined_data.r_steer_angles = msg->r_steer_angles;
        g_combined_data.is_stop = msg->is_stop;
        g_combined_data.odo = msg->odo;
        g_combined_data.today_odo = msg->today_odo;
        g_combined_data.odom_time = msg->time;
        g_combined_data.total_run_time = msg->total_time;
        g_combined_data.odom_controller_temp = msg->odom_controller_temp;
        g_combined_data.odom_controller_voltage = msg->odom_controller_voltage;
    }

    // Timer callback function to periodically update the global data
    void timer_callback()
    {
        std::lock_guard<std::mutex> lock(g_data_mutex);

        // Create a StatusData message and fill it with the combined data
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
        status_data_msg.vx = g_combined_data.vx;
        status_data_msg.vy = g_combined_data.vy;
        status_data_msg.w = g_combined_data.w;
        status_data_msg.steer = g_combined_data.steer;
        status_data_msg.spin = g_combined_data.spin;
        status_data_msg.r_vx = g_combined_data.r_vx;
        status_data_msg.r_vy = g_combined_data.r_vy;
        status_data_msg.r_w = g_combined_data.r_w;
        status_data_msg.r_steer = g_combined_data.r_steer;
        status_data_msg.r_spin = g_combined_data.r_spin;
        status_data_msg.steer_angles = g_combined_data.steer_angles;
        status_data_msg.r_steer_angles = g_combined_data.r_steer_angles;
        status_data_msg.is_stop = g_combined_data.is_stop;
        status_data_msg.odo = g_combined_data.odo;
        status_data_msg.todays_odo = g_combined_data.today_odo;
        status_data_msg.odom_time = g_combined_data.odom_time;
        status_data_msg.total_run_time = g_combined_data.total_run_time;
        status_data_msg.odom_controller_temp = g_combined_data.odom_controller_temp;
        status_data_msg.odom_controller_voltage = g_combined_data.odom_controller_voltage;

        // Publish the combined StatusData message
        publisher_->publish(status_data_msg);
        // Lock the mutex before updating the global variable
        

        // Log the combined data to the console
        RCLCPP_INFO(rclcpp::get_logger("your_node_name"),
                "Updated CombinedData - mac_address: %s, time: %s, controller_voltage: %ld, total_time: %ld, controller_temp: %.2f, ssid: %s, rssi: %ld, x: %f, y: %f, angle: %f, confidence: %f, current_station: %s, last_station: %s, vx: %f, vy: %f, w: %f, steer: %f, spin: %f, r_vx: %f, r_vy: %f, r_w: %f, r_steer: %f, r_spin: %f, is_stop: %s, odo: %f, today_odo: %f, odom_time: %ld, total_run_time: %f",
                g_combined_data.mac_address.c_str(), g_combined_data.time.c_str(),
                g_combined_data.controller_voltage, g_combined_data.total_time,
                g_combined_data.controller_temp, g_combined_data.ssid.c_str(),
                g_combined_data.rssi, g_combined_data.x, g_combined_data.y,
                g_combined_data.angle, g_combined_data.confidence,
                g_combined_data.current_station.c_str(), g_combined_data.last_station.c_str(),
                g_combined_data.vx, g_combined_data.vy, g_combined_data.w,
                g_combined_data.steer, g_combined_data.spin,
                g_combined_data.r_vx, g_combined_data.r_vy, g_combined_data.r_w,
                g_combined_data.r_steer, g_combined_data.r_spin,
                g_combined_data.is_stop ? "true" : "false",
                g_combined_data.odo, g_combined_data.today_odo,
                g_combined_data.odom_time, g_combined_data.total_run_time,
                g_combined_data.odom_controller_temp, g_combined_data.odom_controller_voltage);
}
    rclcpp::Subscription<onix_backend_messages::msg::MotionInformations>::SharedPtr motion_subscription_;
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
