#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include <algorithm>

class DistancePublisher : public rclcpp::Node {
public:
    DistancePublisher() : Node("distance_publisher") {
        // Create subscriber for LiDAR data
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&DistancePublisher::scan_callback, this, std::placeholders::_1));
        
        // Create publisher for distance data
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/distance", 10);
        
        RCLCPP_INFO(this->get_logger(), "Distance publisher ready!");
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Process scan ranges
        std::vector<float> valid_ranges;
        for (float range : msg->ranges) {
            if (std::isfinite(range) && range > msg->range_min) {
                valid_ranges.push_back(range);
            }
        }

        if (!valid_ranges.empty()) {
            // Find and publish minimum distance
            float min_distance = *std::min_element(valid_ranges.begin(), valid_ranges.end());
            auto distance_msg = std_msgs::msg::Float32();
            distance_msg.data = min_distance;
            publisher_->publish(distance_msg);
            
            // Print exactly as requested
            RCLCPP_INFO(this->get_logger(), "Published Distance: %.2f meters", min_distance);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DistancePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
