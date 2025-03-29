#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float32.hpp"
#include <algorithm>

class DistancePublisher : public rclcpp::Node {
public:
    DistancePublisher() : Node("distance_publisher") {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&DistancePublisher::scan_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/distance", 10);
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float min_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        auto distance_msg = std_msgs::msg::Float32();
        distance_msg.data = min_distance;
        publisher_->publish(distance_msg);
        RCLCPP_INFO(this->get_logger(), "Published Distance: %.2f meters", min_distance);
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
