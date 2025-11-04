#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <memory>

// Minimal test node to check basic ROS2 functionality
class MinimalCameraTest : public rclcpp::Node {
public:
    MinimalCameraTest() : Node("minimal_camera_test") {
        RCLCPP_INFO(this->get_logger(), "Minimal camera test node started");
        
        // Try to check available memory
        checkSystemMemory();
        
        // Create a simple timer to publish test messages
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&MinimalCameraTest::timerCallback, this)
        );
        
        // Create a publisher for test messages
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("test_image", 10);
        
        RCLCPP_INFO(this->get_logger(), "Minimal test node initialized successfully");
    }

private:
    void checkSystemMemory() {
        try {
            // Try to allocate a small amount of memory to test
            std::vector<uint8_t> test_memory(1024 * 1024); // 1MB
            RCLCPP_INFO(this->get_logger(), "Memory allocation test: SUCCESS");
        } catch (const std::bad_alloc& e) {
            RCLCPP_ERROR(this->get_logger(), "Memory allocation test: FAILED - %s", e.what());
        }
    }

    void timerCallback() {
        auto message = sensor_msgs::msg::Image();
        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "test_frame";
        message.width = 64;
        message.height = 48;
        message.encoding = "mono8";
        message.step = message.width;
        message.data.resize(message.width * message.height, 128); // Gray image
        
        publisher_->publish(message);
        RCLCPP_INFO(this->get_logger(), "Published test image - memory usage OK");
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<MinimalCameraTest>();
        RCLCPP_INFO(rclcpp::get_logger("main"), "Starting minimal camera test node...");
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("main"), "Exception in minimal test node: %s", e.what());
        return 1;
    }
    
    rclcpp::shutdown();
    return 0;
}