#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/header.hpp>
#include <cmath>

class SimpleOAKDDriver : public rclcpp::Node
{
public:
  SimpleOAKDDriver() : Node("simple_oakd_driver"), frame_count_(0)
  {
    RCLCPP_INFO(this->get_logger(), "Simple OAK-D driver initialized");
    
    // Publishers
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("oak/points", 10);
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("oak/pose", 10);
    
    // Timer to publish test data at 10 Hz
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&SimpleOAKDDriver::publish_test_data, this));
  }

private:
  void publish_test_data()
  {
    try {
      // Generate a simple test point cloud
      std::vector<std::array<float, 3>> points;
      
      // Create a simple 3D grid pattern
      for (int i = -10; i <= 10; ++i) {
        for (int j = -10; j <= 10; ++j) {
          float x = i * 0.2f;
          float y = j * 0.2f;
          float z = 2.0f + 0.5f * std::sin(x + frame_count_ * 0.1f) * std::cos(y + frame_count_ * 0.1f);
          points.push_back({x, y, z});
        }
      }
      
      // Create PointCloud2 message
      auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
      pointcloud_msg->header = create_header();
      pointcloud_msg->height = 1;
      pointcloud_msg->is_dense = false;
      pointcloud_msg->is_bigendian = false;
      
      // Set up fields
      sensor_msgs::PointCloud2Modifier modifier(*pointcloud_msg);
      modifier.setPointCloud2FieldsByString(1, "xyz");
      modifier.resize(points.size());
      
      // Fill point cloud data
      sensor_msgs::PointCloud2Iterator<float> iter_x(*pointcloud_msg, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(*pointcloud_msg, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(*pointcloud_msg, "z");
      
      for (const auto& point : points) {
        *iter_x = point[0];
        *iter_y = point[1];
        *iter_z = point[2];
        ++iter_x;
        ++iter_y;
        ++iter_z;
      }
      
      pointcloud_pub_->publish(*pointcloud_msg);
      
      // Publish pose
      auto pose_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
      pose_msg->header = create_header();
      pose_msg->pose.position.x = 0.0;
      pose_msg->pose.position.y = 0.0;
      pose_msg->pose.position.z = 0.0;
      pose_msg->pose.orientation.w = 1.0;
      pose_pub_->publish(*pose_msg);
      
      frame_count_++;
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error publishing test data: %s", e.what());
    }
  }
  
  std_msgs::msg::Header create_header()
  {
    std_msgs::msg::Header header;
    header.stamp = this->get_clock()->now();
    header.frame_id = "oak_frame";
    return header;
  }
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  int frame_count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<SimpleOAKDDriver>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("simple_oakd_driver"), "Error: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}