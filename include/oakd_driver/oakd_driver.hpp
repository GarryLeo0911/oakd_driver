#ifndef OAKD_DRIVER__OAKD_DRIVER_HPP_
#define OAKD_DRIVER__OAKD_DRIVER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <thread>
#include <atomic>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logger.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "camera_info_manager/camera_info_manager.hpp"

#include <opencv2/opencv.hpp>
#include <depthai/depthai.hpp>

namespace oakd_driver
{

class OAKDDriver : public rclcpp::Node
{
public:
  explicit OAKDDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~OAKDDriver();

private:
  // Initialization methods
  void declare_parameters();
  void get_parameters();
  void setup_publishers();
  void setup_services();
  void setup_diagnostics();
  void check_dependencies();

  // Device connection methods
  void start_camera();
  void stop_camera();
  void detect_and_connect_device();
  void connect_device(const dai::DeviceInfo& device_info, dai::UsbSpeed usb_speed, const std::string& description);
  
  // Pipeline setup methods
  void setup_pipeline();
  void create_camera_nodes();
  void create_stereo_node();
  void setup_outputs();
  dai::ColorCameraProperties::SensorResolution get_rgb_resolution();
  dai::MonoCameraProperties::SensorResolution get_mono_resolution();
  
  // Device configuration
  void start_device();
  void setup_queues();
  void configure_ir();
  void load_external_calibration();
  
  // Frame processing
  void process_frames();
  void publish_rgb_image(std::shared_ptr<dai::ImgFrame> frame);
  void publish_depth_image(std::shared_ptr<dai::ImgFrame> frame);
  void publish_pointcloud(std::shared_ptr<dai::ImgFrame> frame);
  void publish_rgb_camera_info();
  void publish_depth_camera_info();
  void publish_transform();
  void publish_diagnostics();
  
  // Utility methods
  std_msgs::msg::Header create_header(const std::string& frame_id);
  void save_pipeline();
  void save_calibration();
  
  // Service callbacks
  void start_camera_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                        std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void stop_camera_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                       std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void save_pipeline_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                         std::shared_ptr<std_srvs::srv::Trigger::Response> response);
  void save_calibration_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                            std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  // State management
  std::atomic<bool> camera_running_{false};
  std::mutex startup_lock_;
  
  // DepthAI objects
  std::shared_ptr<dai::Device> device_;
  std::shared_ptr<dai::Pipeline> pipeline_;
  std::map<std::string, std::shared_ptr<dai::DataOutputQueue>> queues_;
  dai::CalibrationHandler calibration_data_;
  std::map<std::string, std::string> device_info_;
  
  // Pipeline nodes
  std::shared_ptr<dai::node::ColorCamera> cam_rgb_;
  std::shared_ptr<dai::node::MonoCamera> mono_left_;
  std::shared_ptr<dai::node::MonoCamera> mono_right_;
  std::shared_ptr<dai::node::StereoDepth> stereo_;
  std::shared_ptr<dai::node::XLinkOut> rgb_out_;
  std::shared_ptr<dai::node::XLinkOut> depth_out_;
  
  // Parameters
  // Device connection parameters
  std::string mx_id_;
  std::string ip_;
  std::string usb_port_id_;
  std::string usb_speed_str_;
  
  // Camera parameters
  int fps_;
  std::string rgb_resolution_;
  std::string depth_resolution_;
  
  // Stereo depth parameters
  int confidence_threshold_;
  bool lr_check_;
  bool subpixel_;
  bool extended_disparity_;
  std::string depth_preset_mode_;
  
  // Frame parameters
  std::string camera_name_;
  std::string camera_model_;
  std::string base_frame_;
  std::string parent_frame_;
  bool publish_tf_;
  
  // IR parameters
  bool enable_ir_;
  int laser_dot_brightness_;
  int floodlight_brightness_;
  
  // Pipeline parameters
  std::string pipeline_type_;
  bool pipeline_dump_;
  bool calibration_dump_;
  std::string external_calibration_path_;
  
  // Other parameters
  bool restart_on_error_;
  
  // ROS Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_info_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  
  // ROS Services
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_pipeline_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr save_calib_srv_;
  
  // TF Broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  // Timers
  rclcpp::TimerBase::SharedPtr processing_timer_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;
  rclcpp::TimerBase::SharedPtr delayed_start_timer_;
  
  // Frame count for test data
  int frame_count_;
};

}  // namespace oakd_driver

#endif  // OAKD_DRIVER__OAKD_DRIVER_HPP_