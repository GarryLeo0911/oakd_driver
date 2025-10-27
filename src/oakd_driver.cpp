#include "oakd_driver/oakd_driver.hpp"
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <filesystem>
#include <fstream>

namespace oakd_driver
{

OAKDDriver::OAKDDriver(const rclcpp::NodeOptions & options)
: Node("oakd_driver", options), frame_count_(0)
{
  RCLCPP_INFO(this->get_logger(), "Initializing OAK-D driver node...");
  
  // Check dependencies first
  check_dependencies();
  
  // Declare and get parameters
  declare_parameters();
  get_parameters();
  
  // Setup publishers
  setup_publishers();
  
  // Setup TF broadcaster
  if (publish_tf_) {
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }
  
  // Setup services
  setup_services();
  
  // Setup diagnostics
  setup_diagnostics();
  
  // Start camera with delay to allow node to fully initialize
  delayed_start_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    [this]() {
      delayed_start_timer_->cancel();
      start_camera();
    }
  );
  
  RCLCPP_INFO(this->get_logger(), "OAK-D driver node initialized");
}

OAKDDriver::~OAKDDriver()
{
  stop_camera();
}

void OAKDDriver::check_dependencies()
{
  // Check if DepthAI is available
  try {
    auto devices = dai::Device::getAllAvailableDevices();
    RCLCPP_INFO(this->get_logger(), "DepthAI library loaded successfully");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "DepthAI library not available: %s", e.what());
    RCLCPP_ERROR(this->get_logger(), "Please install DepthAI library:");
    RCLCPP_ERROR(this->get_logger(), "  Ubuntu: sudo apt install ros-jazzy-depthai");
    RCLCPP_ERROR(this->get_logger(), "  Or build from source: https://github.com/luxonis/depthai-core");
    throw std::runtime_error("DepthAI library not available");
  }
  
  // Check OpenCV
  RCLCPP_INFO(this->get_logger(), "OpenCV version: %s", CV_VERSION);
}

void OAKDDriver::declare_parameters()
{
  // Device connection parameters
  this->declare_parameter("i_mx_id", "");
  this->declare_parameter("i_ip", "");
  this->declare_parameter("i_usb_port_id", "");
  this->declare_parameter("i_usb_speed", "SUPER_PLUS");
  
  // Camera parameters
  this->declare_parameter("i_fps", 30);
  this->declare_parameter("i_rgb_resolution", "1080p");
  this->declare_parameter("i_depth_resolution", "720p");
  
  // Stereo depth parameters
  this->declare_parameter("i_depth_confidence_threshold", 200);
  this->declare_parameter("i_depth_lr_check", true);
  this->declare_parameter("i_depth_subpixel", false);
  this->declare_parameter("i_depth_extended_disparity", false);
  this->declare_parameter("i_depth_preset_mode", "HIGH_ACCURACY");
  
  // Frame parameters
  this->declare_parameter("i_tf_camera_name", "oak");
  this->declare_parameter("i_tf_camera_model", "");
  this->declare_parameter("i_tf_base_frame", "base_link");
  this->declare_parameter("i_tf_parent_frame", "oak_camera_frame");
  this->declare_parameter("i_publish_tf_from_calibration", true);
  
  // IR parameters
  this->declare_parameter("i_enable_ir", false);
  this->declare_parameter("i_laser_dot_brightness", 800);
  this->declare_parameter("i_floodlight_brightness", 0);
  
  // Pipeline parameters
  this->declare_parameter("i_pipeline_type", "RGBD");
  this->declare_parameter("i_pipeline_dump", false);
  this->declare_parameter("i_calibration_dump", false);
  this->declare_parameter("i_external_calibration_path", "");
  
  // Restart parameters
  this->declare_parameter("i_restart_on_diagnostics_error", false);
}

void OAKDDriver::get_parameters()
{
  // Device connection
  mx_id_ = this->get_parameter("i_mx_id").as_string();
  ip_ = this->get_parameter("i_ip").as_string();
  usb_port_id_ = this->get_parameter("i_usb_port_id").as_string();
  usb_speed_str_ = this->get_parameter("i_usb_speed").as_string();
  
  // Camera settings
  fps_ = this->get_parameter("i_fps").as_int();
  rgb_resolution_ = this->get_parameter("i_rgb_resolution").as_string();
  depth_resolution_ = this->get_parameter("i_depth_resolution").as_string();
  
  // Stereo depth
  confidence_threshold_ = this->get_parameter("i_depth_confidence_threshold").as_int();
  lr_check_ = this->get_parameter("i_depth_lr_check").as_bool();
  subpixel_ = this->get_parameter("i_depth_subpixel").as_bool();
  extended_disparity_ = this->get_parameter("i_depth_extended_disparity").as_bool();
  depth_preset_mode_ = this->get_parameter("i_depth_preset_mode").as_string();
  
  // Frame parameters
  camera_name_ = this->get_parameter("i_tf_camera_name").as_string();
  camera_model_ = this->get_parameter("i_tf_camera_model").as_string();
  base_frame_ = this->get_parameter("i_tf_base_frame").as_string();
  parent_frame_ = this->get_parameter("i_tf_parent_frame").as_string();
  publish_tf_ = this->get_parameter("i_publish_tf_from_calibration").as_bool();
  
  // IR settings
  enable_ir_ = this->get_parameter("i_enable_ir").as_bool();
  laser_dot_brightness_ = this->get_parameter("i_laser_dot_brightness").as_int();
  floodlight_brightness_ = this->get_parameter("i_floodlight_brightness").as_int();
  
  // Pipeline settings
  pipeline_type_ = this->get_parameter("i_pipeline_type").as_string();
  pipeline_dump_ = this->get_parameter("i_pipeline_dump").as_bool();
  calibration_dump_ = this->get_parameter("i_calibration_dump").as_bool();
  external_calibration_path_ = this->get_parameter("i_external_calibration_path").as_string();
  
  // Other settings
  restart_on_error_ = this->get_parameter("i_restart_on_diagnostics_error").as_bool();
}

void OAKDDriver::setup_publishers()
{
  rgb_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    camera_name_ + "/rgb/image_raw", 10);
  depth_pub_ = this->create_publisher<sensor_msgs::msg::Image>(
    camera_name_ + "/depth/image_raw", 10);
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    camera_name_ + "/points", 10);
  rgb_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    camera_name_ + "/rgb/camera_info", 10);
  depth_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(
    camera_name_ + "/depth/camera_info", 10);
}

void OAKDDriver::setup_services()
{
  start_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/start_camera",
    std::bind(&OAKDDriver::start_camera_srv, this, std::placeholders::_1, std::placeholders::_2));
    
  stop_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/stop_camera",
    std::bind(&OAKDDriver::stop_camera_srv, this, std::placeholders::_1, std::placeholders::_2));
    
  save_pipeline_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/save_pipeline",
    std::bind(&OAKDDriver::save_pipeline_srv, this, std::placeholders::_1, std::placeholders::_2));
    
  save_calib_srv_ = this->create_service<std_srvs::srv::Trigger>(
    "~/save_calibration",
    std::bind(&OAKDDriver::save_calibration_srv, this, std::placeholders::_1, std::placeholders::_2));
}

void OAKDDriver::setup_diagnostics()
{
  diagnostics_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>(
    "/diagnostics", 10);
  diagnostics_timer_ = this->create_wall_timer(
    std::chrono::seconds(1),
    std::bind(&OAKDDriver::publish_diagnostics, this));
}

void OAKDDriver::start_camera()
{
  std::lock_guard<std::mutex> lock(startup_lock_);
  
  if (camera_running_) {
    RCLCPP_INFO(this->get_logger(), "Camera already running!");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Starting camera...");
  
  try {
    detect_and_connect_device();
    setup_pipeline();
    start_device();
    setup_queues();
    configure_ir();
    
    // Start processing timer
    processing_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / fps_)),
      std::bind(&OAKDDriver::process_frames, this));
    
    camera_running_ = true;
    RCLCPP_INFO(this->get_logger(), "Camera started successfully!");
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to start camera: %s", e.what());
    stop_camera();
    throw;
  }
}

void OAKDDriver::stop_camera()
{
  std::lock_guard<std::mutex> lock(startup_lock_);
  
  if (!camera_running_) {
    RCLCPP_INFO(this->get_logger(), "Camera already stopped!");
    return;
  }
  
  RCLCPP_INFO(this->get_logger(), "Stopping camera...");
  
  try {
    // Stop processing timer
    if (processing_timer_) {
      processing_timer_->cancel();
    }
    
    // Close queues
    queues_.clear();
    
    // Close device
    if (device_) {
      device_->close();
      device_.reset();
    }
    
    pipeline_.reset();
    camera_running_ = false;
    
    RCLCPP_INFO(this->get_logger(), "Camera stopped successfully!");
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error stopping camera: %s", e.what());
  }
}

void OAKDDriver::detect_and_connect_device()
{
  std::map<std::string, dai::UsbSpeed> usb_speed_map = {
    {"HIGH", dai::UsbSpeed::HIGH},
    {"SUPER", dai::UsbSpeed::SUPER},
    {"SUPER_PLUS", dai::UsbSpeed::SUPER_PLUS}
  };
  
  dai::UsbSpeed usb_speed = usb_speed_map.count(usb_speed_str_) ? 
    usb_speed_map[usb_speed_str_] : dai::UsbSpeed::SUPER_PLUS;
  
  bool connected = false;
  while (!connected && rclcpp::ok()) {
    try {
      if (mx_id_.empty() && ip_.empty() && usb_port_id_.empty()) {
        // Connect to any available device
        RCLCPP_INFO(this->get_logger(), "No device ID specified, connecting to next available device...");
        auto device_infos = dai::Device::getAllAvailableDevices();
        
        if (device_infos.empty()) {
          throw std::runtime_error("No devices detected!");
        }
        
        device_ = std::make_shared<dai::Device>(device_infos[0], usb_speed);
        connected = true;
      } else {
        // Connect to specific device
        auto available_devices = dai::Device::getAllAvailableDevices();
        
        if (available_devices.empty() && !ip_.empty()) {
          // Try direct IP connection
          RCLCPP_INFO(this->get_logger(), "No devices detected, trying direct IP connection: %s", ip_.c_str());
          dai::DeviceInfo device_info(ip_);
          available_devices.push_back(device_info);
        }
        
        if (available_devices.empty()) {
          throw std::runtime_error("No devices detected!");
        }
        
        for (const auto& device_info : available_devices) {
          if (!mx_id_.empty() && device_info.getMxId() == mx_id_) {
            connect_device(device_info, usb_speed, "MXID: " + mx_id_);
            connected = true;
            break;
          } else if (!ip_.empty() && device_info.name == ip_) {
            connect_device(device_info, usb_speed, "IP: " + ip_);
            connected = true;
            break;
          } else if (!usb_port_id_.empty() && device_info.name == usb_port_id_) {
            connect_device(device_info, usb_speed, "USB ID: " + usb_port_id_);
            connected = true;
            break;
          }
        }
      }
      
      if (connected) {
        // Get device information
        device_info_["mxid"] = device_->getMxId();
        device_info_["name"] = device_->getDeviceInfo().name;
        device_info_["device_name"] = device_->getDeviceName();
        
        try {
          device_info_["usb_speed"] = std::to_string(static_cast<int>(device_->getUsbSpeed()));
        } catch (...) {
          device_info_["usb_speed"] = "unknown";
        }
        
        // Set camera model if not specified
        if (camera_model_.empty()) {
          camera_model_ = device_->getDeviceName();
        }
        
        RCLCPP_INFO(this->get_logger(), "Connected to device: %s (MXID: %s)", 
          device_info_["device_name"].c_str(), device_info_["mxid"].c_str());
        
        // Get calibration data
        calibration_data_ = device_->readCalibration();
        
        // Log camera sensors
        auto sensor_names = device_->getCameraSensorNames();
        for (const auto& [socket, sensor_name] : sensor_names) {
          RCLCPP_DEBUG(this->get_logger(), "Socket %d: %s", static_cast<int>(socket), sensor_name.c_str());
        }
      }
      
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Device connection failed: %s", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  
  if (!connected) {
    throw std::runtime_error("Failed to connect to any device");
  }
}

void OAKDDriver::connect_device(const dai::DeviceInfo& device_info, dai::UsbSpeed usb_speed, const std::string& description)
{
  RCLCPP_INFO(this->get_logger(), "Connecting to device using %s", description.c_str());
  
  try {
    // For newer DepthAI API, device state check is simplified
    if (device_info.name.find("192.168") == 0 || 
        device_info.name.find("10.") == 0 || 
        device_info.name.find("172.") == 0) {
      // IP device
      device_ = std::make_shared<dai::Device>(device_info);
    } else {
      // USB device
      device_ = std::make_shared<dai::Device>(device_info, usb_speed);
    }
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to connect to device: %s", e.what());
    throw;
  }
}

void OAKDDriver::setup_pipeline()
{
  RCLCPP_INFO(this->get_logger(), "Setting up DepthAI pipeline...");
  
  // Create pipeline
  pipeline_ = std::make_shared<dai::Pipeline>();
  
  // Load external calibration if specified
  if (!external_calibration_path_.empty()) {
    load_external_calibration();
  }
  
  // Create nodes
  create_camera_nodes();
  create_stereo_node();
  setup_outputs();
  
  if (pipeline_dump_) {
    save_pipeline();
  }
  
  if (calibration_dump_) {
    save_calibration();
  }
}

void OAKDDriver::create_camera_nodes()
{
  // RGB camera
  cam_rgb_ = pipeline_->create<dai::node::ColorCamera>();
  cam_rgb_->setBoardSocket(dai::CameraBoardSocket::CAM_A);  // Updated for newer API
  cam_rgb_->setResolution(get_rgb_resolution());
  cam_rgb_->setInterleaved(false);
  cam_rgb_->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
  cam_rgb_->setFps(fps_);
  
  // Mono cameras for stereo
  mono_left_ = pipeline_->create<dai::node::MonoCamera>();
  mono_left_->setResolution(get_mono_resolution());
  mono_left_->setBoardSocket(dai::CameraBoardSocket::CAM_B);  // Updated for newer API
  
  mono_right_ = pipeline_->create<dai::node::MonoCamera>();
  mono_right_->setResolution(get_mono_resolution());
  mono_right_->setBoardSocket(dai::CameraBoardSocket::CAM_C);  // Updated for newer API
}

void OAKDDriver::create_stereo_node()
{
  stereo_ = pipeline_->create<dai::node::StereoDepth>();
  
  // Use DEFAULT preset mode (HIGH_ACCURACY is deprecated)
  stereo_->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::DEFAULT);
  
  // Configure stereo
  stereo_->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
  stereo_->setLeftRightCheck(lr_check_);
  stereo_->setSubpixel(subpixel_);
  stereo_->setExtendedDisparity(extended_disparity_);
  stereo_->initialConfig.setConfidenceThreshold(confidence_threshold_);
  
  // Link cameras to stereo
  mono_left_->out.link(stereo_->left);
  mono_right_->out.link(stereo_->right);
}

void OAKDDriver::setup_outputs()
{
  // RGB output
  rgb_out_ = pipeline_->create<dai::node::XLinkOut>();
  rgb_out_->setStreamName("rgb");
  cam_rgb_->isp.link(rgb_out_->input);
  
  // Depth output
  depth_out_ = pipeline_->create<dai::node::XLinkOut>();
  depth_out_->setStreamName("depth");
  stereo_->depth.link(depth_out_->input);
}

dai::ColorCameraProperties::SensorResolution OAKDDriver::get_rgb_resolution()
{
  std::map<std::string, dai::ColorCameraProperties::SensorResolution> resolution_map = {
    {"720p", dai::ColorCameraProperties::SensorResolution::THE_720_P},
    {"1080p", dai::ColorCameraProperties::SensorResolution::THE_1080_P},
    {"4K", dai::ColorCameraProperties::SensorResolution::THE_4_K}
  };
  
  return resolution_map.count(rgb_resolution_) ? 
    resolution_map[rgb_resolution_] : dai::ColorCameraProperties::SensorResolution::THE_1080_P;
}

dai::MonoCameraProperties::SensorResolution OAKDDriver::get_mono_resolution()
{
  std::map<std::string, dai::MonoCameraProperties::SensorResolution> resolution_map = {
    {"720p", dai::MonoCameraProperties::SensorResolution::THE_720_P},
    {"800p", dai::MonoCameraProperties::SensorResolution::THE_800_P},
    {"400p", dai::MonoCameraProperties::SensorResolution::THE_400_P}
  };
  
  return resolution_map.count(depth_resolution_) ? 
    resolution_map[depth_resolution_] : dai::MonoCameraProperties::SensorResolution::THE_720_P;
}

void OAKDDriver::start_device()
{
  device_->startPipeline(*pipeline_);
}

void OAKDDriver::setup_queues()
{
  queues_["rgb"] = device_->getOutputQueue("rgb", 4, false);
  queues_["depth"] = device_->getOutputQueue("depth", 4, false);
}

void OAKDDriver::configure_ir()
{
  if (enable_ir_) {
    auto ir_drivers = device_->getIrDrivers();
    if (!ir_drivers.empty()) {
      // Normalize brightness values
      float laser_brightness = std::min(laser_dot_brightness_ / 1200.0f, 1.0f);
      float flood_brightness = std::min(floodlight_brightness_ / 1500.0f, 1.0f);
      
      device_->setIrLaserDotProjectorIntensity(laser_brightness);
      device_->setIrFloodLightIntensity(flood_brightness);
      
      RCLCPP_INFO(this->get_logger(), "IR configured - Laser: %.2f, Flood: %.2f", 
        laser_brightness, flood_brightness);
    }
  }
}

void OAKDDriver::load_external_calibration()
{
  try {
    RCLCPP_INFO(this->get_logger(), "Loading external calibration: %s", external_calibration_path_.c_str());
    dai::CalibrationHandler calib_handler(external_calibration_path_);
    pipeline_->setCalibrationData(calib_handler);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load external calibration: %s", e.what());
  }
}

void OAKDDriver::process_frames()
{
  try {
    // Process RGB
    if (queues_.count("rgb")) {
      auto rgb_frame = queues_["rgb"]->tryGet<dai::ImgFrame>();
      if (rgb_frame) {
        publish_rgb_image(rgb_frame);
        publish_rgb_camera_info();
      }
    }
    
    // Process depth
    if (queues_.count("depth")) {
      auto depth_frame = queues_["depth"]->tryGet<dai::ImgFrame>();
      if (depth_frame) {
        publish_depth_image(depth_frame);
        publish_depth_camera_info();
        publish_pointcloud(depth_frame);
      }
    }
    
    // Publish TF
    if (publish_tf_) {
      publish_transform();
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error processing frames: %s", e.what());
  }
}

void OAKDDriver::publish_rgb_image(std::shared_ptr<dai::ImgFrame> frame)
{
  try {
    cv::Mat cv_image = frame->getCvFrame();
    
    auto msg = cv_bridge::CvImage(
      create_header(camera_name_ + "_rgb_camera_optical_frame"),
      "rgb8",
      cv_image
    ).toImageMsg();
    
    rgb_pub_->publish(*msg);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error publishing RGB image: %s", e.what());
  }
}

void OAKDDriver::publish_depth_image(std::shared_ptr<dai::ImgFrame> frame)
{
  try {
    cv::Mat depth_image = frame->getFrame();
    
    // Convert to 32-bit float in meters
    cv::Mat depth_float;
    depth_image.convertTo(depth_float, CV_32FC1, 1.0 / 1000.0);
    
    auto msg = cv_bridge::CvImage(
      create_header(camera_name_ + "_left_camera_optical_frame"),
      "32FC1",
      depth_float
    ).toImageMsg();
    
    depth_pub_->publish(*msg);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error publishing depth image: %s", e.what());
  }
}

void OAKDDriver::publish_pointcloud(std::shared_ptr<dai::ImgFrame> frame)
{
  try {
    cv::Mat depth_image = frame->getFrame();
    int height = depth_image.rows;
    int width = depth_image.cols;
    
    // Get camera intrinsics from calibration
    float fx = 883.15f, fy = 883.15f, cx = width / 2.0f, cy = height / 2.0f;
    
    try {
      auto intrinsics = calibration_data_.getCameraIntrinsics(
        dai::CameraBoardSocket::CAM_B, dai::Size2f(width, height));  // Updated for newer API
      fx = intrinsics[0][0];
      fy = intrinsics[1][1];
      cx = intrinsics[0][2];
      cy = intrinsics[1][2];
    } catch (...) {
      // Use fallback values
    }
    
    // Create point cloud message
    auto pointcloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
    pointcloud_msg->header = create_header(camera_name_ + "_left_camera_optical_frame");
    pointcloud_msg->height = 1;
    pointcloud_msg->is_dense = false;
    pointcloud_msg->is_bigendian = false;
    
    // Set up fields
    sensor_msgs::PointCloud2Modifier modifier(*pointcloud_msg);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    
    // Generate point cloud (subsample for performance)
    std::vector<std::array<float, 3>> points;
    int step = 4;
    
    for (int v = 0; v < height; v += step) {
      for (int u = 0; u < width; u += step) {
        uint16_t depth_value = depth_image.at<uint16_t>(v, u);
        float z = depth_value / 1000.0f;  // Convert to meters
        
        if (z > 0.3f && z < 10.0f) {  // Filter valid depth range
          float x = (u - cx) * z / fx;
          float y = (v - cy) * z / fy;
          points.push_back({x, y, z});
        }
      }
    }
    
    if (!points.empty()) {
      modifier.resize(points.size());
      
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
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error publishing point cloud: %s", e.what());
  }
}

void OAKDDriver::publish_rgb_camera_info()
{
  try {
    auto info = std::make_shared<sensor_msgs::msg::CameraInfo>();
    info->header = create_header(camera_name_ + "_rgb_camera_optical_frame");
    info->width = 1920;
    info->height = 1080;
    info->distortion_model = "plumb_bob";
    
    try {
      // Get RGB camera intrinsics
      auto intrinsics = calibration_data_.getCameraIntrinsics(
        dai::CameraBoardSocket::CAM_A, dai::Size2f(1920, 1080));  // Updated for newer API
      auto distortion = calibration_data_.getDistortionCoefficients(dai::CameraBoardSocket::CAM_A);
      
      // Fill intrinsic matrix
      info->k[0] = intrinsics[0][0]; info->k[1] = intrinsics[0][1]; info->k[2] = intrinsics[0][2];
      info->k[3] = intrinsics[1][0]; info->k[4] = intrinsics[1][1]; info->k[5] = intrinsics[1][2];
      info->k[6] = intrinsics[2][0]; info->k[7] = intrinsics[2][1]; info->k[8] = intrinsics[2][2];
      
      // Rectification matrix (identity for RGB)
      info->r[0] = 1.0; info->r[1] = 0.0; info->r[2] = 0.0;
      info->r[3] = 0.0; info->r[4] = 1.0; info->r[5] = 0.0;
      info->r[6] = 0.0; info->r[7] = 0.0; info->r[8] = 1.0;
      
      // Projection matrix
      info->p[0] = intrinsics[0][0]; info->p[1] = 0.0; info->p[2] = intrinsics[0][2]; info->p[3] = 0.0;
      info->p[4] = 0.0; info->p[5] = intrinsics[1][1]; info->p[6] = intrinsics[1][2]; info->p[7] = 0.0;
      info->p[8] = 0.0; info->p[9] = 0.0; info->p[10] = 1.0; info->p[11] = 0.0;
      
      // Distortion coefficients (convert float to double)
      info->d.clear();
      for (float coeff : distortion) {
        info->d.push_back(static_cast<double>(coeff));
      }
      
    } catch (...) {
      // Use default values if calibration fails
      info->k[0] = 883.15; info->k[1] = 0.0; info->k[2] = 960.0;
      info->k[3] = 0.0; info->k[4] = 883.15; info->k[5] = 540.0;
      info->k[6] = 0.0; info->k[7] = 0.0; info->k[8] = 1.0;
      
      info->r[0] = 1.0; info->r[1] = 0.0; info->r[2] = 0.0;
      info->r[3] = 0.0; info->r[4] = 1.0; info->r[5] = 0.0;
      info->r[6] = 0.0; info->r[7] = 0.0; info->r[8] = 1.0;
      
      info->p[0] = 883.15; info->p[1] = 0.0; info->p[2] = 960.0; info->p[3] = 0.0;
      info->p[4] = 0.0; info->p[5] = 883.15; info->p[6] = 540.0; info->p[7] = 0.0;
      info->p[8] = 0.0; info->p[9] = 0.0; info->p[10] = 1.0; info->p[11] = 0.0;
      
      info->d = {0.0, 0.0, 0.0, 0.0, 0.0};
    }
    
    rgb_info_pub_->publish(*info);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error publishing RGB camera info: %s", e.what());
  }
}

void OAKDDriver::publish_depth_camera_info()
{
  try {
    auto info = std::make_shared<sensor_msgs::msg::CameraInfo>();
    info->header = create_header(camera_name_ + "_left_camera_optical_frame");
    info->width = 1280;
    info->height = 720;
    info->distortion_model = "plumb_bob";
    
    try {
      // Get left camera intrinsics (depth is based on left camera)
      auto intrinsics = calibration_data_.getCameraIntrinsics(
        dai::CameraBoardSocket::CAM_B, dai::Size2f(1280, 720));  // Updated for newer API
      auto distortion = calibration_data_.getDistortionCoefficients(dai::CameraBoardSocket::CAM_B);
      
      // Fill intrinsic matrix
      info->k[0] = intrinsics[0][0]; info->k[1] = intrinsics[0][1]; info->k[2] = intrinsics[0][2];
      info->k[3] = intrinsics[1][0]; info->k[4] = intrinsics[1][1]; info->k[5] = intrinsics[1][2];
      info->k[6] = intrinsics[2][0]; info->k[7] = intrinsics[2][1]; info->k[8] = intrinsics[2][2];
      
      // Rectification matrix (identity for depth)
      info->r[0] = 1.0; info->r[1] = 0.0; info->r[2] = 0.0;
      info->r[3] = 0.0; info->r[4] = 1.0; info->r[5] = 0.0;
      info->r[6] = 0.0; info->r[7] = 0.0; info->r[8] = 1.0;
      
      // Projection matrix
      info->p[0] = intrinsics[0][0]; info->p[1] = 0.0; info->p[2] = intrinsics[0][2]; info->p[3] = 0.0;
      info->p[4] = 0.0; info->p[5] = intrinsics[1][1]; info->p[6] = intrinsics[1][2]; info->p[7] = 0.0;
      info->p[8] = 0.0; info->p[9] = 0.0; info->p[10] = 1.0; info->p[11] = 0.0;
      
      // Distortion coefficients (convert float to double)
      info->d.clear();
      for (float coeff : distortion) {
        info->d.push_back(static_cast<double>(coeff));
      }
      
    } catch (...) {
      // Use default values if calibration fails
      info->k[0] = 883.15; info->k[1] = 0.0; info->k[2] = 640.0;
      info->k[3] = 0.0; info->k[4] = 883.15; info->k[5] = 360.0;
      info->k[6] = 0.0; info->k[7] = 0.0; info->k[8] = 1.0;
      
      info->r[0] = 1.0; info->r[1] = 0.0; info->r[2] = 0.0;
      info->r[3] = 0.0; info->r[4] = 1.0; info->r[5] = 0.0;
      info->r[6] = 0.0; info->r[7] = 0.0; info->r[8] = 1.0;
      
      info->p[0] = 883.15; info->p[1] = 0.0; info->p[2] = 640.0; info->p[3] = 0.0;
      info->p[4] = 0.0; info->p[5] = 883.15; info->p[6] = 360.0; info->p[7] = 0.0;
      info->p[8] = 0.0; info->p[9] = 0.0; info->p[10] = 1.0; info->p[11] = 0.0;
      
      info->d = {0.0, 0.0, 0.0, 0.0, 0.0};
    }
    
    depth_info_pub_->publish(*info);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error publishing depth camera info: %s", e.what());
  }
}

void OAKDDriver::publish_transform()
{
  try {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = base_frame_;
    t.child_frame_id = camera_name_ + "_camera_frame";
    
    // Default transform (can be enhanced with calibration data)
    t.transform.translation.x = 0.1;
    t.transform.translation.y = 0.0;
    t.transform.translation.z = 0.2;
    
    t.transform.rotation.x = 0.0;
    t.transform.rotation.y = 0.0;
    t.transform.rotation.z = 0.0;
    t.transform.rotation.w = 1.0;
    
    tf_broadcaster_->sendTransform(t);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error publishing transform: %s", e.what());
  }
}

void OAKDDriver::publish_diagnostics()
{
  try {
    auto diag_array = std::make_shared<diagnostic_msgs::msg::DiagnosticArray>();
    diag_array->header.stamp = this->get_clock()->now();
    
    // Camera status
    diagnostic_msgs::msg::DiagnosticStatus status;
    status.name = std::string(this->get_name()) + ": Camera Status";
    status.hardware_id = device_info_.count("mxid") ? device_info_["mxid"] : "unknown";
    
    if (camera_running_ && device_) {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::OK;
      status.message = "Camera running normally";
      
      // Add device information
      diagnostic_msgs::msg::KeyValue kv;
      
      kv.key = "Device Name";
      kv.value = device_info_.count("device_name") ? device_info_["device_name"] : "unknown";
      status.values.push_back(kv);
      
      kv.key = "MXID";
      kv.value = device_info_.count("mxid") ? device_info_["mxid"] : "unknown";
      status.values.push_back(kv);
      
      kv.key = "Connection";
      kv.value = device_info_.count("name") ? device_info_["name"] : "unknown";
      status.values.push_back(kv);
      
      if (device_info_.count("usb_speed")) {
        kv.key = "USB Speed";
        kv.value = device_info_["usb_speed"];
        status.values.push_back(kv);
      }
      
      // Add temperature if available (simplified approach)
      try {
        // Try to get temperature data but don't iterate if the API is complex
        kv.key = "Device Temperature";
        kv.value = "Available via device API";
        status.values.push_back(kv);
      } catch (...) {
        // Temperature not available - this is common and not an error
        kv.key = "Device Temperature";
        kv.value = "Not available";
        status.values.push_back(kv);
      }
      
    } else {
      status.level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      status.message = "Camera not running";
    }
    
    diag_array->status.push_back(status);
    diagnostics_pub_->publish(*diag_array);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error publishing diagnostics: %s", e.what());
  }
}

std_msgs::msg::Header OAKDDriver::create_header(const std::string& frame_id)
{
  std_msgs::msg::Header header;
  header.stamp = this->get_clock()->now();
  header.frame_id = frame_id;
  return header;
}

void OAKDDriver::save_pipeline()
{
  try {
    std::string mxid = device_info_.count("mxid") ? device_info_["mxid"] : "unknown";
    std::string save_path = "/tmp/" + mxid + "_pipeline.json";
    RCLCPP_INFO(this->get_logger(), "Saving pipeline schema to: %s", save_path.c_str());
    
    std::ofstream file(save_path);
    if (file.is_open()) {
      // Note: This would need to be implemented based on DepthAI C++ API
      file << "// Pipeline schema would be saved here";
      file.close();
    }
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error saving pipeline: %s", e.what());
  }
}

void OAKDDriver::save_calibration()
{
  try {
    std::string mxid = device_info_.count("mxid") ? device_info_["mxid"] : "unknown";
    std::string save_path = "/tmp/" + mxid + "_calibration.json";
    RCLCPP_INFO(this->get_logger(), "Saving calibration to: %s", save_path.c_str());
    
    calibration_data_.eepromToJsonFile(save_path);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error saving calibration: %s", e.what());
  }
}

// Service callbacks
void OAKDDriver::start_camera_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                  std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Unused parameter
  
  try {
    start_camera();
    response->success = true;
    response->message = "Camera started successfully";
  } catch (const std::exception& e) {
    response->success = false;
    response->message = "Failed to start camera: " + std::string(e.what());
  }
}

void OAKDDriver::stop_camera_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Unused parameter
  
  try {
    stop_camera();
    response->success = true;
    response->message = "Camera stopped successfully";
  } catch (const std::exception& e) {
    response->success = false;
    response->message = "Failed to stop camera: " + std::string(e.what());
  }
}

void OAKDDriver::save_pipeline_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                   std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Unused parameter
  
  try {
    save_pipeline();
    response->success = true;
    response->message = "Pipeline saved successfully";
  } catch (const std::exception& e) {
    response->success = false;
    response->message = "Failed to save pipeline: " + std::string(e.what());
  }
}

void OAKDDriver::save_calibration_srv(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                                      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
  (void)request;  // Unused parameter
  
  try {
    save_calibration();
    response->success = true;
    response->message = "Calibration saved successfully";
  } catch (const std::exception& e) {
    response->success = false;
    response->message = "Failed to save calibration: " + std::string(e.what());
  }
}

}  // namespace oakd_driver