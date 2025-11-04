#include "depthai_ros_driver/param_handlers/driver_param_handler.hpp"

#include "depthai/common/UsbSpeed.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
DriverParamHandler::DriverParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat)
    : BaseParamHandler(node, name, deviceName, rsCompat) {
    usbSpeedMap = {
        {"LOW", dai::UsbSpeed::LOW},
        {"FULL", dai::UsbSpeed::FULL},
        {"HIGH", dai::UsbSpeed::HIGH},
        {"SUPER", dai::UsbSpeed::SUPER},
        {"SUPER_PLUS", dai::UsbSpeed::SUPER_PLUS},
    };
}
DriverParamHandler::~DriverParamHandler() = default;

dai::UsbSpeed DriverParamHandler::getUSBSpeed() {
    return utils::getValFromMap(getParam<std::string>("i_usb_speed"), usbSpeedMap);
}

rclcpp::QoS DriverParamHandler::getQoSProfile() {
    int depth = getParam<int>("i_qos_depth");
    rclcpp::QoS qos(depth);
    
    // Set reliability
    std::string reliability = getParam<std::string>("i_qos_reliability");
    if (reliability == "best_effort") {
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    } else {
        qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
    }
    
    // Set durability
    std::string durability = getParam<std::string>("i_qos_durability");
    if (durability == "transient_local") {
        qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
    } else {
        qos.durability(rclcpp::DurabilityPolicy::Volatile);
    }
    
    // Set history
    std::string history = getParam<std::string>("i_qos_history");
    if (history == "keep_all") {
        qos.history(rclcpp::HistoryPolicy::KeepAll);
    } else {
        qos.history(rclcpp::HistoryPolicy::KeepLast);
    }
    
    return qos;
}
void DriverParamHandler::declareParams() {
    declareAndLogParam<bool>("i_enable_ir", true);
    declareAndLogParam<std::string>("i_usb_speed", "SUPER");
    declareAndLogParam<std::string>("i_device_id", "");
    declareAndLogParam<std::string>("i_ip", "");
    declareAndLogParam<std::string>("i_usb_port_id", "");
    declareAndLogParam<bool>("i_pipeline_dump", false);
    declareAndLogParam<bool>("i_calibration_dump", false);
    declareAndLogParam<std::string>("i_external_calibration_path", "");
    declareAndLogParam<float>("r_laser_dot_intensity", 0.6, getRangedFloatDescriptor(0.0, 1.0));
    declareAndLogParam<float>("r_floodlight_intensity", 0.6, getRangedFloatDescriptor(0.0, 1.0));
    declareAndLogParam<bool>("i_restart_on_diagnostics_error", false);
    declareAndLogParam<bool>("i_rs_compat", false);

    // QoS parameters for wireless robot communication
    declareAndLogParam<std::string>("i_qos_reliability", "reliable");      // "reliable" or "best_effort"
    declareAndLogParam<std::string>("i_qos_durability", "volatile");       // "volatile" or "transient_local"  
    declareAndLogParam<std::string>("i_qos_history", "keep_last");         // "keep_last" or "keep_all"
    declareAndLogParam<int>("i_qos_depth", 10);                            // Queue depth

    declareAndLogParam<bool>("i_publish_tf_from_calibration", true);
    declareAndLogParam<std::string>("i_tf_device_name", getROSNode()->get_name());
    declareAndLogParam<std::string>("i_tf_device_model", "");
    declareAndLogParam<std::string>("i_tf_base_frame", "oak");
    declareAndLogParam<std::string>("i_tf_parent_frame", "oak_parent_frame");
    declareAndLogParam<std::string>("i_tf_cam_pos_x", "0.0");
    declareAndLogParam<std::string>("i_tf_cam_pos_y", "0.0");
    declareAndLogParam<std::string>("i_tf_cam_pos_z", "0.0");
    declareAndLogParam<std::string>("i_tf_cam_roll", "0.0");
    declareAndLogParam<std::string>("i_tf_cam_pitch", "0.0");
    declareAndLogParam<std::string>("i_tf_cam_yaw", "0.0");
    declareAndLogParam<std::string>("i_tf_imu_from_descr", "false");
    declareAndLogParam<std::string>("i_tf_custom_urdf_location", "");
    declareAndLogParam<std::string>("i_tf_custom_xacro_args", "");
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver
