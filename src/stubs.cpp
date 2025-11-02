// Stub implementations for missing constructors to satisfy the linker
// These are temporary implementations that will throw runtime errors if used

#include "depthai_ros_driver/dai_nodes/sensors/vio.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/rgbd.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/thermal.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/tof.hpp"
#include "stdexcept"

namespace depthai_ros_driver {
namespace dai_nodes {

// VIO stub constructors
Vio::Vio(const std::string& /*daiNodeName*/,
         std::shared_ptr<rclcpp::Node> /*node*/,
         std::shared_ptr<dai::Pipeline> /*pipeline*/,
         std::shared_ptr<dai::Device> /*device*/,
         bool /*rsCompat*/,
         Stereo& /*stereo*/,
         Imu& /*imu*/)
    : BaseNode("vio_stub", nullptr, nullptr, "stub", false) {
    throw std::runtime_error("VIO functionality not implemented - missing DepthAI BasaltVIO support");
}

// RGBD stub constructors
RGBD::RGBD(const std::string& /*daiNodeName*/,
           std::shared_ptr<rclcpp::Node> /*node*/,
           std::shared_ptr<dai::Pipeline> /*pipeline*/,
           std::shared_ptr<dai::Device> /*device*/,
           bool /*rsCompat*/,
           SensorWrapper& /*rgb*/,
           ToF& /*tof*/,
           bool /*aligned*/)
    : BaseNode("rgbd_stub", nullptr, nullptr, "stub", false) {
    throw std::runtime_error("RGBD with ToF functionality not implemented - missing DepthAI RGBD support");
}

RGBD::RGBD(const std::string& /*daiNodeName*/,
           std::shared_ptr<rclcpp::Node> /*node*/,
           std::shared_ptr<dai::Pipeline> /*pipeline*/,
           std::shared_ptr<dai::Device> /*device*/,
           bool /*rsCompat*/,
           SensorWrapper& /*rgb*/,
           std::shared_ptr<dai::node::StereoDepth> /*stereoDepth*/,
           bool /*aligned*/)
    : BaseNode("rgbd_stub", nullptr, nullptr, "stub", false) {
    throw std::runtime_error("RGBD with StereoDepth functionality not implemented - missing DepthAI RGBD support");
}

// Thermal stub constructor
Thermal::Thermal(const std::string& /*daiNodeName*/,
                 std::shared_ptr<rclcpp::Node> /*node*/,
                 std::shared_ptr<dai::Pipeline> /*pipeline*/,
                 const std::string& /*deviceName*/,
                 bool /*rsCompat*/)
    : BaseNode("thermal_stub", nullptr, nullptr, "stub", false) {
    throw std::runtime_error("Thermal functionality not implemented - missing DepthAI Thermal support");
}

}  // namespace dai_nodes
}  // namespace depthai_ros_driver