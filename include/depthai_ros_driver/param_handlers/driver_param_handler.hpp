#pragma once

#include <string>
#include <unordered_map>

#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
enum class UsbSpeed;
}

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {

class DriverParamHandler : public BaseParamHandler {
   public:
    explicit DriverParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name);
    ~DriverParamHandler();
    dai::CameraControl setRuntimeParams(const std::vector<rclcpp::Parameter>& params) override;
    void declareParams();
    dai::UsbSpeed getUSBSpeed();

   private:
    std::unordered_map<std::string, dai::UsbSpeed> usbSpeedMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
