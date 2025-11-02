#pragma once

#include <depthai/depthai.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "depthai/common/CameraFeatures.hpp"
#include "depthai/pipeline/datatype/CameraControl.hpp"
#include "depthai/pipeline/datatype/ImgFrame.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
namespace node {
class Camera;
}  // namespace node
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
class SensorParamHandler : public BaseParamHandler {
   public:
    explicit SensorParamHandler(
        std::shared_ptr<rclcpp::Node> node, const std::string& name, dai::CameraBoardSocket socket);
    ~SensorParamHandler();
    void declareCommonParams(dai::CameraBoardSocket socket);
    void declareParams(std::shared_ptr<dai::node::Camera> cam, bool publish);
    dai::CameraControl setRuntimeParams(const std::vector<rclcpp::Parameter>& params) override;

   private:
    dai::CameraBoardSocket socketID;
    std::unordered_map<dai::ImgFrame::Type, std::string> frameTypeMap;
    // Commenting out until we find the correct resize enum in newer DepthAI API
    // std::unordered_map<dai::ImgResizeMode, std::string> resizeTypeMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
