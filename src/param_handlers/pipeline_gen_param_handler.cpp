
#include "depthai_ros_driver/param_handlers/pipeline_gen_param_handler.hpp"

#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace param_handlers {
PipelineGenParamHandler::PipelineGenParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name)
    : BaseParamHandler(node, name) {}
PipelineGenParamHandler::~PipelineGenParamHandler() = default;

void PipelineGenParamHandler::declareParams() {
    declareAndLogParam<std::string>("i_pipeline_type", "RGBD");
    declareAndLogParam<std::string>("i_nn_type", "none");
    declareAndLogParam<bool>("i_enable_imu", true);
    declareAndLogParam<bool>("i_enable_diagnostics", false);
    declareAndLogParam<bool>("i_enable_rgbd", false);
    declareAndLogParam<bool>("i_enable_vio", false);
    // declareAndLogParam<bool>("i_enable_slam", false);
}

dai::CameraControl PipelineGenParamHandler::setRuntimeParams(const std::vector<rclcpp::Parameter>& params) {
    dai::CameraControl ctrl;
    // PipelineGenParamHandler doesn't have runtime parameters to set
    // Return empty control object
    return ctrl;
}
}  // namespace param_handlers
}  // namespace depthai_ros_driver
