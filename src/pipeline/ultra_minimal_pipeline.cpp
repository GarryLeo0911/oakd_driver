#include "depthai_ros_driver/pipeline/pipeline_generator.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai_ros_driver/param_handlers/pipeline_gen_param_handler.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace pipeline_gen {

PipelineGenerator::PipelineGenerator() {
    pluginTypeMap = {
        {"RGB", "depthai_ros_driver::pipeline_gen::RGB"},
        {"RGBD", "depthai_ros_driver::pipeline_gen::RGBD"}  // Map RGBD to minimal RGB
    };
    pipelineTypeMap = {
        {"RGB", PipelineType::RGB},
        {"RGBD", PipelineType::RGB}  // Treat RGBD as RGB in minimal mode
    };
}

PipelineGenerator::~PipelineGenerator() = default;

std::vector<std::unique_ptr<dai_nodes::BaseNode>> PipelineGenerator::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                                    std::shared_ptr<dai::Device> device,
                                                                                    std::shared_ptr<dai::Pipeline> pipeline,
                                                                                    bool rsCompat) {
    auto deviceName = device->getDeviceName();
    RCLCPP_INFO(node->get_logger(), "Creating minimal pipeline for device: %s", deviceName.c_str());
    ph = std::make_shared<param_handlers::PipelineGenParamHandler>(node, "pipeline_gen", deviceName, rsCompat);
    ph->declareParams();
    auto pipelineType = ph->getParam<std::string>("i_pipeline_type");
    RCLCPP_INFO(node->get_logger(), "Pipeline type: %s", pipelineType.c_str());
    
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    
    // Ultra-minimal approach: Create empty node list but still connect the camera
    // This allows the device to initialize without complex node management
    if (pipelineType == "RGB" || pipelineType == "rgbd") {
        if (pipelineType == "rgbd") {
            RCLCPP_WARN(node->get_logger(), "RGBD pipeline requested but running in RGB-only minimal mode");
        }
        RCLCPP_INFO(node->get_logger(), "Creating minimal RGB pipeline (camera only, no ROS nodes)");
        
        // Create a basic DAI camera node directly in the pipeline without ROS wrapper
        auto camRgb = pipeline->create<dai::node::ColorCamera>();
        camRgb->setPreviewSize(320, 240);  // Very small size
        camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_720_P);
        camRgb->setInterleaved(false);
        camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::RGB);
        camRgb->setFps(10);  // Low FPS
        
        RCLCPP_INFO(node->get_logger(), "Basic camera node created in pipeline");
        
        // Return empty nodes list - the camera will be accessible but won't publish to ROS
        // This avoids all the complex memory allocation in sensor wrappers
    } else {
        RCLCPP_ERROR(node->get_logger(), "Pipeline type '%s' not supported in minimal mode. Supported: RGB, rgbd", pipelineType.c_str());
        throw std::runtime_error("Unsupported pipeline type");
    }

    RCLCPP_INFO(node->get_logger(), "Minimal pipeline created successfully with %zu ROS nodes", daiNodes.size());
    return daiNodes;
}

std::string PipelineGenerator::validatePipeline(std::shared_ptr<rclcpp::Node> node, const std::string& pipelineType, int sensorNum, const std::string& deviceName) {
    (void)sensorNum;   // Mark as intentionally unused
    (void)deviceName;  // Mark as intentionally unused
    
    // Support RGB and RGBD (but treat RGBD as RGB in minimal mode)
    if (pipelineType != "RGB" && pipelineType != "rgbd") {
        RCLCPP_ERROR(node->get_logger(), "Pipeline type '%s' not supported in minimal mode. Supported: RGB, rgbd", pipelineType.c_str());
        throw std::out_of_range("Unsupported pipeline type");
    }
    
    if (pipelineType == "rgbd") {
        RCLCPP_WARN(node->get_logger(), "RGBD pipeline will run in RGB-only minimal mode");
    }
    
    return pipelineType;
}

}  // namespace pipeline_gen
}  // namespace depthai_ros_driver