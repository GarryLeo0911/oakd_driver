#include "depthai_ros_driver/pipeline/pipeline_generator.hpp"

#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sync.hpp"
#include "depthai_ros_driver/dai_nodes/sys_logger.hpp"
#include "depthai_ros_driver/param_handlers/pipeline_gen_param_handler.hpp"
#include "depthai_ros_driver/pipeline/base_pipeline.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace pipeline_gen {

// Simple RGB Pipeline Implementation - No plugins needed
class SimpleRGBPipeline {
public:
    static std::vector<std::unique_ptr<dai_nodes::BaseNode>> createSimpleRGB(
        std::shared_ptr<rclcpp::Node> node,
        std::shared_ptr<dai::Device> device,
        std::shared_ptr<dai::Pipeline> pipeline,
        std::shared_ptr<param_handlers::PipelineGenParamHandler> ph) {
        
        (void)ph; // Mark parameter as intentionally unused
        std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
        
        try {
            // Create a simple RGB camera node with correct constructor parameters
            // Parameters: daiNodeName, node, pipeline, deviceName, rsCompat, socket, publish
            auto rgb = std::make_unique<dai_nodes::SensorWrapper>(
                "rgb",                           // daiNodeName
                node,                           // node
                pipeline,                       // pipeline
                device->getDeviceName(),        // deviceName
                false,                          // rsCompat
                dai::CameraBoardSocket::CAM_A,  // socket (main RGB camera)
                true                            // publish
            );
            daiNodes.push_back(std::move(rgb));
            
            RCLCPP_INFO(node->get_logger(), "Simple RGB pipeline created successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(node->get_logger(), "Error creating simple RGB pipeline: %s", e.what());
            throw;
        }
        
        return daiNodes;
    }
};

PipelineGenerator::PipelineGenerator() {
    pluginTypeMap = {{"RGB", "depthai_ros_driver::pipeline_gen::RGB"}};
    pipelineTypeMap = {{"RGB", PipelineType::RGB}};
}

PipelineGenerator::~PipelineGenerator() = default;

std::vector<std::unique_ptr<dai_nodes::BaseNode>> PipelineGenerator::createPipeline(std::shared_ptr<rclcpp::Node> node,
                                                                                    std::shared_ptr<dai::Device> device,
                                                                                    std::shared_ptr<dai::Pipeline> pipeline,
                                                                                    bool rsCompat) {
    auto deviceName = device->getDeviceName();
    RCLCPP_INFO(node->get_logger(), "Creating pipeline for device: %s", deviceName.c_str());
    ph = std::make_shared<param_handlers::PipelineGenParamHandler>(node, "pipeline_gen", deviceName, rsCompat);
    ph->declareParams();
    auto pipelineType = ph->getParam<std::string>("i_pipeline_type");
    auto nnType = ph->getParam<std::string>("i_nn_type");
    RCLCPP_INFO(node->get_logger(), "Pipeline type: %s", pipelineType.c_str());
    
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
    
    // Use direct implementation instead of plugin system to avoid memory issues
    if (pipelineType == "RGB") {
        RCLCPP_INFO(node->get_logger(), "Using direct RGB pipeline implementation (no plugins)");
        daiNodes = SimpleRGBPipeline::createSimpleRGB(node, device, pipeline, ph);
    } else {
        RCLCPP_ERROR(node->get_logger(), "Only RGB pipeline type is supported in this minimal implementation");
        throw std::runtime_error("Unsupported pipeline type");
    }

    // Skip diagnostics and sync to save memory
    RCLCPP_INFO(node->get_logger(), "Pipeline created with %zu nodes (diagnostics and sync disabled for memory optimization)", daiNodes.size());
    
    return daiNodes;
}

std::string PipelineGenerator::validatePipeline(std::shared_ptr<rclcpp::Node> node, const std::string& pipelineType, int sensorNum, const std::string& deviceName) {
    (void)sensorNum;   // Mark as intentionally unused
    (void)deviceName;  // Mark as intentionally unused
    
    // Only support RGB for now
    if (pipelineType != "RGB") {
        RCLCPP_ERROR(node->get_logger(), "Only RGB pipeline is supported in minimal mode");
        throw std::out_of_range("Unsupported pipeline type");
    }
    return pipelineType;
}

}  // namespace pipeline_gen
}  // namespace depthai_ros_driver