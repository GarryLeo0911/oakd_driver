// Stub implementations for missing constructors to satisfy the linker
// These are temporary implementations that will throw runtime errors if used

#include <stdexcept>
#include <string>
#include <memory>

// Forward declarations only - don't include full headers to avoid incomplete type issues
namespace rclcpp { class Node; }
namespace dai { 
    class Pipeline; 
    class Device; 
    namespace node { class StereoDepth; }
}

namespace depthai_ros_driver {
namespace dai_nodes {

// Forward declarations for the classes we need to stub
class BaseNode;
class Vio;
class RGBD; 
class Thermal;
class SensorWrapper;
class ToF;
class Stereo;
class Imu;

// Minimal BaseNode stub for inheritance
class BaseNode {
public:
    BaseNode(const std::string&, std::shared_ptr<rclcpp::Node>, std::shared_ptr<dai::Pipeline>, const std::string&, bool) {}
    virtual ~BaseNode() = default;
};

// VIO stub class declaration and implementation
class Vio : public BaseNode {
public:
    Vio(const std::string& daiNodeName,
        std::shared_ptr<rclcpp::Node> node,
        std::shared_ptr<dai::Pipeline> pipeline,
        std::shared_ptr<dai::Device> device,
        bool rsCompat,
        Stereo& stereo,
        Imu& imu)
        : BaseNode(daiNodeName, node, pipeline, "stub", rsCompat) {
        throw std::runtime_error("VIO functionality not implemented - missing DepthAI BasaltVIO support");
    }
};

// RGBD stub class declaration and implementations
class RGBD : public BaseNode {
public:
    RGBD(const std::string& daiNodeName,
         std::shared_ptr<rclcpp::Node> node,
         std::shared_ptr<dai::Pipeline> pipeline,
         std::shared_ptr<dai::Device> device,
         bool rsCompat,
         SensorWrapper& rgb,
         ToF& tof,
         bool aligned)
         : BaseNode(daiNodeName, node, pipeline, "stub", rsCompat) {
        throw std::runtime_error("RGBD with ToF functionality not implemented - missing DepthAI RGBD support");
    }

    RGBD(const std::string& daiNodeName,
         std::shared_ptr<rclcpp::Node> node,
         std::shared_ptr<dai::Pipeline> pipeline,
         std::shared_ptr<dai::Device> device,
         bool rsCompat,
         SensorWrapper& rgb,
         std::shared_ptr<dai::node::StereoDepth> stereoDepth,
         bool aligned)
         : BaseNode(daiNodeName, node, pipeline, "stub", rsCompat) {
        throw std::runtime_error("RGBD with StereoDepth functionality not implemented - missing DepthAI RGBD support");
    }
};

// Thermal stub class declaration and implementation
class Thermal : public BaseNode {
public:
    Thermal(const std::string& daiNodeName,
            std::shared_ptr<rclcpp::Node> node,
            std::shared_ptr<dai::Pipeline> pipeline,
            const std::string& deviceName,
            bool rsCompat)
            : BaseNode(daiNodeName, node, pipeline, deviceName, rsCompat) {
        throw std::runtime_error("Thermal functionality not implemented - missing DepthAI Thermal support");
    }
};

}  // namespace dai_nodes
}  // namespace depthai_ros_driver