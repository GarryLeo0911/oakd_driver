#pragma once

#include <depthai/pipeline/node/NeuralNetwork.hpp>
#include <fstream>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "nlohmann/json.hpp"

namespace dai {
namespace node {
class NeuralNetwork;
class ImageManip;
}  // namespace node
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
namespace nn {
enum class NNFamily { Segmentation, Detection };
}
class NNParamHandler : public BaseParamHandler {
   public:
    explicit NNParamHandler(std::shared_ptr<rclcpp::Node> node,
                            const std::string& name,
                            const dai::CameraBoardSocket& socket = dai::CameraBoardSocket::CAM_A);
    ~NNParamHandler();
    nn::NNFamily getNNFamily();
    dai::CameraControl setRuntimeParams(const std::vector<rclcpp::Parameter>& params) override;
    template <typename T>
    void declareParams(std::shared_ptr<T> nn) {
        declareAndLogParam<bool>("i_enable_passthrough", false);
        declareAndLogParam<bool>("i_enable_passthrough_depth", false);
        declareAndLogParam<bool>("i_get_base_device_timestamp", false);
        declareAndLogParam<bool>("i_update_ros_base_time_on_ros_msg", false);
        setNNParams(nn);
    }

    void setNNParams(std::shared_ptr<dai::node::NeuralNetwork> nn);

    void setSpatialParams(std::shared_ptr<dai::node::NeuralNetwork> /* nn */) {
        // Spatial parameters are handled directly in the neural network
        // These parameters may need to be set differently in newer API
    }

   private:
    std::string getModelPath(const nlohmann::json& data);
    std::unordered_map<std::string, nn::NNFamily> nnFamilyMap;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
