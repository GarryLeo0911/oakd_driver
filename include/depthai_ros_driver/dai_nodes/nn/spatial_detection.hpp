#pragma once

#include <memory>
#include <string>
#include <vector>

#include "camera_info_manager/camera_info_manager.hpp"
#include "depthai/common/CameraBoardSocket.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/device/DataQueue.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/NeuralNetwork.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_bridge/SpatialDetectionConverter.hpp"
#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/dai_nodes/nn/nn_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_wrapper.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/stereo.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"
#include "depthai_ros_driver/param_handlers/nn_param_handler.hpp"
#include "rclcpp/node.hpp"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace nn {
class SpatialDetection : public BaseNode {
   public:
    SpatialDetection(const std::string& daiNodeName,
                     std::shared_ptr<rclcpp::Node> node,
                     std::shared_ptr<dai::Pipeline> pipeline,
                     const std::string& deviceName,
                     bool rsCompat,
                     SensorWrapper& /* camNode */,
                     Stereo& /* stereoNode */,
                     const dai::CameraBoardSocket& socket = dai::CameraBoardSocket::CAM_A)
        : BaseNode(daiNodeName, node, pipeline, deviceName, rsCompat) {
        RCLCPP_DEBUG(getLogger(), "Creating node %s", daiNodeName.c_str());
        setNames();
        spatialNode = pipeline->create<dai::node::NeuralNetwork>();
        ph = std::make_unique<param_handlers::NNParamHandler>(node, daiNodeName, socket);
        ph->declareParams(spatialNode);
        // Set model path directly instead of using deprecated build() method
        spatialNode->setBlobPath(ph->getParam<std::string>("i_nn_model"));
        spatialNode->setNumInferenceThreads(2);
        spatialNode->input.setBlocking(false);
        spatialNode->input.setQueueSize(1);
        RCLCPP_DEBUG(getLogger(), "Node %s created", daiNodeName.c_str());
        setInOut(pipeline);
    }
    ~SpatialDetection() = default;
    void setupQueues(std::shared_ptr<dai::Device> device) override {
        nnQ = device->getOutputQueue(nnQName, ph->getParam<int>("i_max_q_size"), false);
        std::string socketName = getSocketName(ph->getSocketID());
        auto tfPrefix = getOpticalFrameName(socketName);
        detConverter = std::make_unique<dai::ros::SpatialDetectionConverter>(tfPrefix, false, ph->getParam<bool>("i_get_base_device_timestamp"));
        detConverter->setUpdateRosBaseTimeOnToRosMsg(ph->getParam<bool>("i_update_ros_base_time_on_ros_msg"));
        nnQ->addCallback(std::bind(&SpatialDetection::spatialCB, this, std::placeholders::_1, std::placeholders::_2));
        rclcpp::PublisherOptions options;
        options.qos_overriding_options = rclcpp::QosOverridingOptions();
        detPub = getROSNode()->template create_publisher<vision_msgs::msg::Detection3DArray>("~/" + getName() + "/spatial_detections", 10, options);

        if(ph->getParam<bool>("i_enable_passthrough")) {
            utils::ImgConverterConfig convConf;
            convConf.tfPrefix = tfPrefix;
            convConf.getBaseDeviceTimestamp = ph->getParam<bool>("i_get_base_device_timestamp");
            convConf.updateROSBaseTimeOnRosMsg = ph->getParam<bool>("i_update_ros_base_time_on_ros_msg");

            utils::ImgPublisherConfig pubConf;
            pubConf.daiNodeName = getName();
            pubConf.topicName = "~/" + getName() + "/passthrough";
            pubConf.infoSuffix = "/passthrough";
            pubConf.socket = static_cast<dai::CameraBoardSocket>(ph->getParam<int>("i_board_socket_id"));

            ptPub->setup(device, convConf, pubConf);
            ptQ = device->getOutputQueue(ptQName, ph->getParam<int>("i_max_q_size"), false);
        }

        // Note: passthroughDepth not available in NeuralNetwork
        // if(ph->getParam<bool>("i_enable_passthrough_depth")) {
        //     // Depth passthrough setup would go here
        // }
    };
    void link(dai::Node::Input& in, int /*linkType = 0*/) override {
        spatialNode->out.link(in);
    };
    dai::Node::Input& getInput(int /* linkType */ = 0) override {
        return spatialNode->input;
    };
    void setNames() override {
        nnQName = getName() + "_nn";
        ptQName = getName() + "_pt";
        ptDepthQName = getName() + "_pt_depth";
    };
    void setInOut(std::shared_ptr<dai::Pipeline> pipeline) override {
        // Create XLinkOut node for spatial detection output
        auto spatialOut = pipeline->create<dai::node::XLinkOut>();
        spatialOut->setStreamName(nnQName);
        spatialNode->out.link(spatialOut->input);
        
        if(ph->getParam<bool>("i_enable_passthrough")) {
            ptPub = setupOutput(pipeline, ptQName, &spatialNode->passthrough);
        }
        // Note: passthroughDepth not available in NeuralNetwork, only in SpatialDetectionNetwork
        // if(ph->getParam<bool>("i_enable_passthrough_depth")) {
        //     ptDepthPub = setupOutput(pipeline, ptDepthQName, &spatialNode->passthroughDepth);
        // }
    };
    void closeQueues() override {
        nnQ->close();
        if(ph->getParam<bool>("i_enable_passthrough")) {
            ptQ->close();
        }
        // Note: passthroughDepth not available in NeuralNetwork
        // if(ph->getParam<bool>("i_enable_passthrough_depth")) {
        //     ptDepthQ->close();
        // }
    };

   private:
    void spatialCB(const std::string& /*name*/, const std::shared_ptr<dai::ADatatype>& data) {
        auto inDet = std::dynamic_pointer_cast<dai::SpatialImgDetections>(data);
        std::deque<vision_msgs::msg::Detection3DArray> deq;
        detConverter->toRosVisionMsg(inDet, deq);
        while(deq.size() > 0) {
            auto currMsg = deq.front();
            detPub->publish(currMsg);
            deq.pop_front();
        }
    };
    std::unique_ptr<dai::ros::SpatialDetectionConverter> detConverter;
    std::vector<std::string> labelNames;
    rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detPub;
    std::shared_ptr<dai::ros::ImageConverter> ptImageConverter, ptDepthImageConverter;
    std::shared_ptr<sensor_helpers::ImagePublisher> ptPub, ptDepthPub;
    std::shared_ptr<camera_info_manager::CameraInfoManager> ptInfoMan, ptDepthInfoMan;
    std::shared_ptr<dai::node::NeuralNetwork> spatialNode;
    std::shared_ptr<dai::node::ImageManip> imageManip;
    std::unique_ptr<param_handlers::NNParamHandler> ph;
    std::shared_ptr<dai::DataOutputQueue> nnQ, ptQ, ptDepthQ;
    std::shared_ptr<dai::node::XLinkOut> xoutNN, xoutPT, xoutPTDepth;
    std::string nnQName, ptQName, ptDepthQName;
};

}  // namespace nn
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
