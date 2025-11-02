cmake_minimum_required(VERSION 3.22)
project(oakd_driver VERSION 1.0.0)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_SHARED_LIBS ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -Wno-deprecated-declarations -Wno-maybe-uninitialized)
    # Reduce memory usage during compilation to avoid "Killed" errors
    add_compile_options(-g1)  # Reduced debug info
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        add_compile_options(-O1)  # Basic optimization in debug mode
    endif()
endif()
# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(backward_ros REQUIRED)

# Explicitly find critical packages to ensure proper linking order
# Order matters - find depthai_bridge before depthai to ensure compatibility
find_package(depthai_bridge REQUIRED)
find_package(depthai REQUIRED)
find_package(depthai_ros_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_auto_find_build_dependencies()

set(COMMON_DEPS
${camera_info_manager_TARGETS}
cv_bridge::cv_bridge
depthai::core
${depthai_bridge_TARGETS}
image_transport::image_transport
rclcpp::rclcpp
${sensor_msgs_TARGETS}
diagnostic_updater::diagnostic_updater
${diagnostic_msgs_TARGETS}
${ffmpeg_image_transport_msgs_TARGETS}
)

set(SENSOR_DEPS
cv_bridge::cv_bridge
${camera_info_manager_TARGETS}
depthai::core
${depthai_bridge_TARGETS}
rclcpp::rclcpp
${geometry_msgs_TARGETS}
${depthai_ros_msgs_TARGETS}
${nav_msgs_TARGETS}
${std_msgs_TARGETS}
${sensor_msgs_TARGETS}
${std_srvs_TARGETS}
tf2_ros::tf2_ros
image_transport::image_transport
)

set(NN_DEPS
depthai::core
${depthai_bridge_TARGETS}
rclcpp::rclcpp
${vision_msgs_TARGETS}
)

set(CAM_DEPS
depthai::core
rclcpp::rclcpp
${rclcpp_components_TARGETS}
${std_srvs_TARGETS}
${pluginlib_TARGETS}
)


include_directories(include)

# Ensure we use ROS2 DepthAI headers only
include_directories(BEFORE SYSTEM /opt/ros/jazzy/include)

# Explicitly exclude system-wide DepthAI if it exists
if(EXISTS "/usr/local/include/depthai")
    message(WARNING "System-wide DepthAI installation detected at /usr/local/include/depthai. This may cause conflicts. Consider removing it.")
endif()

set(COMMON_LIB_NAME ${PROJECT_NAME}_common)
add_library(
  ${COMMON_LIB_NAME} SHARED
  src/utils.cpp
  src/dai_nodes/base_node.cpp
  src/dai_nodes/sys_logger.cpp
  src/dai_nodes/sensors/img_pub.cpp
  src/dai_nodes/sensors/sensor_helpers.cpp
  src/param_handlers/driver_param_handler.cpp
  src/param_handlers/imu_param_handler.cpp
  src/param_handlers/nn_param_handler.cpp
  src/param_handlers/sensor_param_handler.cpp
  src/param_handlers/feature_tracker_param_handler.cpp
  # src/param_handlers/rgbd_param_handler.cpp  # Missing RGBD.hpp
  # src/param_handlers/vio_param_handler.cpp   # Missing BasaltVIO.hpp
  src/param_handlers/stereo_param_handler.cpp
  # src/param_handlers/thermal_param_handler.cpp  # Missing ThermalConfig.hpp
  # src/param_handlers/tof_param_handler.cpp      # API compatibility issues
  src/param_handlers/pipeline_gen_param_handler.cpp
  src/param_handlers/sync_param_handler.cpp
)

target_link_libraries(${COMMON_LIB_NAME}
PUBLIC
  ${COMMON_DEPS}
)

set(SENSOR_LIB_NAME ${PROJECT_NAME}_sensor_nodes)
add_library(
  ${SENSOR_LIB_NAME} SHARED
  # src/dai_nodes/sensors/imu.cpp           # API compatibility issues
  # src/dai_nodes/sensors/camera.cpp        # API compatibility issues
  # src/dai_nodes/sensors/feature_tracker.cpp # API compatibility issues
  # src/dai_nodes/sensors/sensor_wrapper.cpp  # API compatibility issues
  # src/dai_nodes/sensors/stereo.cpp         # Missing capabilities header
  # src/dai_nodes/sensors/thermal.cpp       # Missing ThermalConfig.hpp
  # src/dai_nodes/sensors/rgbd.cpp          # Missing RGBD.hpp
  # src/dai_nodes/sensors/vio.cpp           # Missing BasaltVIO.hpp
  # src/dai_nodes/sensors/tof.cpp           # API compatibility issues
  # src/dai_nodes/sensors/sync.cpp          # API compatibility issues
)

target_link_libraries(${SENSOR_LIB_NAME}
PUBLIC
  ${SENSOR_DEPS}
  ${COMMON_LIB_NAME}
)

set(NN_LIB_NAME ${PROJECT_NAME}_nn_nodes)
add_library(
  ${NN_LIB_NAME} SHARED
  # src/dai_nodes/nn/nn_wrapper.cpp         # Missing modelzoo header
  # src/dai_nodes/nn/spatial_nn_wrapper.cpp # Missing MessageQueue header
  # src/dai_nodes/nn/segmentation.cpp       # API compatibility issues
)

target_link_libraries(${NN_LIB_NAME}
PUBLIC
  ${NN_DEPS}
  ${COMMON_LIB_NAME}
)

add_library(
  ${PROJECT_NAME} SHARED
  src/driver.cpp
  src/pipeline/pipeline_generator.cpp
  src/pipeline/base_pipeline.cpp
  src/pipeline/base_types.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
PUBLIC
  ${CAM_DEPS}
  ${SENSOR_LIB_NAME}
  ${NN_LIB_NAME}
  ${COMMON_LIB_NAME}
)

add_executable(
  oakd_driver_node
  src/driver_node.cpp)
target_link_libraries(oakd_driver_node
PUBLIC
  ${PROJECT_NAME}
  ${SENSOR_LIB_NAME}
  ${NN_LIB_NAME}
  ${COMMON_LIB_NAME})

rclcpp_components_register_nodes(${PROJECT_NAME} "depthai_ros_driver::Driver")
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
ament_export_include_directories(include)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${COMMON_DEPS} ${SENSOR_DEPS} ${NN_DEPS} ${CAM_DEPS})

install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME} ${SENSOR_LIB_NAME}
${NN_LIB_NAME}
${COMMON_LIB_NAME}
EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
        INCLUDES DESTINATION include
)

install(EXPORT ${PROJECT_NAME}Targets
      DESTINATION share/${PROJECT_NAME}/cmake)
install(TARGETS oakd_driver_node DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(
  include
)

install(
    DIRECTORY include/
    DESTINATION include
)

ament_package()