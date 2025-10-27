cmake_minimum_required(VERSION 3.5)
project(oakd_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(camera_info_manager REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Find DepthAI
find_package(depthai CONFIG REQUIRED)

# Print debug information
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV include dirs: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "cv_bridge found: ${cv_bridge_FOUND}")

# Include directories
include_directories(include)

# Create the simple test executable first (no complex dependencies)
add_executable(simple_oakd_driver
  src/simple_oakd_driver.cpp
)

ament_target_dependencies(simple_oakd_driver
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Create the oakd_publisher_node executable (test mode)
add_executable(oakd_publisher_node
  src/oakd_publisher_node.cpp
)

ament_target_dependencies(oakd_publisher_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
)

# Only build the full driver if all dependencies are available
find_package(depthai CONFIG QUIET)
if(depthai_FOUND AND cv_bridge_FOUND)
  message(STATUS "Building full OAK-D driver with DepthAI support")
  
  # Create the oakd_driver_node executable
  add_executable(oakd_driver_node
    src/oakd_driver_node.cpp
    src/oakd_driver.cpp
  )

  # Specify libraries to link
  ament_target_dependencies(oakd_driver_node
    rclcpp
    rclcpp_components
    std_msgs
    std_srvs
    sensor_msgs
    geometry_msgs
    diagnostic_msgs
    tf2_ros
    tf2_geometry_msgs
    cv_bridge
    image_transport
    camera_info_manager
  )

  # Link OpenCV and DepthAI
  target_link_libraries(oakd_driver_node
    ${OpenCV_LIBRARIES}
    depthai::opencv
  )
  
  set(FULL_DRIVER_BUILT TRUE)
else()
  message(WARNING "DepthAI or cv_bridge not found. Building simple test driver only.")
  message(STATUS "To install DepthAI: wget -qO- https://raw.githubusercontent.com/luxonis/depthai/main/install_dependencies.sh | bash")
  message(STATUS "Then: sudo apt install libdepthai-dev")
  message(STATUS "For cv_bridge: sudo apt install ros-jazzy-cv-bridge")
  set(FULL_DRIVER_BUILT FALSE)
endif()

target_link_libraries(oakd_publisher_node
  ${OpenCV_LIBRARIES}
)

# Install executables
if(FULL_DRIVER_BUILT)
  install(TARGETS
    oakd_driver_node
    oakd_publisher_node
    simple_oakd_driver
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  install(TARGETS
    oakd_publisher_node
    simple_oakd_driver
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install include headers
install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()