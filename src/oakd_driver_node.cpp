#include "oakd_driver/oakd_driver.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  try {
    auto node = std::make_shared<oakd_driver::OAKDDriver>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("oakd_driver"), "Error: %s", e.what());
    return 1;
  }
  
  rclcpp::shutdown();
  return 0;
}