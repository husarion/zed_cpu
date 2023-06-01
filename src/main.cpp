#include "zed_cpu.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto zed_camera_node = std::make_shared<zed_cpu::ZedCameraNode>();

  while (rclcpp::ok()) {
    zed_camera_node->run();
    rclcpp::spin_some(zed_camera_node);
  }

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
