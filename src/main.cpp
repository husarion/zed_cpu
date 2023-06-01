#include "zed_cpu.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("zed_camera", "zed_camera");
  auto it = std::make_shared<image_transport::ImageTransport>(node);

  zed_cpu::ZedCameraNode zed_camera_node(node, it);
  while (rclcpp::ok()) {
    zed_camera_node.run();
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
