#include "zed_cpu.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("zed_camera");

  auto nh = node->create_sub_node("~");
  auto it = std::make_shared<image_transport::ImageTransport>(node);

  ZedCameraNode zedCameraNode(nh, it);
  while (rclcpp::ok()) {
    zedCameraNode.run();
    rclcpp::spin_some(node);
  }

  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
