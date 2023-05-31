#include "zed_cpu.hpp"

int main(int argc, char * argv[])
{
  ros::init(argc, argv, "zed_camera");

  auto nh = std::make_shared<ros::NodeHandle>("~");
  auto it = std::make_shared<image_transport::ImageTransport>(*nh);

  ZedCameraNode zed_camera_node(nh, it);
  while (ros::ok()) {
    zed_camera_node.run();
    ros::spinOnce();
  }

  ros::shutdown();

  return EXIT_SUCCESS;
}