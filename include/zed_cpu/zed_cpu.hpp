// ZED lib
#include "zed_lib/sensorcapture.hpp"
#include "zed_lib/videocapture.hpp"

// ROS lib
#include <ros/ros.h>
#include <image_transport/image_transport.h>

class ZedCameraNode
{
public:
  ZedCameraNode(
    const std::shared_ptr<ros::NodeHandle> & nh,
    const std::shared_ptr<image_transport::ImageTransport> & it);
  void run();

private:
  void CameraInit();
  void SensorInit();
  void PublishImages();
  void PublishIMU();

  std::string node_name_;
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  ros::Publisher imu_pub_;
  image_transport::Publisher left_image_pub_;
  image_transport::Publisher right_image_pub_;
  std::unique_ptr<sl_oc::video::VideoCapture> cap_;
  std::unique_ptr<sl_oc::sensors::SensorCapture> sens_;
};
