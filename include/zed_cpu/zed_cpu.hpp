// ZED lib
#include "zed_lib/sensorcapture.hpp"
#include "zed_lib/videocapture.hpp"

// ROS lib
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Imu.h>

// OpenCV lib
#include <opencv2/opencv.hpp>

class ZedCameraNode
{
public:
  ZedCameraNode(
    const std::shared_ptr<ros::NodeHandle> & nh,
    const std::shared_ptr<image_transport::ImageTransport> & it);
  void run();

private:
  void cameraInit();
  void sensorInit();
  void publishImages();
  void publishIMU();

  bool is_sensInit_;
  std::shared_ptr<ros::NodeHandle> nh_;
  std::shared_ptr<image_transport::ImageTransport> it_;
  ros::Publisher imuPub_;
  image_transport::Publisher leftImagePub_;
  image_transport::Publisher rightImagePub_;
  std::unique_ptr<sl_oc::video::VideoCapture> cap_;
  std::unique_ptr<sl_oc::sensors::SensorCapture> sens_;
};
