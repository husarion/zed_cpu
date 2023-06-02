#include <memory>
#include <vector>

#include <image_transport/image_transport.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <zed_lib/sensorcapture.hpp>
#include <zed_lib/videocapture.hpp>

namespace zed_cpu
{

class ZedCameraNode : public rclcpp::Node
{
public:
  ZedCameraNode();
  void run();

private:
  void CameraInit();
  void SensorInit();
  void PublishImages();
  void PublishIMU();

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  image_transport::Publisher left_image_pub_;
  image_transport::Publisher right_image_pub_;
  std::unique_ptr<sl_oc::video::VideoCapture> cap_;
  std::unique_ptr<sl_oc::sensors::SensorCapture> sens_;
};

} // namespace zed_cpu