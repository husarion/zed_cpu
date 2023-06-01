#include <zed_cpu.hpp>

#include <memory>
#include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>

#include <zed_lib/sensorcapture.hpp>
#include <zed_lib/videocapture.hpp>

namespace zed_cpu
{

ZedCameraNode::ZedCameraNode() : Node("zed_camera", "zed_camera")
{
  // ROS initialization
  left_image_pub_ = image_transport::create_publisher(this, "rgb/left_image");
  right_image_pub_ = image_transport::create_publisher(this, "rgb/right_image");

  imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu_data", 1);

  CameraInit();
  SensorInit();

  RCLCPP_INFO(get_logger(), "Node started");
}

void ZedCameraNode::run()
{
  PublishImages();
  PublishIMU();
}

void ZedCameraNode::CameraInit()
{
  // Initialize ZED camera
  sl_oc::video::VideoParams params;
  params.res = sl_oc::video::RESOLUTION::HD720;
  params.fps = sl_oc::video::FPS::FPS_60;
  params.verbose = sl_oc::VERBOSITY::ERROR;

  // Create Video Capture
  cap_ = std::make_unique<sl_oc::video::VideoCapture>(params);
  if (!cap_->initializeVideo()) {
    RCLCPP_ERROR(get_logger(), "Cannot open camera video capture");
    rclcpp::shutdown();
    return;
  }

  RCLCPP_INFO_STREAM(
    get_logger(),
    "Connected to camera sn: " << cap_->getSerialNumber() << " [" << cap_->getDeviceName() << "]");
}

void ZedCameraNode::SensorInit()
{
  sens_ = std::make_unique<sl_oc::sensors::SensorCapture>(sl_oc::VERBOSITY::ERROR);

  std::vector<int> devs = sens_->getDeviceList();

  if (devs.size() == 0) {
    RCLCPP_ERROR(get_logger(), "No available ZED 2, ZED 2i or ZED Mini cameras");
    rclcpp::shutdown();
    return;
  }

  uint16_t fw_maior;
  uint16_t fw_minor;
  sens_->getFirmwareVersion(fw_maior, fw_minor);
  RCLCPP_INFO_STREAM(
    get_logger(), "Connected to IMU firmware version: " << std::to_string(fw_maior) << "."
                                                        << std::to_string(fw_minor));

  // Initialize the sensors
  if (!sens_->initializeSensors(devs[0])) {
    RCLCPP_ERROR(get_logger(), "IMU initialize failed");
    rclcpp::shutdown();
    return;
  }
}

void ZedCameraNode::PublishImages()
{
  // Get last available frame
  const sl_oc::video::Frame frame = cap_->getLastFrame();

  // Process and publish the frame
  if (frame.data != nullptr) {
    cv::Mat frame_yuv = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
    cv::Mat frame_bgr;
    cv::cvtColor(frame_yuv, frame_bgr, cv::COLOR_YUV2BGR_YUYV);

    // Split the frame into left and right images
    cv::Mat left_img = frame_bgr(cv::Rect(0, 0, frame_bgr.cols / 2, frame_bgr.rows));
    cv::Mat right_img =
      frame_bgr(cv::Rect(frame_bgr.cols / 2, 0, frame_bgr.cols / 2, frame_bgr.rows));

    // Convert the OpenCV images to ROS image messages
    sensor_msgs::msg::Image::SharedPtr left_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", left_img).toImageMsg();
    sensor_msgs::msg::Image::SharedPtr right_msg =
      cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", right_img).toImageMsg();

    // Publish the left and right image messages
    left_image_pub_.publish(left_msg);
    right_image_pub_.publish(right_msg);
  }
}

void ZedCameraNode::PublishIMU()
{
  // Get IMU data with a timeout of 5 milliseconds
  const sl_oc::sensors::data::Imu imu_data = sens_->getLastIMUData(5000);

  if (imu_data.valid == sl_oc::sensors::data::Imu::NEW_VAL) {
    // Create a sensor_msgs/Imu message
    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp = get_clock()->now();
    imu_msg.header.frame_id = "imu_frame";

    // Convert the IMU data to the sensor_msgs/Imu message fields
    imu_msg.linear_acceleration.x = -imu_data.aX;
    imu_msg.linear_acceleration.y = imu_data.aY;
    imu_msg.linear_acceleration.z = imu_data.aZ;

    imu_msg.angular_velocity.x = -imu_data.gX;
    imu_msg.angular_velocity.y = imu_data.gY;
    imu_msg.angular_velocity.z = imu_data.gZ;

    // Publish the sensor_msgs/Imu message
    imu_pub_->publish(imu_msg);
  }
}

}  // namespace zed_cpu