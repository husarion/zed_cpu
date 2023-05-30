#include <zed_cpu.hpp>

ZedCameraNode::ZedCameraNode(
  const std::shared_ptr<ros::NodeHandle> & nh,
  const std::shared_ptr<image_transport::ImageTransport> & it)
: nh_(nh), it_(it)
{
  // ROS initialization
  leftImagePub_ = it_->advertise("rgb/left_image", 1);
  rightImagePub_ = it_->advertise("rgb/right_image", 1);
  imuPub_ = nh_->advertise<sensor_msgs::Imu>("imu_data", 1);

  cameraInit();
  sensorInit();
}

void ZedCameraNode::run()
{
  publishImages();
  publishIMU();
}

void ZedCameraNode::cameraInit()
{
  // Initialize ZED camera
  sl_oc::video::VideoParams params;
  params.res = sl_oc::video::RESOLUTION::HD720;
  params.fps = sl_oc::video::FPS::FPS_60;
  params.verbose = sl_oc::VERBOSITY::ERROR;

  // Create Video Capture
  cap_ = std::make_unique<sl_oc::video::VideoCapture>(params);
  if (!cap_->initializeVideo()) {
    ROS_ERROR("Cannot open camera video capture");
    return;
  }

  ROS_INFO_STREAM(
    "Connected to camera sn: " << cap_->getSerialNumber() << " [" << cap_->getDeviceName() << "]");
}

void ZedCameraNode::sensorInit()
{
  sens_ = std::make_unique<sl_oc::sensors::SensorCapture>(sl_oc::VERBOSITY::ERROR);

  std::vector<int> devs = sens_->getDeviceList();

  if (devs.size() == 0) {
    ROS_ERROR("No available ZED 2, ZED 2i or ZED Mini cameras");
    return;
  }

  uint16_t fw_maior;
  uint16_t fw_minor;
  sens_->getFirmwareVersion(fw_maior, fw_minor);
  ROS_INFO_STREAM(
    "Connected to IMU firmware version: " << std::to_string(fw_maior) << "."
                                          << std::to_string(fw_minor));

  is_sensInit_ = sens_->initializeSensors(devs[0]);
}

void ZedCameraNode::publishImages()
{
  // Get last available frame
  const sl_oc::video::Frame frame = cap_->getLastFrame();

  // Process and publish the frame
  if (frame.data != nullptr) {
    cv::Mat frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
    cv::Mat frameBGR;
    cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);

    // Split the frame into left and right images
    cv::Mat leftImage = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
    cv::Mat rightImage = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));

    // Convert the OpenCV images to ROS image messages
    sensor_msgs::ImagePtr leftMsg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", leftImage).toImageMsg();
    sensor_msgs::ImagePtr rightMsg =
      cv_bridge::CvImage(std_msgs::Header(), "bgr8", rightImage).toImageMsg();

    // Publish the left and right image messages
    leftImagePub_.publish(leftMsg);
    rightImagePub_.publish(rightMsg);
  }
}

void ZedCameraNode::publishIMU()
{
  // Initialize the sensors
  if (!is_sensInit_) {
    ROS_ERROR("Connection failed");
    return;
  }

  // Get IMU data with a timeout of 5 milliseconds
  const sl_oc::sensors::data::Imu imuData = sens_->getLastIMUData(5000);

  if (imuData.valid == sl_oc::sensors::data::Imu::NEW_VAL) {
    // Create a sensor_msgs/Imu message
    sensor_msgs::Imu imuMsg;
    imuMsg.header.stamp = ros::Time::now();
    imuMsg.header.frame_id = "imu_frame";

    // Convert the IMU data to the sensor_msgs/Imu message fields
    imuMsg.linear_acceleration.x = -imuData.aX;
    imuMsg.linear_acceleration.y = imuData.aY;
    imuMsg.linear_acceleration.z = imuData.aZ;

    imuMsg.angular_velocity.x = -imuData.gX;
    imuMsg.angular_velocity.y = imuData.gY;
    imuMsg.angular_velocity.z = imuData.gZ;

    // Publish the sensor_msgs/Imu message
    imuPub_.publish(imuMsg);
  }
}
