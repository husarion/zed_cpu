// ZED lib
#include "videocapture.hpp"

// ROS lib
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

// OpenCV lib
#include <opencv2/opencv.hpp>

class ZedCameraNode
{
public:
    ZedCameraNode() : nh_("~")
    {
        // ROS initialization
        image_transport::ImageTransport it(nh_);
        leftImagePub_ = it.advertise("zed_camera/left_image", 1);  // Publish left camera image
        rightImagePub_ = it.advertise("zed_camera/right_image", 1);  // Publish right camera image

        // Initialize ZED camera
        sl_oc::video::VideoParams params;
        params.res = sl_oc::video::RESOLUTION::HD720;
        params.fps = sl_oc::video::FPS::FPS_30;

        // Create Video Capture
        cap_ = std::make_unique<sl_oc::video::VideoCapture>(params);
        if (!cap_->initializeVideo())
        {
            ROS_ERROR("Cannot open camera video capture");
            ROS_ERROR("See verbosity level for more details.");
            return;
        }

        ROS_INFO_STREAM("Connected to camera sn: " << cap_->getSerialNumber() << "[" << cap_->getDeviceName() << "]");
    }

    void run()
    {
        // Get last available frame
        const sl_oc::video::Frame frame = cap_->getLastFrame();

        // Process and publish the frame
        if (frame.data != nullptr)
        {
            cv::Mat frameYUV = cv::Mat(frame.height, frame.width, CV_8UC2, frame.data);
            cv::Mat frameBGR;
            cv::cvtColor(frameYUV, frameBGR, cv::COLOR_YUV2BGR_YUYV);

            // Split the frame into left and right images
            cv::Mat leftImage = frameBGR(cv::Rect(0, 0, frameBGR.cols / 2, frameBGR.rows));
            cv::Mat rightImage = frameBGR(cv::Rect(frameBGR.cols / 2, 0, frameBGR.cols / 2, frameBGR.rows));

            // Convert the OpenCV images to ROS image messages
            sensor_msgs::ImagePtr leftMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", leftImage).toImageMsg();
            sensor_msgs::ImagePtr rightMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", rightImage).toImageMsg();

            // Publish the left and right image messages
            leftImagePub_.publish(leftMsg);
            rightImagePub_.publish(rightMsg);
        }
    }

private:
    ros::NodeHandle nh_;
    image_transport::Publisher leftImagePub_;
    image_transport::Publisher rightImagePub_;
    std::unique_ptr<sl_oc::video::VideoCapture> cap_;
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "zed_camera_node");

    ZedCameraNode zedCameraNode;
    while (ros::ok())
    {
        zedCameraNode.run();
        ros::spinOnce();
    }

    ros::shutdown();

    return EXIT_SUCCESS;
}
