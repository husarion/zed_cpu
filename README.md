<h1 align="center">
  ZED Camera ROS wrapper on CPU
</h1>


## Description
ROS wrapper for Stereolabs cameras (ZED 2, ZED 2i, and ZED Mini) dedicatet for CPU. Based on Stereolabs [zed-open-capture](https://github.com/stereolabs/zed-open-capture) repository. Repository delivers only video capture and IMU data. For more advanced data, please refer to the original repository from Stereolabs [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper)

## Build

### Prerequisites

 * Stereo camera: [ZED 2i](https://www.stereolabs.com/zed-2i/), [ZED 2](https://www.stereolabs.com/zed-2/), [ZED Mini](https://www.stereolabs.com/zed-mini/)
 * Linux OS
 * ROS Humble
 * GCC (v7.5+)
 * OpenCV (v3.4.0+)

### Clone the repository

    mkdir -p ros_ws/src
    cd ros_ws
    git clone https://github.com/husarion/zed-cpu.git src/zed-cpu

### Install prerequisites

* Install HIDAPI and LIBUSB libraries:

    `sudo apt install libusb-1.0-0-dev libhidapi-libusb0 libhidapi-dev`

* Install OpenCV:

    `sudo apt install libopencv-dev libopencv-viz-dev`

* Install ROS dependencies:

    `rosdep install --from-paths src --ignore-src -r -y`


### Add udev rule
Stereo cameras such as ZED 2 and ZED Mini have built-in sensors (e.g. IMU) that are identified as USB HID devices.
To be able to access the USB HID device, you must add a udev rule contained in the `udev` folder:

    ```bash
    cd src/zed-cpu/udev
    bash install_udev_rule.sh
    ```

### Build code

    ```bash
    cd ~/ros_ws
    colcon build
    ```
### Run code

    ```bash
    source ~/ros_ws/devel/setup.bash
    ros2 launch zed_cpu zed.launch
    ```

## Coordinates system

The given IMU and Magnetometer data are expressed in the RAW coordinate system as show below

<div align="center">

![](./images/imu_axis.jpg)

</div>