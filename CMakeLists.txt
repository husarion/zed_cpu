cmake_minimum_required(VERSION 3.5)
project(zed_cpu)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Raspberry Pi 4 - OS 64bit
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  add_compile_options(-mtune=cortex-a72 -mcpu=cortex-a72)
  add_definitions(-DEMBEDDED_ARM)
endif()

# Raspberry Pi 4 - OS 32bit
if(CMAKE_SYSTEM_PROCESSOR MATCHES "armv7l")
  add_compile_options(-mfloat-abi=hard -mfpu=neon-fp-armv8 -mneon-for-64bits -mtune=cortex-a72)
  add_definitions(-DEMBEDDED_ARM)
endif()

list(INSERT CMAKE_MODULE_PATH 0 "${PROJECT_SOURCE_DIR}/cmake")

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(HIDAPI REQUIRED)
find_package(LIBUSB 1 REQUIRED)

set(dependencies
  cv_bridge
  image_transport
  rclcpp
  sensor_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include/zed_cpu
  LIBRARIES zed_lib
  CATKIN_DEPENDS ${dependencies}
)

include_directories(
  include/zed_cpu
  ${HIDAPI_INCLUDE_DIRS}
  ${LibUSB_INCLUDE_DIRS}
)

add_library(zed_lib
  src/zed_lib/videocapture.cpp
  src/zed_lib/sensorcapture.cpp
)

target_link_libraries(zed_lib
  ${OpenCV_LIBS}
  ${LibUSB_LIBRARIES}
  ${HIDAPI_LIBRARIES}
)

add_executable(zed_node
  src/main.cpp
  src/zed_cpu.cpp
)

target_link_libraries(zed_node
  zed_lib
)

ament_target_dependencies(zed_node ${dependencies})

install(
  TARGETS zed_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()
