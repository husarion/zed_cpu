cmake_minimum_required(VERSION 3.1)
project(zed_cpu)

add_compile_options(-std=c++14)

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

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)
find_package(HIDAPI REQUIRED)
find_package(LIBUSB 1 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${HIDAPI_INCLUDE_DIRS}
  ${LibUSB_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include/zed_cpu
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
  src/main.cpp src/zed_cpu.cpp
)

target_link_libraries(zed_node
  zed_lib
  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/include/zed_cpu
)