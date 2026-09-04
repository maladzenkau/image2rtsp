cmake_minimum_required(VERSION 3.8)
project(image2rtsp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgcodecs)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GST REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-rtsp-server-1.0
)

add_executable(image2rtsp
  src/image2rtsp.cpp
  src/video.cpp
)
target_include_directories(image2rtsp PRIVATE include)
ament_target_dependencies(image2rtsp rclcpp sensor_msgs)
target_link_libraries(image2rtsp PkgConfig::GST ${OpenCV_LIBS})

install(TARGETS image2rtsp
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/rtsp.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
