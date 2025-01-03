cmake_minimum_required(VERSION 3.5)
project(image2rtsp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Suppress all warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
pkg_check_modules(GST REQUIRED 
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-rtsp-server-1.0
)
include_directories(${OpenCV_INCLUDE_DIRS})

file(GLOB SOURCES src/video.cpp)

add_executable(image2rtsp src/image2rtsp.cpp ${SOURCES})
ament_target_dependencies(image2rtsp rclcpp sensor_msgs)

include_directories(
  ${GST_INCLUDE_DIRS}
  )
target_link_libraries(image2rtsp 
  ${GST_LIBRARIES}
  ${OpenCV_LIBS}
  )

install(TARGETS
  image2rtsp
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY 
  config 
  launch
  DESTINATION share/${PROJECT_NAME}
  )

# Install Python modules
ament_python_install_package(python)

# Install Python executables
install(PROGRAMS
  python/rtsp.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()
