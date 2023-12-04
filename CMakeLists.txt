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

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rcutils REQUIRED)
pkg_check_modules(GST REQUIRED 
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-rtsp-server-1.0
)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

include_directories(
  include
  ${GST_INCLUDE_DIRS}
  )

# create ament index resource which references the libraries in the binary dir
set(node_plugins "")

add_library(image2rtsp SHARED
  src/image2rtsp.cpp src/video.cpp)

ament_target_dependencies(image2rtsp rclcpp sensor_msgs)
target_compile_definitions(image2rtsp
  PRIVATE "IMAGE2RTSP_BUILDING_DLL")

target_link_libraries(image2rtsp
  ${sensor_msgs_TARGETS}
  ${GST_LIBRARIES})

ament_target_dependencies(image2rtsp
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs")
rclcpp_components_register_nodes(image2rtsp "Image2rtsp")
set(node_plugins "${node_plugins}Image2rtsp;$<TARGET_FILE:image2rtsp>\n")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


install(TARGETS
  image2rtsp
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
install(DIRECTORY
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
