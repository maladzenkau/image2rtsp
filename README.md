## Acknowledgement
This project is a migration from ROS1 to ROS2. The original code was developed by [CircusMonkey](https://github.com/CircusMonkey/ros_rtsp/tree/master). I would like to express my gratitude for his contribution.

## image2rtsp
This project enables the conversion of a selected ROS2 topic of type `sensor_msgs::msg::Image` into an `RTSP` stream, with an anticipated delay of approximately 30-50ms. The generated stream can be utilized for various purposes such as remote control, object detection tasks, monitoring, and more. Please note that the migration process is ongoing, and therefore, the complete functionality of the original package is not yet available.

The development is being carried out on Ubuntu 20.04 with ROS2 Foxy, also tested on Ubuntu 22.04 with ROS2 Humble. 

You are reading now the README for a package written as a ROS2 **component**. To know about ROS2 components you can look into the official documentation ([link1](https://docs.ros.org/en/foxy/Concepts/About-Composition.html), [link2](https://docs.ros.org/en/foxy/Tutorials/Intermediate/Composition.html)). If you want to use this package as a default ROS2 package, checkout `master` branch. 

## Dependencies
- ROS2 Foxy/Humble

- gstreamer libs:
```bash
sudo apt-get install libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev libgstrtspserver-1.0-dev gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad
```
## Install
  - Navigate to the root directory, create a new directory named `ros2_ws/src`, and then change the current working directory to `ros2_ws/src`:
      ```bashrc
      cd
      mkdir -p ros2_ws/src
      cd ros2_ws/src/
      ```
  - Clone the package and navigate into the directory `image2rtsp`:
      ```bashrc
      git clone https://github.com/maladzenkau/image2rtsp.git -b ros2_component
      ```
  - Check the framerate of the topic to be subscribed:
      ```bashrc
      ros2 topic hz /someTopic
    ```  
  - All the parameters are hardcoded in `image2rtsp.hpp`, so be sure to change them before adding the `Image2rtsp` component into `ComponentManager`:
      ```bashrc
      gedit ~/ros2_ws/src/image2rtsp/include/image2rtsp.hpp
      ```
# Example ROS2 Image topic stream
    topic: "/color/image_raw"  # The ROS2 topic to subscribe to
    mountpoint: "rs"           # Choose the mountpoint for the rtsp stream. 
                               # This will be able to be accessed from rtsp://<server_ip>/portAndMountpoint
    bitrate: "500"
    framerate = "30";
    caps: "video/x-raw,framerate=" + framerate + "/1,width=1280,height=720";
                               # Set the caps to be applied after getting the ROS2 Image and before the x265 encoder.
    port: "8554"
    local_only: True           # True = rtsp://127.0.0.1:port (The stream is accessible only from the local machine)
                               # False = rtsp://0.0.0.0:portAndMountpoint (The stream is accessible from the outside) 
                               # For example, to access the stream running on the machine with IP = 192.168.20.20,
                               # use rtsp://192.186.20.20:portAndMountpoint
  - Save your configuration and navigate to `ros2_ws` colcon root, source and build the package:
      ```bashrc
      cd ~/ros2_ws/
      colcon build --packages-select image2rtsp
      ```
## Run
  - Open another shell, source local setup and run `ComponentManager`:
      ```bashrc
      cd ~/ros2_ws/
      source install/setup.bash
      ros2 run rclcpp_components component_container
      ```
  - Add a component with a publisher node first. 
  - Source `install` and add the `Image2rtsp` component:
      ```bashrc
      source install/setup.bash
      ros2 component load /ComponentManager image2rtsp Image2rtsp
      ```
      OR to enable intra-process communication:
      ```bashrc
      ros2 component load /ComponentManager image2rtsp Image2rtsp -e use_intra_process_comms:=true
      ```    
## Check the stream
To check the stream, follow the instructions for gstreamer, mpv or VLC provided by [CircusMonkey](https://github.com/CircusMonkey/ros_rtsp/blob/master/README.md) or use python script provided in this package (ensure before that the open-cv library is installed, if not `pip install opencv-python`). Ensure that components with publisher and subscriber nodes are loaded in the `ComponentManager`. Then:
```bash
gedit ~/ros2_ws/src/image2rtsp/python/rtsp.py
```
Replace the `rtsp://0.0.0.0:8554/rs` with your server's IP address, port and mount point `rtsp://YOUR_IP:PORT/MOUNT_POINT`. Save and run:
```bash
cd ~/ros2_ws/
source install/setup.bash
ros2 launch image2rtsp rtsp.launch.py 
```
