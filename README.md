## Acknowledgement
This project is a migration from ROS1 to ROS2. The original code was developed by [CircusMonkey](https://github.com/CircusMonkey/ros_rtsp/tree/master). I would like to express my gratitude for his contribution.

## image2rtsp
This project enables the conversion of a selected ROS2 topic of type `sensor_msgs::msg::Image` into an `RTSP` stream, with an anticipated delay of approximately 0,3-0,4s. It also supports usb camera as a direct source. The generated stream can be utilized for various purposes such as remote control, object detection tasks, monitoring, and more. Please note that the migration process is ongoing, and therefore, the complete functionality of the original package is not yet available.

Currently supported `sensor_msgs::msg::Image` formats: "rgb8", "rgba8", "rgb16", "rgba16", "bgr8", "bgra8", "bgr16", "bgra16", "mono8", "mono16", "yuv422_yuy2".

The development is being carried out on Ubuntu 20.04 with ROS2 Foxy, also tested on Ubuntu 22.04 with ROS2 Humble.

You are reading now the README for a **default** ROS2 package. If you want to use this package written as a ROS2 component, checkout `ros2_component` branch. 
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
  - Clone the package and then navigate into the directory `image2rtsp`:
      ```bashrc
      git clone https://github.com/maladzenkau/image2rtsp.git --single-branch
      ```
  - Check the framerate of the topic to be subscribed:
      ```bashrc
      ros2 topic hz /someTopic
      ```  
  - Adjust  `parameters.yaml` according to your needs:
      ```bashrc
      gedit ~/ros2_ws/src/image2rtsp/config/parameters.yaml
      ```
# Example ROS2 Image topic stream

    # If camera serves as a source (switched off by default)
    camera: False      
    source: "v4l2src device=/dev/video0"
    
    topic: "/color/image_raw"  # The ROS2 topic to subscribe to. Dont change, if you use a camera

    # Parameters for both cases
    mountpoint: "/back"        # Choose the mountpoint for the rtsp stream. 
                               # This will be able to be accessed from rtsp://<server_ip>/portAndMountpoint
    bitrate: "500"
    framerate: "30"            # Make sure that your framerate corresponds to the frequency of a topic you are subscribing to
    caps_1: "video/x-raw,framerate="
    capr_2: "/1,width=640,height=480"
                               # Set the caps to be applied after getting the ROS2 Image and before the x265 encoder. Ignore
                               # framerate setting here.
    port: "8554"
    local_only: True           # True = rtsp://127.0.0.1:portAndMountpoint (The stream is accessible only from the local machine)
                               # False = rtsp://0.0.0.0:portAndMountpoint (The stream is accessible from the outside) 
                               # For example, to access the stream running on the machine with IP = 192.168.20.20,
                               # use rtsp://192.186.20.20:portAndMountpoint
  - Save your configuration and navigate to `ros2_ws` colcon root, source and build the package:
      ```bashrc
      cd ~/ros2_ws/
      colcon build --packages-select image2rtsp
      ```
## Run
  - Source `install` and launch the package:
      ```bashrc
      source install/setup.bash
      ros2 launch image2rtsp image2rtsp.launch.py 
      ```
      Don't use **`ros2 run`**!
    
## Check the stream
To check the stream, follow the instructions for gstreamer, mpv or VLC provided by [CircusMonkey](https://github.com/CircusMonkey/ros_rtsp/blob/master/README.md) or use python script provided in this package (ensure before that the open-cv library is installed, if not `pip install opencv-python`). Open new terminal, ensure that the topic to be converted exists and the RTSP stream is running. Then:
```bash
gedit ~/ros2_ws/src/image2rtsp/python/rtsp.py
```
Replace the `rtsp://127.0.0.1:8554/back` with your server's IP address, port and mount point `rtsp://YOUR_IP:PORT/MOUNT_POINT`. Save and run:
```bash
cd ~/ros2_ws/
source install/setup.bash
ros2 launch image2rtsp rtsp.launch.py 
```
## Limitations
- As was previously mentioned, this package allows the conversion of only data source into an RTSP Stream.
## To Do
- Add web camera ros2 topic formats
- Add compressed images formats (jpeg, png)
