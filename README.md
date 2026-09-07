## Acknowledgement
This project is a migration from ROS1 to ROS2. The original code was developed by [CircusMonkey](https://github.com/CircusMonkey/ros_rtsp/tree/master). I would like to express my gratitude for his contribution.

## image2rtsp
This project enables the conversion of a selected ROS2 topic of type `sensor_msgs::msg::Image` or `sensor_msgs::msg::CompressedImage` into an `RTSP` stream, with an anticipated delay of approximately 0,3-0,4s. It also supports usb camera as a direct source. Several clients can watch the stream at the same time; they share one pipeline and one encoder. The generated stream can be utilized for various purposes such as remote control, object detection tasks, monitoring and more.

Currently supported and tested `sensor_msgs::msg::Image` formats: "**rgb8**", "**rgba8**", "**rgb16**", "**rgba16**", "**bgr8**", "**bgra8**", "**bgr16**", "**bgra16**", "**mono8**", "**mono16**", "**8UC1**", "**8UC3**", "**8UC4**", "**16UC1**", "**yuv422**", "**yuv422_yuy2**", "**nv21**", "**nv24**". "8UC3"/"8UC4" are assumed to be in OpenCV's BGR order; "rgb16"/"bgr16" are reduced to 8 bit per channel before encoding. 

Supported and tested `sensor_msgs::msg::CompressedImage` formats: "**rgb8; jpeg compressed bgr8**". Other formats may work as well with some color scheme deviations. Please open an issue in this case and attach your **ros2 bag**, so i can fix it. If you need some specific unsupported format, create an issue and i will try to add it as soon as possible, but normally it takes pretty long, so dont hesitate to create a PR.

The development is being carried out on Ubuntu 22.04 with ROS2 Humble. Tested with Intel RealSense d435i.

> **Note:** the `ros2_component` branch has been removed. It contained an
> experimental variant of this package written as a composable ROS2 component.
> It had fallen far behind `master` (missing compressed image support and the
> GStreamer pipeline refactor), and composition brought little practical benefit
> here: receiving and deserializing images accounts for only a few percent of
> this node's CPU usage, while H.264 encoding dominates the rest.

## Dependencies
- ROS2 Humble

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
  - Adjust  `parameters.yaml` according to your needs:
      ```bashrc
      gedit ~/ros2_ws/src/image2rtsp/config/parameters.yaml
      ```
# Example ROS2 Image topic stream
    # If the source is a ros2 topic (default case)
      compressed:       False
      topic:            "/color/image_raw"
      default_pipeline: |
                        ( appsrc name=imagesrc do-timestamp=true min-latency=0 
                          max-latency=0 max-bytes=1000 is-live=true !
                          videoconvert !
                          videoscale !
                          video/x-raw, framerate=30/1, width=640, height=480 !
                          x264enc tune=zerolatency bitrate=500 key-int-max=30 !
                          video/x-h264, profile=baseline !
                          rtph264pay name=pay0 pt=96 )

      # Notice: Set framerate to the real rate of the ROS2 topic. The stream runs at the topic rate regardless,
      # but x264enc's rate control uses this value: a framerate below the real rate roughly doubles the CPU
      # usage and overshoots the configured bitrate.


      # If camera serves as a source
      camera:           False      
      camera_pipeline:  |
                        ( v4l2src device=/dev/video0 !
                          videoconvert !
                          videoscale !
                          video/x-raw, framerate=30/1, width=640, height=480 !
                          x264enc tune=zerolatency bitrate=500 key-int-max=30 !
                          video/x-h264, profile=baseline !
                          rtph264pay name=pay0 pt=96 )

      # Notice: Here the framerate might be set to the camera framerate, otherwise "503 Service Unavailable" error will appear.
      # With camera: True the device is checked once at startup. If it is missing or already in use by another
      # process, the node reports the reason and exits instead of starting a server that can only answer 503.
      # A pipeline that fails later, for example because of the framerate above, is reported on the ROS log.

      # RTSP setup
      mountpoint:       "/back"
      port:             "8554"
      local_only:       True     # True = rtsp://127.0.0.1:portAndMountpoint (The stream is accessible only from the local machine)
                                 # False = rtsp://0.0.0.0:portAndMountpoint (The stream is accessible from the outside) 
                                 # For example, to access the stream running on the machine with IP = 192.168.20.20,
                                 # use rtsp://192.168.20.20:portAndMountpoint

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
      `ros2 run image2rtsp image2rtsp` works too, but only the launch file loads `parameters.yaml`; with `ros2 run`, pass `--ros-args --params-file <your parameters.yaml>` yourself.
    
## Check the stream
To check the stream, follow the instructions for gstreamer, mpv or VLC provided by [CircusMonkey](https://github.com/CircusMonkey/ros_rtsp/blob/master/README.md) or use the viewer script provided in this package (it needs the OpenCV Python bindings: `sudo apt install python3-opencv`). Open a new terminal, ensure that the topic to be converted exists and the RTSP stream is running. Then:
```bash
cd ~/ros2_ws/
source install/setup.bash
ros2 launch image2rtsp rtsp.launch.py url:=rtsp://127.0.0.1:8554/back
```
Replace the URL with your server's IP address, port and mount point (`rtsp://YOUR_IP:PORT/MOUNT_POINT`). Press `q` in the video window to quit.
## Contributing
Please open pull requests against the `dev` branch, from a topic branch on your fork. `master` is the release branch and only receives merges from `dev`.

## Note

- The YAML configuration allows you to fully customize the pipeline according to your needs (Useful insights can be found, for example, [here](https://github.com/maladzenkau/image2rtsp/pull/9)). This package does not provide any built-in acceleration. As its stability has not been validated across a wide range of Linux systems using advanced hardware or software techniques, support for such configurations is left to the user. There are no plans to update the package to support GPU/CPU acceleration. Please do not open issues related to software/hardware acceleration if they are directly related to the GStreamer pipeline itself. 
