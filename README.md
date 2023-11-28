## Acknowledgement
This project is a migration from ROS1 to ROS2. The original code was developed by [CircusMonkey](https://github.com/CircusMonkey/ros_rtsp/tree/master). I would like to express my gratitude for his contribution.

## image2rtsp
The migration is still in process, thus the full functionality of the original package is not presented yet. The current version supports only the conversion of one chosen ROS2 topic into an RTSP stream with an expected delay of about 30-50ms. 
It is being developed on Ubuntu 20.04 with ROS2 Foxy.

## Dependencies
- ROS2 foxy

- gstreamer libs:
```bash
sudo apt-get install libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev libgstrtspserver-1.0-dev gstreamer1.0-plugins-ugly gstreamer1.0-plugins-bad
```
## Install
  - Clone the package:
      ```bashrc
      git clone https://github.com/45kmh/ros2_rtsp.git
      ```
  - Adjust  `parameters.yaml` according to your needs:
      ```bashrc
      gedit ~/image2rtsp/src/image2rtsp/config/parameters.yaml
      ```
# Example ROS2 Image topic stream
    topic: "/color/image_raw"  # The ROS2 topic to subscribe to
    mountpoint: "/back"        # Choose the mountpoint for the rtsp stream. 
                               # This will be able to be accessed from rtsp://<server_ip>/portAndMountpoint
    caps: "video/x-raw,framerate=10/1,width=640,height=480"
                               # Set the caps to be applied after getting the ROS2 Image and before the x265 encoder.
    bitrate: "500"
    port: "8554"
    local_only: True           # True = rtsp://127.0.0.1:port (The stream is accessible only from the local machine)
                               # False = rtsp://0.0.0.0:portAndMountpoint (The stream is accessible from the outside) 
                               # For example, to access the stream running on the machine with IP = 192.168.20.20,
                               # use rtsp://192.186.20.20:portAndMountpoint
                               # True = rtsp://127.0.0.1:portAndMountpoint (The stream is accessible only from the local machine)
  - Save your configuration and navigate to `image2rtsp ` colcon root, source and build the package:
      ```bashrc
      cd ~/image2rtsp/
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
To check the stream, follow the instructions for gstreamer, mpv or VLC provided by [CircusMonkey](https://github.com/CircusMonkey/ros_rtsp/blob/master/README.md) or use python script (ensure before that the open-cv library is installed, if not `pip install opencv-python`):
```bash
gedit ~/image2rtsp/src/image2rtsp/python/rtsp.py
```
Replace the `rtsp://127.0.0.1:8554/back` with your server's IP address, port and mount point `rtsp://YOUR_IP:PORT/MOUNT_POINT`. Save and run:
```bash
python3 ~/image2rtsp/src/image2rtsp/python/rtsp.py
```
## Limitations
- As was previously mentioned, this package allows the conversion of only one topic into an RTSP Stream.
- Some machines may have a significantly bigger delay of about 150-200ms. The reason is currently unknown.
## To Do
- Add a branch with the package rewritten as a ROS2 component
- Port the package to Nvidia Jetson Orin. Will the use of the Nvidia encoder make the delay smaller?
- Port packages to ROS2 Humble.
- Complete the functionality according to the functionality of the original ROS package.


