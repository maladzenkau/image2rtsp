#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>
#include "../include/image2rtsp.hpp"

using std::placeholders::_1;

Image2rtsp::Image2rtsp() : Node("image2rtsp"){
    // Declare and get the parameters
    this->declare_parameter("topic",        "/color/image_raw");
    this->declare_parameter("mountpoint",   "/back");
    this->declare_parameter("port",         "8554");
    this->declare_parameter("local_only",   true);
    this->declare_parameter("camera",       false);
    this->declare_parameter("compressed",   false);

    this->declare_parameter("default_pipeline",   R"(
                                                    ( appsrc name=imagesrc do-timestamp=true min-latency=0 max-latency=0 max-bytes=1000 is-live=true !
                                                    videoconvert !
                                                    videoscale !
                                                    video/x-raw, framerate=30/1, width=640, height=480 !
                                                    x264enc tune=zerolatency bitrate=500 key-int-max=30 !
                                                    video/x-h264, profile=baseline !
                                                    rtph264pay name=pay0 pt=96 )
                                                    )");

    this->declare_parameter("camera_pipeline",    R"(
                                                    ( v4l2src device=/dev/video0 !
                                                    videoconvert !
                                                    videoscale !
                                                    video/x-raw, framerate=30/1, width=640, height=480 !
                                                    x264enc tune=zerolatency bitrate=500 key-int-max=30 !
                                                    video/x-h264, profile=baseline !
                                                    rtph264pay name=pay0 pt=96 )
                                                    )");

    topic               = this->get_parameter("topic").as_string();
    mountpoint          = this->get_parameter("mountpoint").as_string();
    port                = this->get_parameter("port").as_string();
    local_only          = this->get_parameter("local_only").as_bool();
    camera              = this->get_parameter("camera").as_bool();
    compressed          = this->get_parameter("compressed").as_bool();
    default_pipeline    = this->get_parameter("default_pipeline").as_string();
    camera_pipeline     = this->get_parameter("camera_pipeline").as_string();

    // Start the subscription
    if (camera == false){
        if (compressed == false){
            subscription_ = this->create_subscription<sensor_msgs::msg::Image>(topic, 10, std::bind(&Image2rtsp::topic_callback, this, _1));
            RCLCPP_INFO(this->get_logger(), "Subscribing to sensor_msgs::msg::Image");
        }
        else {
            subscription_compressed_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(topic, 10, std::bind(&Image2rtsp::compressed_topic_callback, this, _1));
            RCLCPP_INFO(this->get_logger(), "Subscribing to sensor_msgs::msg::CompressedImage");
        }
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Trying to access camera device");
    }

    // Start the RTSP server
    video_mainloop_start();
    rtsp_server = rtsp_server_create(port, local_only);

    pipeline = camera ? camera_pipeline : default_pipeline;
    framerate = extract_framerate(pipeline, 30);
    rtsp_server_add_url(mountpoint.c_str(), pipeline.c_str());

    const char *server_address = gst_rtsp_server_get_address(rtsp_server);
    if (local_only) {
        RCLCPP_INFO(this->get_logger(), "Stream available at rtsp://%s:%s%s", server_address, port.c_str(), mountpoint.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "RTSP server bound to %s:%s%s", server_address, port.c_str(), mountpoint.c_str());
        RCLCPP_INFO(this->get_logger(), "Connect clients using rtsp://<host-ip>:%s%s (0.0.0.0 is bind-only)", port.c_str(), mountpoint.c_str());
    }
}

uint Image2rtsp::extract_framerate(const std::string& pipeline, uint default_framerate = 30) {
    std::string search_str = "framerate=";
    size_t pos = pipeline.find(search_str);
    if (pos == std::string::npos) {
        RCLCPP_WARN(this->get_logger(), "Framerate not found in pipeline, using default: %d", default_framerate);
        return default_framerate;
    }

    pos += search_str.length();

    size_t end_pos = pipeline.find_first_of("/,", pos);
    if (end_pos == std::string::npos) {
        RCLCPP_WARN(this->get_logger(), "Invalid framerate format in pipeline, using default: %d", default_framerate);
        return default_framerate;
    }

    std::string framerate_str = pipeline.substr(pos, end_pos - pos);

    framerate_str.erase(0, framerate_str.find_first_not_of(" \t"));
    framerate_str.erase(framerate_str.find_last_not_of(" \t") + 1);
    
    try {
        uint framerate = std::stoi(framerate_str);
        if (framerate <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid framerate value %d, using default: %d", framerate, default_framerate);
            return default_framerate;
        }
        RCLCPP_INFO(this->get_logger(), "Using set framerate %d", framerate);
        return framerate;
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse framerate '%s', using default: %d", framerate_str.c_str(), default_framerate);
        return default_framerate;
    }
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    int rc = 0;
    try {
        auto node = std::make_shared<Image2rtsp>();
        rclcpp::spin(node);
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("image2rtsp"), "%s", e.what());
        rc = 1;
    }
    rclcpp::shutdown();
    return rc;
}
