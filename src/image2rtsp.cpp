#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>
#include "../include/image2rtsp.hpp"

using std::placeholders::_1;

Image2rtsp::Image2rtsp() : Node("image2rtsp"){
    // Declare and get the parameters
    this->declare_parameter("source", "v4l2src device=/dev/video0");
    this->declare_parameter("topic", "/color/image_raw");
    this->declare_parameter("mountpoint", "/back");
    this->declare_parameter("bitrate", "500");
    this->declare_parameter("framerate", "30");
    this->declare_parameter("caps_1", "video/x-raw, framerate =");
    this->declare_parameter("caps_2", "/1,width=640,height=480");
    this->declare_parameter("port", "8554");
    this->declare_parameter("local_only", true);
    this->declare_parameter("camera", false);
    this->declare_parameter("compressed", false);

    source = this->get_parameter("source").as_string();
    topic = this->get_parameter("topic").as_string();
    mountpoint = this->get_parameter("mountpoint").as_string();
    bitrate = this->get_parameter("bitrate").as_string();
    framerate = this->get_parameter("framerate").as_string();
    caps_1 = this->get_parameter("caps_1").as_string();
    caps_2 = this->get_parameter("caps_2").as_string();
    port = this->get_parameter("port").as_string();
    local_only = this->get_parameter("local_only").as_bool();
    camera = this->get_parameter("camera").as_bool();
    compressed = this->get_parameter("compressed").as_bool();

    // Start the subscription
    if (compressed == false){
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(topic, 10, std::bind(&Image2rtsp::topic_callback, this, _1));
    }
    else{
        subscription_compressed_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(topic, 10, std::bind(&Image2rtsp::compressed_topic_callback, this, _1));
    }

    // Start the RTSP server
    video_mainloop_start();
    rtsp_server = rtsp_server_create(port, local_only);
    appsrc = NULL;
    // Setup the pipeline
    pipeline_tail = "key-int-max=30 ! video/x-h264, profile=baseline ! rtph264pay name=pay0 pt=96 )";
    if (camera == false){
        pipeline_head = "( appsrc name=imagesrc do-timestamp=true min-latency=0 max-latency=0 max-bytes=1000 is-live=true ! videoconvert ! videoscale ! ";
        pipeline = pipeline_head + caps_1 + framerate + caps_2 + " ! x264enc tune=zerolatency bitrate=" + bitrate + pipeline_tail;
        rtsp_server_add_url(mountpoint.c_str(), pipeline.c_str(), (GstElement **)&(appsrc));
    }
    else {
        pipeline = "( " + source + " ! videoconvert ! videoscale ! " + caps_1 + framerate + caps_2 + " ! x264enc tune=zerolatency bitrate=" + bitrate + pipeline_tail;
        rtsp_server_add_url(mountpoint.c_str(), pipeline.c_str(), NULL);
    }
    RCLCPP_INFO(this->get_logger(), "Stream available at rtsp://%s:%s%s", gst_rtsp_server_get_address(rtsp_server), port.c_str(), mountpoint.c_str());
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Image2rtsp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
