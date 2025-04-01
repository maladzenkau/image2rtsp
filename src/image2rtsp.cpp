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
    this->declare_parameter("encoder", "software");

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
    encoder = this->get_parameter("encoder").as_string();

    // Start the subscription
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(topic, 10, std::bind(&Image2rtsp::topic_callback, this, _1));

    // Start the RTSP server
    video_mainloop_start();
    rtsp_server = rtsp_server_create(port, local_only);
    appsrc = NULL;
    // Setup the pipeline
    pipeline_tail = " ! video/x-h264,profile=constrained-baseline ! rtph264pay name=pay0 pt=96 "
    "config-interval=1 aggregate-mode=zero-latency ! "
    "application/x-rtp,media=video )";
    auto encoder = get_encoder_pipeline();
    if (camera == false){
        pipeline_head = "( appsrc name=imagesrc do-timestamp=true min-latency=0 "
        "max-latency=0 max-bytes=2000000 is-live=true ! queue max-size-buffers=1 leaky=downstream ! videoconvert ! videoscale ! ";
        pipeline = pipeline_head + caps_1 + framerate + caps_2 + 
            " ! " + encoder + " bitrate=" + bitrate + pipeline_tail;
        rtsp_server_add_url(mountpoint.c_str(), pipeline.c_str(), (GstElement **)&(appsrc));
        RCLCPP_INFO(this->get_logger(), "pipeline: %s", pipeline.c_str());
    }
    else {
        pipeline = "( " + source + " ! videoconvert ! videoscale ! " + caps_1 + framerate + caps_2 + " ! x264enc tune=zerolatency bitrate=" + bitrate + pipeline_tail;
        rtsp_server_add_url(mountpoint.c_str(), pipeline.c_str(), NULL);
    }
    RCLCPP_INFO(this->get_logger(), "Stream available at rtsp://%s:%s%s", gst_rtsp_server_get_address(rtsp_server), port.c_str(), mountpoint.c_str());
}

std::string Image2rtsp::get_encoder_pipeline() {
    GstElementFactory* factory;

    // Check for NVIDIA encoder
    factory = gst_element_factory_find("nvh264enc");
    if (factory && (encoder == "hardware" || encoder == "nvidia")) {
        gst_object_unref(factory);
        RCLCPP_INFO(this->get_logger(), "Using NVIDIA hardware encoder");
        return "nvh264enc preset=low-latency-hp rc-mode=cbr-ld-hq zerolatency=true";
    }

    // Check for VA-API encoder
    factory = gst_element_factory_find("vaapih264enc");
    if (factory && (encoder == "hardware" || encoder == "intel")) {
        gst_object_unref(factory);
        RCLCPP_INFO(this->get_logger(), "Using VA-API hardware encoder");
        return "vaapipostproc ! vaapih264enc rate-control=cbr quality-level=7";
    }

    // Check for AMD encoder
    factory = gst_element_factory_find("amfh264enc");
    if (factory && (encoder == "hardware" || encoder == "amd")) {
        gst_object_unref(factory);
        RCLCPP_INFO(this->get_logger(), "Using AMD hardware encoder");
        return "amfh264enc usage=ultra-low-latency";
    }

    // Fallback to software encoder
    RCLCPP_INFO(this->get_logger(), "Using software encoder");
    return "x264enc tune=zerolatency speed-preset=ultrafast";
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Image2rtsp>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
