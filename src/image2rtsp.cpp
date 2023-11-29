#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>
#include "../include/image2rtsp.hpp"

using std::placeholders::_1;

Image2rtsp::Image2rtsp() : Node("image2rtsp"){
    // Declare and get the parameters
    this->declare_parameter("topic");
    this->declare_parameter("mountpoint");
    this->declare_parameter("bitrate");
    this->declare_parameter("caps");
    this->declare_parameter("port");
    this->declare_parameter("local_only");

    topic = this->get_parameter("topic").as_string();
    mountpoint = this->get_parameter("mountpoint").as_string();
    bitrate = this->get_parameter("bitrate").as_string();
    caps = this->get_parameter("caps").as_string();
    port = this->get_parameter("port").as_string();
    local_only = this->get_parameter("local_only").as_bool();

    // Check if the parameter is set, since no default value is provided
    if (!this->has_parameter("topic") || !this->has_parameter("mountpoint") || !this->has_parameter("bitrate") || !this->has_parameter("caps") || !this->has_parameter("port")){
        rclcpp::shutdown(); // Shutdown the node if there are some issues with launch file
        return;
    }

    // Start the subscription
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(topic, 10, std::bind(&Image2rtsp::topic_callback, this, _1));

    // Start the RTSP server
    video_mainloop_start();
    rtsp_server = rtsp_server_create(port, local_only);
    appsrc = NULL;
    // Setup the pipeline
    pipeline_head = "( appsrc name=imagesrc do-timestamp=true min-latency=0 max-latency=0 max-bytes=1000 is-live=true ! videoconvert ! videoscale ! ";
    pipeline_tail = "key-int-max=30 ! video/x-h264, profile=baseline ! rtph264pay name=pay0 pt=96 )";
    pipeline = pipeline_head + caps + " ! x264enc tune=zerolatency bitrate=" + bitrate + pipeline_tail;
    rtsp_server_add_url(mountpoint.c_str(), pipeline.c_str(), (GstElement **)&(appsrc));
    RCLCPP_INFO(this->get_logger(), "Stream available at rtsp://%s:%s%s", gst_rtsp_server_get_address(rtsp_server), port.c_str(), mountpoint.c_str());
}

int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    try{
        auto node = std::make_shared<Image2rtsp>();
        if (rclcpp::ok()){
            rclcpp::spin(node);
        }
    }catch (const std::exception &e){
        RCLCPP_INFO(rclcpp::get_logger("image2rtsp"), "One or more required parameters are not set. Shutting down the node.");
        RCLCPP_INFO(rclcpp::get_logger("image2rtsp"), "Tip: Start the node only using launch file. Also rebuild after making changes in the YAML file");
        RCLCPP_ERROR(rclcpp::get_logger("image2rtsp"), "Parameter is not set: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
