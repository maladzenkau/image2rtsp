#include "../include/image2rtsp.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>

using std::placeholders::_1;

Image2rtsp::Image2rtsp(const rclcpp::NodeOptions & options) : Node("image2rtsp"){
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

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(Image2rtsp)
