#ifndef IMAGE2RTSP__IMAGE2RTSP_HPP_
#define IMAGE2RTSP__IMAGE2RTSP_HPP_

#include "visibility_control.h"
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include "sensor_msgs/msg/image.hpp"
#include <gst/app/gstappsrc.h>

using namespace std;

class Image2rtsp : public rclcpp::Node
{
public:
  IMAGE2RTSP_PUBLIC
  explicit Image2rtsp(const rclcpp::NodeOptions & options);
  GstRTSPServer *rtsp_server;

private:
  string topic = "/color/image_raw";
  string mountpoint = "/rs";
  string bitrate = "500";
  string framerate = "30";
  string caps = "video/x-raw,framerate=" + framerate + "/1,width=1280,height=720";
  string port = "8554";
  string pipeline;
  string pipeline_head;
  string pipeline_tail;
  bool local_only = false;
  GstAppSrc *appsrc;

  void video_mainloop_start();
  void rtsp_server_add_url(const char *url, const char *sPipeline, GstElement **appsrc);
  void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  GstRTSPServer *rtsp_server_create(const string &port, const bool local_only);
  GstCaps *gst_caps_new_from_image(const sensor_msgs::msg::Image::SharedPtr &msg);
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

static void media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media, GstElement **appsrc);
static void *mainloop(void *arg);
static gboolean session_cleanup(Image2rtsp *node, rclcpp::Logger logger, gboolean ignored);


#endif  // IMAGE2RTSP__IMAGE2RTSP_HPP_