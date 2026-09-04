#ifndef IMAGE2RTSP_IMAGE2RTSP_HPP
#define IMAGE2RTSP_IMAGE2RTSP_HPP

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <vector>
#include <mutex>
#include <algorithm>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp>

using namespace std;

class Image2rtsp : public rclcpp::Node{
public:
    Image2rtsp();
    GstRTSPServer *rtsp_server;
    uint framerate;
    std::vector<GstAppSrc*> appsrc_list;
    std::mutex appsrc_mutex;

private:
    string topic;
    string mountpoint;
    string port;
    string pipeline;
    string default_pipeline;
    string camera_pipeline;
    bool local_only;
    bool camera;
    bool compressed;

    void video_mainloop_start();
    void rtsp_server_add_url(const char *url, const char *sPipeline);
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void compressed_topic_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    uint extract_framerate(const std::string& pipeline, uint default_framerate);
    GstRTSPServer *rtsp_server_create(const string &port, const bool local_only);
    GstCaps *gst_caps_new_from_image(const sensor_msgs::msg::Image::SharedPtr &msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_compressed_;
};

static void media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media, gpointer user_data);
static void *mainloop(void *arg);
static gboolean session_cleanup(Image2rtsp *node, rclcpp::Logger logger, gboolean ignored);

#endif // IMAGE2RTSP_IMAGE2RTSP_HPP
