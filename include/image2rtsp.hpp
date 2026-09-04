#ifndef IMAGE2RTSP_IMAGE2RTSP_HPP
#define IMAGE2RTSP_IMAGE2RTSP_HPP

#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>
#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <string>
#include <thread>
#include <vector>
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

// GStreamer video/x-raw format for a ROS image encoding
struct ImageFormat {
    const char *gst;
    bool reduce_to_8bit;   // 16-bit packed RGB/BGR: GStreamer has no such format, keep the high byte of each sample
};

class Image2rtsp : public rclcpp::Node{
public:
    Image2rtsp();
    ~Image2rtsp() override;

private:
    std::string topic;
    std::string mountpoint;
    std::string port;
    std::string pipeline;
    std::string default_pipeline;
    std::string camera_pipeline;
    bool local_only;
    bool camera;
    bool compressed;
    unsigned int framerate;

    GMainLoop *main_loop = nullptr;
    std::thread main_loop_thread;
    GstRTSPServer *rtsp_server = nullptr;
    guint server_source_id = 0;
    guint cleanup_source_id = 0;

    // The appsrc of every prepared RTSP media: added in media_configure, removed in media_unprepared.
    std::vector<GstAppSrc*> appsrc_list;
    std::mutex appsrc_mutex;

    // Caps of the last pushed frame, rebuilt only when format or size change
    GstCaps *caps = nullptr;
    std::string caps_format;
    int caps_width = 0;
    int caps_height = 0;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_compressed_;

    void video_mainloop_start();
    GstRTSPServer *rtsp_server_create(const std::string &port, bool local_only);
    void rtsp_server_add_url(const char *url, const char *sPipeline);
    unsigned int extract_framerate(const std::string &pipeline, unsigned int default_framerate);
    const ImageFormat *format_from_encoding(const sensor_msgs::msg::Image &msg);
    GstCaps *caps_for(const char *gst_format, int width, int height);
    void push_frame(GstBuffer *buf, GstCaps *frame_caps);
    void topic_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void compressed_topic_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

    // GStreamer callbacks; user_data is the Image2rtsp instance (or a MediaCleanupData for media_unprepared)
    static void media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media, gpointer user_data);
    static void media_unprepared(GstRTSPMedia *media, gpointer user_data);
    static gboolean session_cleanup(gpointer user_data);
};

#endif // IMAGE2RTSP_IMAGE2RTSP_HPP
