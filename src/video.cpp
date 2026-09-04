#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <opencv2/imgcodecs.hpp>
#include <algorithm>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

#include "image2rtsp.hpp"

void Image2rtsp::video_mainloop_start(){
    main_loop = g_main_loop_new(NULL, FALSE);
    main_loop_thread = std::thread([this]{ g_main_loop_run(main_loop); });
}

Image2rtsp::~Image2rtsp(){
    /* close every client connection, stop the GStreamer main loop and release the server */
    GList *clients = gst_rtsp_server_client_filter(rtsp_server,
        [](GstRTSPServer *, GstRTSPClient *, gpointer){ return GST_RTSP_FILTER_REMOVE; }, nullptr);
    g_list_free_full(clients, g_object_unref);
    if (main_loop){
        g_main_loop_quit(main_loop);
        main_loop_thread.join();
    }
    if (cleanup_source_id) g_source_remove(cleanup_source_id);
    if (server_source_id) g_source_remove(server_source_id);
    g_object_unref(rtsp_server);
    if (main_loop) g_main_loop_unref(main_loop);

    /* media that never got to emit "unprepared" still holds appsrc references in the list */
    std::lock_guard<std::mutex> lock(appsrc_mutex);
    for (GstAppSrc *appsrc : appsrc_list) gst_object_unref(appsrc);
    appsrc_list.clear();
    if (caps) gst_caps_unref(caps);
}

GstRTSPServer *Image2rtsp::rtsp_server_create(const std::string &port, bool local_only){
    GstRTSPServer *server = gst_rtsp_server_new();
    g_object_set(server, "service", port.c_str(), NULL);
    if (local_only){
        g_object_set(server, "address", "127.0.0.1", NULL);
    }
    /* attach the server to the default main context, which the main loop thread will run */
    server_source_id = gst_rtsp_server_attach(server, NULL);
    if (server_source_id == 0){
        gchar *address = gst_rtsp_server_get_address(server);
        std::string msg = "Could not bind the RTSP server to " + std::string(address) + ":" + port + " (is the port already in use?)";
        g_free(address);
        g_object_unref(server);
        throw std::runtime_error(msg);
    }
    /* periodically remove expired sessions */
    cleanup_source_id = g_timeout_add_seconds(2, session_cleanup, this);
    return server;
}

void Image2rtsp::rtsp_server_add_url(const char *url, const char *sPipeline){
    GstRTSPMountPoints *mounts;
    GstRTSPMediaFactory *factory;

    /* get the mount points for this server, every server has a default object
     * that be used to map uri mount points to media factories */
    mounts = gst_rtsp_server_get_mount_points(rtsp_server);

    /* make a media factory for a test stream. The default media factory can use
     * gst-launch syntax to create pipelines.
     * any launch line works as long as it contains elements named pay%d. Each
     * element with pay%d names will be a stream */
    factory = gst_rtsp_media_factory_new();
    gst_rtsp_media_factory_set_launch(factory, sPipeline);

    /* notify when our media is ready, This is called whenever someone asks for
     * the media and a new pipeline is created */
    // Pass `this` as user_data so media_configure can register the media's appsrc on the node
    g_signal_connect(factory, "media-configure", G_CALLBACK(media_configure), this);

    gst_rtsp_media_factory_set_shared(factory, TRUE);

    /* attach the factory to the url */
    gst_rtsp_mount_points_add_factory(mounts, url, factory);

    /* don't need the ref to the mounts anymore */
    g_object_unref(mounts);
}

struct MediaCleanupData {
    Image2rtsp *node;
    GstAppSrc *appsrc;
};

void Image2rtsp::media_unprepared(GstRTSPMedia *, gpointer user_data){
    MediaCleanupData *data = static_cast<MediaCleanupData*>(user_data);
    {
        std::lock_guard<std::mutex> lock(data->node->appsrc_mutex);
        auto &list = data->node->appsrc_list;
        list.erase(std::remove(list.begin(), list.end(), data->appsrc), list.end());
    }
    gst_object_unref(data->appsrc);
    delete data;
}

void Image2rtsp::media_configure(GstRTSPMediaFactory *, GstRTSPMedia *media, gpointer user_data){
    Image2rtsp *node = static_cast<Image2rtsp*>(user_data);
    GstElement *pipeline = gst_rtsp_media_get_element(media);
    GstElement *imagesrc = gst_bin_get_by_name(GST_BIN(pipeline), "imagesrc");
    gst_object_unref(pipeline);

    if (imagesrc){
        GstAppSrc *appsrc = GST_APP_SRC(imagesrc);

        gst_util_set_object_arg(G_OBJECT(appsrc), "format", "time");
        gst_app_src_set_stream_type(appsrc, GST_APP_STREAM_TYPE_STREAM);
        /* Queue at most two frames. If the encoder cannot keep up with the topic, drop the
         * oldest frame instead of growing the queue, and the latency, without bound. */
        gst_app_src_set_max_buffers(appsrc, 2);
        gst_app_src_set_max_bytes(appsrc, 0);
        gst_app_src_set_max_time(appsrc, 0);
#if GST_CHECK_VERSION(1, 20, 0)
        gst_app_src_set_leaky_type(appsrc, GST_APP_LEAKY_TYPE_DOWNSTREAM);
#endif

        {
            std::lock_guard<std::mutex> lock(node->appsrc_mutex);
            node->appsrc_list.push_back(appsrc);
        }

        auto *cleanup = new MediaCleanupData{node, appsrc};
        g_signal_connect(media, "unprepared", G_CALLBACK(media_unprepared), cleanup);
    }
}

const ImageFormat *Image2rtsp::format_from_encoding(const sensor_msgs::msg::Image &msg){
    // ROS encoding -> GStreamer video/x-raw format (https://gstreamer.freedesktop.org/documentation/video/video-format.html)
    namespace enc = sensor_msgs::image_encodings;
    static const std::map<std::string, ImageFormat> known_formats = {
        {enc::RGB8,        {"RGB",       false}},
        {enc::BGR8,        {"BGR",       false}},
        {enc::RGBA8,       {"RGBA",      false}},
        {enc::BGRA8,       {"BGRA",      false}},
        {enc::MONO8,       {"GRAY8",     false}},
        {enc::MONO16,      {"GRAY16_LE", false}},
        {enc::RGB16,       {"RGB",       true}},
        {enc::BGR16,       {"BGR",       true}},
        {enc::RGBA16,      {"RGBA64_LE", false}},
        {enc::BGRA16,      {"BGRA64_LE", false}},
        {enc::TYPE_8UC1,   {"GRAY8",     false}},
        {enc::TYPE_8UC3,   {"BGR",       false}},   // OpenCV channel order
        {enc::TYPE_8UC4,   {"BGRA",      false}},
        {enc::TYPE_16UC1,  {"GRAY16_LE", false}},
        {enc::YUV422,      {"UYVY",      false}},
        {enc::YUV422_YUY2, {"YUY2",      false}},
        {enc::NV21,        {"NV21",      false}},
        {enc::NV24,        {"NV24",      false}},
    };

    if (msg.is_bigendian){
        RCLCPP_ERROR(this->get_logger(), "GST: big endian image format is not supported");
        return nullptr;
    }

    auto format = known_formats.find(msg.encoding);
    if (format == known_formats.end()){
        RCLCPP_ERROR(this->get_logger(), "GST: image format '%s' unknown", msg.encoding.c_str());
        return nullptr;
    }
    return &format->second;
}

GstCaps *Image2rtsp::caps_for(const char *gst_format, int width, int height){
    if (!caps || caps_format != gst_format || caps_width != width || caps_height != height){
        if (caps) gst_caps_unref(caps);
        caps = gst_caps_new_simple("video/x-raw",
                                   "format", G_TYPE_STRING, gst_format,
                                   "width", G_TYPE_INT, width,
                                   "height", G_TYPE_INT, height,
                                   "framerate", GST_TYPE_FRACTION, framerate, 1,
                                   nullptr);
        caps_format = gst_format;
        caps_width = width;
        caps_height = height;
    }
    return caps;
}

/* Wrap memory owned by `holder` in a read-only GstBuffer without copying it.
 * The holder is deleted once the pipeline has released the buffer. */
template <typename Holder>
static GstBuffer *wrap_buffer(const void *data, size_t size, Holder *holder){
    GstBuffer *buf = gst_buffer_new_wrapped_full(GST_MEMORY_FLAG_READONLY, const_cast<void*>(data), size, 0, size,
                                                 holder, [](gpointer p){ delete static_cast<Holder*>(p); });
    GST_BUFFER_FLAG_SET(buf, GST_BUFFER_FLAG_LIVE);
    return buf;
}

/* Hand one frame to every prepared media. Takes ownership of buf. Call with appsrc_mutex held. */
void Image2rtsp::push_frame(GstBuffer *buf, GstCaps *frame_caps){
    for (GstAppSrc *appsrc : appsrc_list){
        gst_app_src_set_caps(appsrc, frame_caps);
        gst_app_src_push_buffer(appsrc, gst_buffer_ref(buf));
    }
    gst_buffer_unref(buf);
}

gboolean Image2rtsp::session_cleanup(gpointer user_data){
    Image2rtsp *node = static_cast<Image2rtsp*>(user_data);
    GstRTSPServer *server = node->rtsp_server;
    GstRTSPSessionPool *pool;
    int num;

    pool = gst_rtsp_server_get_session_pool(server);
    num = gst_rtsp_session_pool_cleanup(pool);
    g_object_unref(pool);

    if (num > 0){
        RCLCPP_DEBUG(node->get_logger(), "Sessions cleaned: %d", num);
    }
    return TRUE;
}

void Image2rtsp::topic_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    std::lock_guard<std::mutex> lock(appsrc_mutex);
    if (appsrc_list.empty()) return;

    RCLCPP_DEBUG(this->get_logger(), "Received image %dx%d, encoding=%s", msg->width, msg->height, msg->encoding.c_str());
    const ImageFormat *format = format_from_encoding(*msg);
    if (!format) return;

    GstBuffer *buf;
    if (format->reduce_to_8bit){
        /* little-endian 16-bit samples: keep the high byte */
        auto *reduced = new std::vector<uint8_t>(msg->data.size() / 2);
        for (size_t i = 0; i < reduced->size(); i++) (*reduced)[i] = msg->data[2 * i + 1];
        buf = wrap_buffer(reduced->data(), reduced->size(), reduced);
    } else {
        /* no copy: the buffer keeps the message alive until the pipeline is done with it */
        buf = wrap_buffer(msg->data.data(), msg->data.size(), new sensor_msgs::msg::Image::SharedPtr(msg));
    }
    push_frame(buf, caps_for(format->gst, msg->width, msg->height));
}

void Image2rtsp::compressed_topic_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg){
    std::lock_guard<std::mutex> lock(appsrc_mutex);
    if (appsrc_list.empty()) return;

    auto *img = new cv::Mat(cv::imdecode(cv::Mat(msg->data), cv::IMREAD_UNCHANGED));
    if (img->empty()){
        RCLCPP_ERROR(this->get_logger(), "Failed to decompress image");
        delete img;
        return;
    }

    // cv::imdecode returns OpenCV channel order: BGR / BGRA / GRAY
    const char *gst_format;
    switch (img->type()){
        case CV_8UC3: gst_format = "BGR"; break;
        case CV_8UC4: gst_format = "BGRA"; break;
        case CV_8UC1: gst_format = "GRAY8"; break;
        default:
            RCLCPP_ERROR(this->get_logger(), "Unsupported decoded image type (depth %d, %d channels) for format '%s'", img->depth(), img->channels(), msg->format.c_str());
            delete img;
            return;
    }
    if (!img->isContinuous()) *img = img->clone();

    /* the buffer keeps the decoded image alive until the pipeline is done with it */
    GstBuffer *buf = wrap_buffer(img->data, img->total() * img->elemSize(), img);
    push_frame(buf, caps_for(gst_format, img->cols, img->rows));
}
