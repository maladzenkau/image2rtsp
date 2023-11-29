#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>
#include <rclcpp/rclcpp.hpp>

#include "../include/image2rtsp.hpp"
#include "../include/image_encodings.h"

using namespace std;

static void *mainloop(void *arg){
    GMainLoop *loop = g_main_loop_new(NULL, FALSE);
    g_main_loop_run(loop);
    g_main_destroy(loop);
    return NULL;
}

void Image2rtsp::video_mainloop_start(){
    pthread_t tloop;
    gst_init(NULL, NULL);
    pthread_create(&tloop, NULL, &mainloop, NULL);
}

GstRTSPServer *Image2rtsp::rtsp_server_create(const std::string &port, const bool local_only){
    GstRTSPServer *server;

    /* create a server instance */
    server = gst_rtsp_server_new();
    // char *port = (char *) port;
    g_object_set(server, "service", port.c_str(), NULL);
    /* attach the server to the default maincontext */
    if (local_only){
    g_object_set(server, "address", "127.0.0.1", NULL);
    }
    gst_rtsp_server_attach(server, NULL);
    /* add a timeout for the session cleanup */
    g_timeout_add_seconds(2, (GSourceFunc)session_cleanup, this);
    return server;
}

void Image2rtsp::rtsp_server_add_url(const char *url, const char *sPipeline, GstElement **appsrc){
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
    g_signal_connect(factory, "media-configure", (GCallback)media_configure, appsrc);

    gst_rtsp_media_factory_set_shared(factory, TRUE);

    /* attach the factory to the url */
    gst_rtsp_mount_points_add_factory(mounts, url, factory);

    /* don't need the ref to the mounts anymore */
    g_object_unref(mounts);
}

static void media_configure(GstRTSPMediaFactory *factory, GstRTSPMedia *media, GstElement **appsrc){
    if(appsrc){
        GstElement *pipeline = gst_rtsp_media_get_element(media);

        *appsrc = gst_bin_get_by_name(GST_BIN(pipeline), "imagesrc");

        /* this instructs appsrc that we will be dealing with timed buffer */
        gst_util_set_object_arg(G_OBJECT(*appsrc), "format", "time");

        gst_object_unref(pipeline);
    }else{
        guint i, n_streams;
        n_streams = gst_rtsp_media_n_streams(media);

        for (i = 0; i < n_streams; i++){
            GstRTSPAddressPool *pool;
            GstRTSPStream *stream;
            gchar *min, *max;

            stream = gst_rtsp_media_get_stream(media, i);

            /* make a new address pool */
            pool = gst_rtsp_address_pool_new();

            min = g_strdup_printf("224.3.0.%d", (2 * i) + 1);
            max = g_strdup_printf("224.3.0.%d", (2 * i) + 2);
            gst_rtsp_address_pool_add_range(pool, min, max, 5000 + (10 * i), 5010 + (10 * i), 1);
            g_free(min);
            g_free(max);
            gst_rtsp_stream_set_address_pool(stream, pool);
            g_object_unref(pool);
        }
    }
}

GstCaps *Image2rtsp::gst_caps_new_from_image(const sensor_msgs::msg::Image::SharedPtr &msg){
    // http://gstreamer.freedesktop.org/data/doc/gstreamer/head/pwg/html/section-types-definitions.html
    static const std::map<std::string, std::string> known_formats = {
        {sensor_msgs::image_encodings::RGB8, "RGB"},
        {sensor_msgs::image_encodings::RGB16, "RGB16"},
        {sensor_msgs::image_encodings::RGBA8, "RGBA"},
        {sensor_msgs::image_encodings::RGBA16, "RGBA16"},
        {sensor_msgs::image_encodings::BGR8, "BGR"},
        {sensor_msgs::image_encodings::BGR16, "BGR16"},
        {sensor_msgs::image_encodings::BGRA8, "BGRA"},
        {sensor_msgs::image_encodings::BGRA16, "BGRA16"},
        {sensor_msgs::image_encodings::MONO8, "GRAY8"},
        {sensor_msgs::image_encodings::MONO16, "GRAY16_LE"},
    };

    if (msg->is_bigendian){
        RCLCPP_ERROR(this->get_logger(), "GST: big endian image format is not supported");
        return nullptr;
    }

    auto format = known_formats.find(msg->encoding);
    if (format == known_formats.end()){
        RCLCPP_ERROR(this->get_logger(), "GST: image format '%s' unknown", msg->encoding.c_str());
        return nullptr;
    }

    return gst_caps_new_simple("video/x-raw",
                               "format", G_TYPE_STRING, format->second.c_str(),
                               "width", G_TYPE_INT, msg->width,
                               "height", G_TYPE_INT, msg->height,
                               "framerate", GST_TYPE_FRACTION, 10, 1,
                               nullptr);
}

static gboolean session_cleanup(Image2rtsp *node, rclcpp::Logger logger, gboolean ignored){
    GstRTSPServer *server = node->rtsp_server;
    GstRTSPSessionPool *pool;
    int num;

    pool = gst_rtsp_server_get_session_pool(server);
    num = gst_rtsp_session_pool_cleanup(pool);
    g_object_unref(pool);

    if (num > 0)
    {
        char s[32];
        snprintf(s, 32, (char *)"Sessions cleaned: %d", num);
        RCLCPP_INFO(node->get_logger(), s);
    }
    return TRUE;
}

void Image2rtsp::topic_callback(const sensor_msgs::msg::Image::SharedPtr msg){
    GstBuffer *buf;
    GstCaps *caps; // image properties. see return of Image2rtsp::gst_caps_new_from_image
    char *gst_type, *gst_format = (char *)"";
    if (appsrc != NULL){
        // Set caps from message
        caps = gst_caps_new_from_image(msg);
        gst_app_src_set_caps(appsrc, caps);
        buf = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
        gst_buffer_fill(buf, 0, msg->data.data(), msg->data.size());
        GST_BUFFER_FLAG_SET(buf, GST_BUFFER_FLAG_LIVE);
        gst_app_src_push_buffer(appsrc, buf);
    }
}