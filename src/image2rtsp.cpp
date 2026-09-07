#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <gst/gst.h>
#include <gst/rtsp-server/rtsp-server.h>
#include <gst/app/gstappsrc.h>
#include "image2rtsp.hpp"
#include <stdexcept>
#include <string>
#ifdef __GLIBC__
#include <malloc.h>
#endif

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

    // Set up the RTSP server. The GStreamer main loop thread is started last, so a
    // failure here (e.g. the port is in use) can still be reported by throwing.
    gst_init(NULL, NULL);

    // With a camera source nothing touches the device until the first client sends
    // DESCRIBE, so a busy or missing device used to surface only as a 503 on the
    // client with no trace in the ROS log. Check it up front and refuse to start.
    if (camera){
        std::string device = extract_device(camera_pipeline);
        if (device.empty()){
            RCLCPP_WARN(this->get_logger(), "camera is true but camera_pipeline has no 'device=': skipping the device check");
        } else {
            RCLCPP_INFO(this->get_logger(), "Checking camera device %s", device.c_str());
            probe_camera_device(device);
            RCLCPP_INFO(this->get_logger(), "Camera device %s is available", device.c_str());
        }
    }

    rtsp_server = rtsp_server_create(port, local_only);

    pipeline = camera ? camera_pipeline : default_pipeline;
    framerate = extract_framerate(pipeline, 30);
    rtsp_server_add_url(mountpoint.c_str(), pipeline.c_str());
    video_mainloop_start();

    gchar *server_address = gst_rtsp_server_get_address(rtsp_server);
    if (local_only) {
        RCLCPP_INFO(this->get_logger(), "Stream available at rtsp://%s:%s%s", server_address, port.c_str(), mountpoint.c_str());
    } else {
        RCLCPP_INFO(this->get_logger(), "RTSP server bound to %s:%s%s", server_address, port.c_str(), mountpoint.c_str());
        RCLCPP_INFO(this->get_logger(), "Connect clients using rtsp://<host-ip>:%s%s (0.0.0.0 is bind-only)", port.c_str(), mountpoint.c_str());
    }
    g_free(server_address);
}

unsigned int Image2rtsp::extract_framerate(const std::string& pipeline, unsigned int default_framerate) {
    std::string search_str = "framerate=";
    size_t pos = pipeline.find(search_str);
    if (pos == std::string::npos) {
        RCLCPP_WARN(this->get_logger(), "Framerate not found in pipeline, using default: %u", default_framerate);
        return default_framerate;
    }

    pos += search_str.length();

    size_t end_pos = pipeline.find_first_of("/,", pos);
    if (end_pos == std::string::npos) {
        RCLCPP_WARN(this->get_logger(), "Invalid framerate format in pipeline, using default: %u", default_framerate);
        return default_framerate;
    }

    std::string framerate_str = pipeline.substr(pos, end_pos - pos);

    framerate_str.erase(0, framerate_str.find_first_not_of(" \t"));
    framerate_str.erase(framerate_str.find_last_not_of(" \t") + 1);
    
    try {
        int framerate = std::stoi(framerate_str);
        if (framerate <= 0) {
            RCLCPP_WARN(this->get_logger(), "Invalid framerate value %d, using default: %u", framerate, default_framerate);
            return default_framerate;
        }
        RCLCPP_INFO(this->get_logger(), "Using set framerate %d", framerate);
        return static_cast<unsigned int>(framerate);
    } catch (const std::exception& e) {
        RCLCPP_WARN(this->get_logger(), "Failed to parse framerate '%s', using default: %u", framerate_str.c_str(), default_framerate);
        return default_framerate;
    }
}

/* Pull the "device=..." value out of a gst-launch pipeline description. */
std::string Image2rtsp::extract_device(const std::string &pipeline){
    const std::string key = "device=";
    size_t pos = pipeline.find(key);
    if (pos == std::string::npos) return "";
    pos += key.length();
    size_t end = pipeline.find_first_of(" \t\r\n!", pos);
    if (end == std::string::npos) end = pipeline.length();
    std::string device = pipeline.substr(pos, end - pos);
    /* tolerate device="/dev/video0" */
    if (device.size() >= 2 && device.front() == '"' && device.back() == '"') device = device.substr(1, device.size() - 2);
    return device;
}

/* Grab a single frame from the device so that an unusable camera (busy, missing, or
 * without a usable format) stops the node instead of failing silently per client.
 * Throws std::runtime_error, which main() turns into a FATAL log and exit code 1. */
void Image2rtsp::probe_camera_device(const std::string &device){
    std::string desc = "v4l2src device=" + device + " num-buffers=1 ! fakesink sync=false";
    GError *parse_error = nullptr;
    GstElement *probe = gst_parse_launch(desc.c_str(), &parse_error);
    if (parse_error){
        std::string msg = "Could not build the camera probe pipeline: " + std::string(parse_error->message);
        g_error_free(parse_error);
        if (probe) gst_object_unref(probe);
        throw std::runtime_error(msg);
    }

    gst_element_set_state(probe, GST_STATE_PLAYING);
    GstBus *bus = gst_element_get_bus(probe);
    /* EOS: the frame arrived and the device works. ERROR: it does not. */
    GstMessage *msg = gst_bus_timed_pop_filtered(bus, 5 * GST_SECOND,
                                                 (GstMessageType)(GST_MESSAGE_ERROR | GST_MESSAGE_EOS));
    std::string failure;
    bool timed_out = (msg == nullptr);
    if (msg && GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR){
        GError *err = nullptr;
        gchar *debug = nullptr;
        gst_message_parse_error(msg, &err, &debug);
        failure = "Camera device " + device + " is not usable: " + (err ? err->message : "unknown error");
        if (debug) failure += std::string(" (") + debug + ")";
        if (err) g_error_free(err);
        g_free(debug);
    }
    if (msg) gst_message_unref(msg);
    gst_element_set_state(probe, GST_STATE_NULL);
    gst_object_unref(bus);
    gst_object_unref(probe);

    /* A timeout is ambiguous (a slow camera looks the same as a stuck one), so warn
     * rather than refuse to start; a reported error is definitive and is fatal. */
    if (timed_out){
        RCLCPP_WARN(this->get_logger(), "No frame from %s within 5 s; starting anyway, the stream may not work", device.c_str());
    }
    if (!failure.empty()) throw std::runtime_error(failure);
}

int main(int argc, char *argv[]){
#ifdef __GLIBC__
    // Every RTSP media pipeline starts a new set of threads, and glibc gives each
    // thread its own malloc arena whose freed memory is not returned to the OS.
    // With clients connecting and disconnecting this grew by ~5 MB per cycle;
    // a single arena keeps it flat at no measurable CPU cost. Must run before
    // any thread is created.
    mallopt(M_ARENA_MAX, 1);
#endif
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
