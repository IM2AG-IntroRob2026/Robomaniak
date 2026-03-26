#include <chrono>
#include <memory>
#include <string>
#include <stdexcept>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/videoio.hpp>

using namespace std::chrono_literals;

// ─────────────────────────────────────────────────────────────────────────────
// VideoNode
//
// Capte un flux MJPEG (ex: IP Webcam sur Android) et publie chaque frame
// sur /camera/image_raw (sensor_msgs/Image, encodage bgr8).
//
// Stratégie anti-latence :
//   Avant chaque retrieve(), on vide le buffer interne OpenCV avec grab() afin
//   de toujours publier la frame la plus récente (frame dropping).
// ─────────────────────────────────────────────────────────────────────────────

class VideoNode : public rclcpp::Node
{
public:
    VideoNode() : Node("video_node")
    {
        this->declare_parameter<std::string>("stream_url", "http://192.168.1.100:8080/video");
        this->declare_parameter<int>("publish_rate_hz", 30);
        this->declare_parameter<int>("grab_drain_count", 3);

        stream_url_        = this->get_parameter("stream_url").as_string();
        const int rate_hz  = this->get_parameter("publish_rate_hz").as_int();
        grab_drain_count_  = this->get_parameter("grab_drain_count").as_int();

        pub_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);

        openStream();

        const auto period = std::chrono::duration<double>(1.0 / rate_hz);
        timer_ = this->create_wall_timer(period, std::bind(&VideoNode::onTimer, this));

        RCLCPP_INFO(this->get_logger(), "VideoNode initialized. Stream URL: %s, Publish Rate: %d Hz, Grab Drain Count: %d",
            stream_url_.c_str(), rate_hz, grab_drain_count_);
    }

private:
    void openStream()
    {
        cap_.open(stream_url_, cv::CAP_FFMPEG);
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

        if (cap_.isOpened()) {
            RCLCPP_INFO(this->get_logger(), "Stream open successfully: %s", stream_url_.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "Failed to open stream: %s", stream_url_.c_str());
        }
    }

    void onTimer()
    {
        if (!cap_.isOpened()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Stream not open, retrying...");
            openStream();
            return;
        }

        for (int i = 0; i < grab_drain_count_; ++i) {
            if (!cap_.grab()) {
                RCLCPP_WARN(this->get_logger(), "grab() failed, closing stream and will retry...");
                cap_.release();
                return;
            }
        }

        cv::Mat frame;
        if (!cap_.retrieve(frame) || frame.empty()) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "retrieve() failed or empty frame, closing stream and will retry...");
            return;
        }

        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header{}, "bgr8", frame).toImageMsg();

        img_msg->header.stamp    = this->now();
        img_msg->header.frame_id = "camera_link";

        pub_->publish(*img_msg);

        // RCLCPP_INFO(this->get_logger(), "Published frame at %s", img_msg->header.stamp.sec == 0 ? "invalid timestamp" : std::to_string(img_msg->header.stamp.sec).c_str());
    }

    std::string stream_url_;
    int grab_drain_count_{3};
    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<VideoNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("video_node"), "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}