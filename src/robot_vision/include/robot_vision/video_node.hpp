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
private:
    // Attributs
    std::string stream_url_;
    int grab_drain_count_{3};
    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

public:
    // Methods
    VideoNode();

private:
    // Methods
    void openStream();

    void onTimer();
};