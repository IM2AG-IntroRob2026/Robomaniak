#include <chrono>
#include <filesystem>
#include <format>
#include <memory>
#include <string>

#include "robot_vision/detection_node.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/bool.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <irobot_create_msgs/msg/lightring_leds.hpp>
#include <irobot_create_msgs/msg/led_color.hpp>

#include "robot_vision/yolo_detector.hpp"

namespace fs = std::filesystem;
using LightringLeds = irobot_create_msgs::msg::LightringLeds;
using LedColor = irobot_create_msgs::msg::LedColor;
using Detection2DArray = vision_msgs::msg::Detection2DArray;
using Detection2D = vision_msgs::msg::Detection2D;

DetectionNode::DetectionNode() : Node("detection_node")
{
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<float>("confidence_threshold", robot_vision::YOLO_DEFAULT_CONF);
    this->declare_parameter<float>("nms_iou_threshold", robot_vision::YOLO_DEFAULT_NMS_IOU);
    this->declare_parameter<bool>("use_gpu", true);
    this->declare_parameter<std::string>("debug_output_dir", "");
    this->declare_parameter<int>("debug_max_saves", 200);

    const auto  model_path      = this->get_parameter("model_path").as_string();
    const float conf_thresh     = this->get_parameter("confidence_threshold").as_double();
    const float nms_iou         = this->get_parameter("nms_iou_threshold").as_double();
    const bool  use_gpu         = this->get_parameter("use_gpu").as_bool();
    const auto  debug_dir_str   = this->get_parameter("debug_output_dir").as_string();
    debug_max_saves_            = this->get_parameter("debug_max_saves").as_int();

    // Init debug folder
    if (!debug_dir_str.empty()) {
        debug_output_dir_ = debug_dir_str;
        std::error_code ec;
        fs::create_directories(debug_output_dir_, ec);
        if (ec) {
            RCLCPP_WARN(this->get_logger(),
                "Creating debug folder '%s' failed: %s — debug disabled.",
                debug_dir_str.c_str(), ec.message().c_str());
            debug_output_dir_.clear();
        } else {
            RCLCPP_INFO(this->get_logger(),
                "Debug activated — saving annotated images to '%s' (max %d saves).",
                debug_output_dir_.string().c_str(), debug_max_saves_);
        }
    }

    RCLCPP_INFO(this->get_logger(), "Loading model : %s", model_path.c_str());
    try {
        detector_ = std::make_unique<robot_vision::YoloDetector>(model_path, use_gpu, conf_thresh, nms_iou);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to load model: %s", e.what());
        throw;
    }

    RCLCPP_INFO(this->get_logger(),
        "Model loaded successfully using %s provider. Confidence threshold: %.2f",
        std::string(detector_->providerName()).c_str(), conf_thresh);

    human_pub_ = this->create_publisher<std_msgs::msg::Bool>("/detection/human_present", 10);
    detections_pub_ = this->create_publisher<Detection2DArray>("/detection/detections", 10);
    led_pub_ = this->create_publisher<LightringLeds>("/cmd_lightring", 10);
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&DetectionNode::onImage, this, std::placeholders::_1));

    led_human_ = makeLightring(255,   0,   0);  // red
    led_idle_  = makeLightring(  0,   0, 255);  // blue
    publishLed(led_idle_);

    RCLCPP_INFO(this->get_logger(), "DetectionNode ready, waiting for images...");
}

void DetectionNode::onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
{
    cv::Mat frame;
    try {
        frame = cv_bridge::toCvShare(msg, "bgr8")->image;
    } catch (const cv_bridge::Exception& e) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "cv_bridge : %s", e.what());
        return;
    }
    if (frame.empty()) return;

    const auto detections  = detector_->detect(frame);
    const bool human_found = !detections.empty();

    std_msgs::msg::Bool bool_msg;
    bool_msg.data = human_found;

    human_pub_->publish(bool_msg);
    detections_pub_->publish(toDetection2DArray(detections, msg->header));

    if (human_found && debugEnabled()) {
        saveDebugImage(frame, detections);
        RCLCPP_INFO(this->get_logger(), "Debug image saved with %zu detection(s).", detections.size());
    }

    if (human_found != last_human_state_) {
        publishLed(human_found ? led_human_ : led_idle_);
        last_human_state_ = human_found;

        if (human_found) {
            RCLCPP_INFO(this->get_logger(),
                "Humain detected (%zu detection(s), max confidence: %.1f%%)",
                detections.size(),
                std::max_element(
                    detections.begin(), detections.end(),
                    [](const auto& a, const auto& b) {
                        return a.confidence < b.confidence;
                    }
                )->confidence);
        } else {
            RCLCPP_INFO(this->get_logger(), "No human detected.");
        }
    }
}

[[nodiscard]] Detection2DArray DetectionNode::toDetection2DArray(const std::vector<robot_vision::Detection>& dets, const std_msgs::msg::Header& header)
{
    Detection2DArray array;
    array.header = header;
    array.detections.reserve(dets.size());

    for (const auto& det : dets) {
        Detection2D d;
        d.header = header;

        d.bbox.center.position.x = static_cast<double>(det.cx);
        d.bbox.center.position.y = static_cast<double>(det.cy);
        d.bbox.size_x            = static_cast<double>(det.w);
        d.bbox.size_y            = static_cast<double>(det.h);

        vision_msgs::msg::ObjectHypothesisWithPose hyp;
        hyp.hypothesis.class_id = std::to_string(det.class_id);
        hyp.hypothesis.score    = static_cast<double>(det.confidence);
        d.results.push_back(hyp);

        array.detections.push_back(d);
    }

    return array;
}

[[nodiscard]] bool DetectionNode::debugEnabled() const noexcept
{
    return !debug_output_dir_.empty() && (debug_max_saves_ == 0 || debug_save_count_ < debug_max_saves_);
}

void DetectionNode::saveDebugImage(const cv::Mat& frame, const std::vector<robot_vision::Detection>& detections)
{
    cv::Mat annotated = frame.clone();

    for (const auto& det : detections) {
        const int x1 = static_cast<int>(det.cx - det.w / 2.0f);
        const int y1 = static_cast<int>(det.cy - det.h / 2.0f);
        const int x2 = static_cast<int>(det.cx + det.w / 2.0f);
        const int y2 = static_cast<int>(det.cy + det.h / 2.0f);

        cv::rectangle(annotated, {x1, y1}, {x2, y2}, cv::Scalar(0, 220, 0), 1);

        const std::string label = std::format("person {:.0f}%", det.confidence * 100.0f);

        int baseline = 0;
        const auto text_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.55, 1, &baseline);
        const int label_y = std::max(y1 - 4, text_size.height + 4);
        cv::rectangle(annotated,
            {x1, label_y - text_size.height - 4},
            {x1 + text_size.width + 4, label_y + baseline},
            cv::Scalar(0, 220, 0), cv::FILLED);

        cv::putText(annotated, label,
            {x1 + 2, label_y - 2},
            cv::FONT_HERSHEY_SIMPLEX, 0.55,
            cv::Scalar(0, 0, 0), 1, cv::LINE_AA);
    }

    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    const auto filename = debug_output_dir_ / std::format("detection_{}.jpg", now_ns);

    if (cv::imwrite(filename.string(), annotated)) {
        ++debug_save_count_;
        if (debug_max_saves_ > 0 && debug_save_count_ >= debug_max_saves_) {
            RCLCPP_WARN(this->get_logger(),
                "Reached debug save limit (%d), no more debug images will be saved.",
                debug_max_saves_);
        }
    } else {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Failed to save debug image to '%s'", filename.string().c_str());
    }
}

void DetectionNode::publishLed(const LightringLeds& msg)
{
    led_pub_->publish(msg);
}

[[nodiscard]] LightringLeds DetectionNode::makeLightring(uint8_t r, uint8_t g, uint8_t b)
{
    LedColor color;
    color.red   = r;
    color.green = g;
    color.blue  = b;

    LightringLeds msg;
    msg.override_system = true;
    msg.leds.fill(color);
    return msg;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<DetectionNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("detection_node"), "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}