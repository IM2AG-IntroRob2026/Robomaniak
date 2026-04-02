#pragma once

#include <chrono>
#include <filesystem>
#include <format>
#include <memory>
#include <string>

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
using Detection2DArray = vision_msgs::msg::Detection2DArray;

// ─────────────────────────────────────────────────────────────────────────────
// DetectionNode
//
// S'abonne à /camera/image_raw, exécute YOLOv8 via ONNX Runtime, puis :
//   - publie /detection/human_present (std_msgs/Bool)
//   - met à jour les LEDs du Create 3 (/cmd_lightring) uniquement lors d'un
//     changement d'état (humain détecté ↔ absent) pour éviter le flood.
//   - [debug] sauvegarde les frames avec détections annotées sur disque.
//
// LED :
//   Humain détecté → VERT   (  0, 255,   0)
//   Aucun humain   → BLEU   (  0,   0, 255)
// ─────────────────────────────────────────────────────────────────────────────

class DetectionNode : public rclcpp::Node
{
private : 
    // Attributs
    std::unique_ptr<robot_vision::YoloDetector> detector_;
    
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr human_pub_;
    rclcpp::Publisher<LightringLeds>::SharedPtr led_pub_;
    rclcpp::Publisher<Detection2DArray>::SharedPtr detections_pub_;

    LightringLeds led_human_;
    LightringLeds led_idle_;
    bool last_human_state_{false};

    fs::path debug_output_dir_;
    int debug_max_saves_{200};
    int debug_save_count_{0};

public:
    // Constructor
    DetectionNode();

private:
    // Methods
    void onImage(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

    [[nodiscard]] static Detection2DArray toDetection2DArray(const std::vector<robot_vision::Detection>& dets, const std_msgs::msg::Header& header);

    [[nodiscard]] bool debugEnabled() const noexcept;

    void saveDebugImage(const cv::Mat& frame, const std::vector<robot_vision::Detection>& detections);

    void publishLed(const LightringLeds& msg);

    [[nodiscard]] static LightringLeds makeLightring(uint8_t r, uint8_t g, uint8_t b);
};