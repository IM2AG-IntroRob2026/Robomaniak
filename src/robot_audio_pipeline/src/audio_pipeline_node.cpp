#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>

#include "robot_audio_pipeline/portaudio_input.hpp"
#include "robot_audio_pipeline/snowboy_engine.hpp"

namespace robot_audio_pipeline
{

namespace
{
std::vector<float> build_sensitivities(
  const std::vector<double> & raw_values,
  std::size_t keyword_count)
{
  if (keyword_count == 0) {
    return {};
  }

  auto clamp01 = [](double value) -> float {
      return static_cast<float>(std::clamp(value, 0.0, 1.0));
    };

  if (raw_values.empty()) {
    return std::vector<float>(keyword_count, 0.5F);
  }

  if (raw_values.size() == 1U) {
    return std::vector<float>(keyword_count, clamp01(raw_values.front()));
  }

  if (raw_values.size() != keyword_count) {
    return std::vector<float>(keyword_count, 0.5F);
  }

  std::vector<float> out;
  out.reserve(keyword_count);
  for (double value : raw_values) {
    out.push_back(clamp01(value));
  }
  return out;
}

std::vector<std::string> build_labels(
  const std::vector<std::string> & model_paths,
  const std::vector<std::string> & explicit_labels)
{
  std::vector<std::string> labels;
  labels.reserve(model_paths.size());

  for (std::size_t i = 0; i < model_paths.size(); ++i) {
    if (i < explicit_labels.size() && !explicit_labels[i].empty()) {
      labels.push_back(explicit_labels[i]);
      continue;
    }
    labels.push_back(std::filesystem::path(model_paths[i]).stem().string());
  }

  return labels;
}

float compute_rms(const std::vector<int16_t> & frame)
{
  if (frame.empty()) {
    return 0.0F;
  }

  double sum = 0.0;
  for (const auto sample : frame) {
    const double normalized = static_cast<double>(sample) / 32768.0;
    sum += normalized * normalized;
  }
  return static_cast<float>(std::sqrt(sum / static_cast<double>(frame.size())));
}
}  // namespace

class AudioPipelineNode : public rclcpp::Node
{
public:
  AudioPipelineNode()
  : Node("audio_pipeline_node")
  {
    this->declare_parameter<std::string>("resource_path", "");
    this->declare_parameter<std::vector<std::string>>(
      "hotword_model_paths", std::vector<std::string>{});
    this->declare_parameter<std::vector<std::string>>("keyword_labels", std::vector<std::string>{});
    this->declare_parameter<std::vector<double>>(
      "hotword_sensitivities", std::vector<double>{0.5});
    this->declare_parameter<double>("audio_gain", 1.0);
    this->declare_parameter<bool>("apply_frontend", true);
    this->declare_parameter<int>("frame_duration_ms", 100);
    this->declare_parameter<int>("audio_input_device_index", -1);
    this->declare_parameter<bool>("publish_audio_rms", true);
    this->declare_parameter<int>("detection_cooldown_ms", 800);
    this->declare_parameter<std::string>("detections_topic", "/audio/detected_hotword");
    this->declare_parameter<std::string>("rms_topic", "/audio/rms");

    const auto resource_path = this->get_parameter("resource_path").as_string();
    const auto hotword_model_paths = this->get_parameter("hotword_model_paths").as_string_array();
    const auto keyword_labels = this->get_parameter("keyword_labels").as_string_array();
    const auto sensitivities_raw = this->get_parameter("hotword_sensitivities").as_double_array();
    const auto audio_gain = static_cast<float>(this->get_parameter("audio_gain").as_double());
    const bool apply_frontend = this->get_parameter("apply_frontend").as_bool();
    const int frame_duration_ms = this->get_parameter("frame_duration_ms").as_int();
    const int device_index = this->get_parameter("audio_input_device_index").as_int();
    publish_audio_rms_ = this->get_parameter("publish_audio_rms").as_bool();
    detection_cooldown_ms_ = this->get_parameter("detection_cooldown_ms").as_int();

    const auto detections_topic = this->get_parameter("detections_topic").as_string();
    const auto rms_topic = this->get_parameter("rms_topic").as_string();

    if (hotword_model_paths.empty()) {
      throw std::runtime_error("hotword_model_paths est vide");
    }

    const auto sensitivities = build_sensitivities(sensitivities_raw, hotword_model_paths.size());
    if (sensitivities_raw.size() > 1U && sensitivities_raw.size() != hotword_model_paths.size()) {
      RCLCPP_WARN(
        this->get_logger(),
        "hotword_sensitivities size=%zu != hotword_model_paths size=%zu, fallback sur 0.5",
        sensitivities_raw.size(), hotword_model_paths.size());
    }

    keyword_labels_ = build_labels(hotword_model_paths, keyword_labels);

    detected_hotword_pub_ = this->create_publisher<std_msgs::msg::String>(detections_topic, 10);
    if (publish_audio_rms_) {
      rms_pub_ = this->create_publisher<std_msgs::msg::Float32>(rms_topic, 10);
    }

    std::string error;
    if (!snowboy_.initialize(
        resource_path,
        hotword_model_paths,
        sensitivities,
        audio_gain,
        apply_frontend,
        frame_duration_ms,
        &error))
    {
      throw std::runtime_error("Initialisation Snowboy impossible: " + error);
    }

    if (!audio_input_.open(
        snowboy_.sample_rate(),
        static_cast<unsigned long>(snowboy_.frame_length()),
        device_index,
        &error))
    {
      throw std::runtime_error("Initialisation PortAudio impossible: " + error);
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Audio pipeline pret: sample_rate=%d frame_length=%d input_device='%s'",
      snowboy_.sample_rate(),
      snowboy_.frame_length(),
      audio_input_.device_name().c_str());

    running_.store(true);
    worker_ = std::thread(&AudioPipelineNode::capture_loop, this);
  }

  ~AudioPipelineNode() override
  {
    running_.store(false);
    if (worker_.joinable()) {
      worker_.join();
    }
    audio_input_.close();
  }

private:
  void capture_loop()
  {
    rclcpp::Time last_detection_time(0, 0, RCL_ROS_TIME);
    std::vector<int16_t> frame;
    frame.reserve(static_cast<std::size_t>(snowboy_.frame_length()));

    while (rclcpp::ok() && running_.load()) {
      std::string error;
      if (!audio_input_.read_frame(&frame, &error)) {
        RCLCPP_WARN(this->get_logger(), "Lecture audio echouee: %s", error.c_str());
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        continue;
      }

      if (publish_audio_rms_ && rms_pub_ != nullptr) {
        std_msgs::msg::Float32 rms_msg;
        rms_msg.data = compute_rms(frame);
        rms_pub_->publish(rms_msg);
      }

      int32_t keyword_index = -1;
      if (!snowboy_.process(
          frame.data(), static_cast<int32_t>(frame.size()), &keyword_index, &error))
      {
        RCLCPP_WARN(this->get_logger(), "Inference Snowboy echouee: %s", error.c_str());
        continue;
      }

      if (keyword_index < 0) {
        continue;
      }

      const rclcpp::Time now = this->now();
      if (last_detection_time.nanoseconds() > 0) {
        const int64_t dt_ms = (now - last_detection_time).nanoseconds() / 1000000;
        if (dt_ms < detection_cooldown_ms_) {
          continue;
        }
      }
      last_detection_time = now;

      std_msgs::msg::String detected_msg;
      const auto index = static_cast<std::size_t>(keyword_index);
      if (index < keyword_labels_.size()) {
        detected_msg.data = keyword_labels_[index];
      } else {
        detected_msg.data = "keyword_" + std::to_string(keyword_index);
      }
      detected_hotword_pub_->publish(detected_msg);

      RCLCPP_INFO(
        this->get_logger(),
        "Hotword detecte: '%s' (index=%d)",
        detected_msg.data.c_str(),
        keyword_index);
    }
  }

  SnowboyEngine snowboy_;
  PortAudioInput audio_input_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr detected_hotword_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr rms_pub_;

  std::vector<std::string> keyword_labels_;
  bool publish_audio_rms_{true};
  int detection_cooldown_ms_{800};

  std::thread worker_;
  std::atomic<bool> running_{false};
};

}  // namespace robot_audio_pipeline

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<robot_audio_pipeline::AudioPipelineNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "audio_pipeline_node fatal error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
