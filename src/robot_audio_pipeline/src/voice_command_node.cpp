#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

namespace robot_audio_pipeline
{

class VoiceCommandNode : public rclcpp::Node
{
public:
  VoiceCommandNode()
  : Node("voice_command_node")
  {
    this->declare_parameter<std::string>("detections_topic", "/audio/detected_hotword");
    this->declare_parameter<std::string>("cmd_vel_topic", "/audio/cmd_vel");

    this->declare_parameter<std::string>("forward_keyword", "avance");
    this->declare_parameter<std::string>("left_keyword", "gauche");
    this->declare_parameter<std::string>("right_keyword", "droite");

    this->declare_parameter<double>("forward_duration_s", 3.0);
    this->declare_parameter<double>("linear_speed", 1.0);
    this->declare_parameter<double>("turn_angle_deg", 45.0);
    this->declare_parameter<double>("angular_speed_rad_s", 1.0);
    this->declare_parameter<double>("publish_rate_hz", 30.0);

    detections_topic_ = this->get_parameter("detections_topic").as_string();
    cmd_vel_topic_ = this->get_parameter("cmd_vel_topic").as_string();

    forward_keyword_ = this->get_parameter("forward_keyword").as_string();
    left_keyword_ = this->get_parameter("left_keyword").as_string();
    right_keyword_ = this->get_parameter("right_keyword").as_string();

    forward_duration_s_ = std::max(0.1, this->get_parameter("forward_duration_s").as_double());
    linear_speed_ = this->get_parameter("linear_speed").as_double();
    turn_angle_deg_ = std::clamp(this->get_parameter("turn_angle_deg").as_double(), 1.0, 360.0);
    angular_speed_rad_s_ = std::max(
      0.05, std::abs(this->get_parameter("angular_speed_rad_s").as_double()));
    publish_rate_hz_ = std::max(5.0, this->get_parameter("publish_rate_hz").as_double());

    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_, 10);

    hotword_sub_ = this->create_subscription<std_msgs::msg::String>(
      detections_topic_, 10,
      std::bind(&VoiceCommandNode::on_hotword, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&VoiceCommandNode::on_control_timer, this));

    RCLCPP_INFO(
      this->get_logger(),
      "VoiceCommandNode pret: detections='%s', cmd_vel='%s', keywords=[%s,%s,%s]",
      detections_topic_.c_str(), cmd_vel_topic_.c_str(), forward_keyword_.c_str(),
      left_keyword_.c_str(), right_keyword_.c_str());
  }

  ~VoiceCommandNode() override
  {
    publish_stop();
  }

private:
  void on_hotword(const std_msgs::msg::String::SharedPtr msg)
  {
    if (!msg) {
      return;
    }

    if (action_active_) {
      RCLCPP_DEBUG(
        this->get_logger(), "Commande '%s' ignoree: action en cours", msg->data.c_str());
      return;
    }

    const std::string & keyword = msg->data;

    if (keyword == forward_keyword_) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = linear_speed_;
      start_action(cmd, forward_duration_s_, "avance");
      return;
    }

    if (keyword == left_keyword_) {
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = angular_speed_rad_s_;
      const double turn_duration_s = deg_to_rad(turn_angle_deg_) / angular_speed_rad_s_;
      start_action(cmd, turn_duration_s, "rotation_gauche_45");
      return;
    }

    if (keyword == right_keyword_) {
      geometry_msgs::msg::Twist cmd;
      cmd.angular.z = -angular_speed_rad_s_;
      const double turn_duration_s = deg_to_rad(turn_angle_deg_) / angular_speed_rad_s_;
      start_action(cmd, turn_duration_s, "rotation_droite_45");
      return;
    }

    RCLCPP_DEBUG(this->get_logger(), "Hotword non mappe: '%s'", keyword.c_str());
  }

  void start_action(const geometry_msgs::msg::Twist & cmd, double duration_s, const char * label)
  {
    current_cmd_ = cmd;
    action_deadline_ = this->now() + rclcpp::Duration::from_seconds(duration_s);
    action_active_ = true;

    cmd_vel_pub_->publish(current_cmd_);
    RCLCPP_INFO(
      this->get_logger(), "Action '%s' demarree (%.2f s)", label, duration_s);
  }

  void on_control_timer()
  {
    if (!action_active_) {
      return;
    }

    if (this->now() >= action_deadline_) {
      action_active_ = false;
      current_cmd_ = geometry_msgs::msg::Twist{};
      publish_stop();
      RCLCPP_INFO(this->get_logger(), "Action terminee");
      return;
    }

    cmd_vel_pub_->publish(current_cmd_);
  }

  void publish_stop()
  {
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
  }

  static double deg_to_rad(double deg)
  {
    constexpr double kPi = 3.14159265358979323846;
    return deg * kPi / 180.0;
  }

  std::string detections_topic_;
  std::string cmd_vel_topic_;

  std::string forward_keyword_;
  std::string left_keyword_;
  std::string right_keyword_;

  double forward_duration_s_{3.0};
  double linear_speed_{1.0};
  double turn_angle_deg_{45.0};
  double angular_speed_rad_s_{1.0};
  double publish_rate_hz_{30.0};

  bool action_active_{false};
  rclcpp::Time action_deadline_{0, 0, RCL_ROS_TIME};
  geometry_msgs::msg::Twist current_cmd_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr hotword_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace robot_audio_pipeline

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<robot_audio_pipeline::VoiceCommandNode>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "voice_command_node fatal error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
