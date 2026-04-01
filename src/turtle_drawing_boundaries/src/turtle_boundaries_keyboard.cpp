#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

using namespace std::chrono_literals;

class TurtleBoundariesKeyboard : public rclcpp::Node
{
public:
  TurtleBoundariesKeyboard()
  : Node("turtle_boundaries_keyboard")
  {
    this->declare_parameter<double>("manual_linear_speed", 1.2);
    this->declare_parameter<double>("manual_angular_speed", 2.2);
    this->get_parameter("manual_linear_speed", manual_linear_speed_);
    this->get_parameter("manual_angular_speed", manual_angular_speed_);

    pause_toggle_pub_ = this->create_publisher<std_msgs::msg::Empty>(
      "/turtle_boundaries/pause_toggle", 10);
    manual_cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "/turtle_boundaries/manual_cmd", 10);

    if (!configure_terminal()) {
      RCLCPP_FATAL(this->get_logger(), "Terminal non interactif: impossible de lire le clavier");
      throw std::runtime_error("stdin is not a tty");
    }

    timer_ = this->create_wall_timer(30ms, std::bind(&TurtleBoundariesKeyboard::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "Clavier pret");
    RCLCPP_INFO(this->get_logger(), "Espace: pause/reprise contour");
    RCLCPP_INFO(this->get_logger(), "Direction: fleches ou ZQSD/WASD");
  }

  ~TurtleBoundariesKeyboard() override
  {
    restore_terminal();
  }

private:
  enum class KeyAction
  {
    None,
    TogglePause,
    Forward,
    Backward,
    TurnLeft,
    TurnRight,
    Stop
  };

  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr pause_toggle_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr manual_cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  int stdin_fd_{STDIN_FILENO};
  bool terminal_configured_{false};
  bool manual_mode_active_{false};
  struct termios original_terminal_state_{};
  double manual_linear_speed_{1.2};
  double manual_angular_speed_{2.2};

  bool configure_terminal()
  {
    if (!::isatty(stdin_fd_)) {
      return false;
    }

    if (::tcgetattr(stdin_fd_, &original_terminal_state_) != 0) {
      return false;
    }

    struct termios raw_state = original_terminal_state_;
    raw_state.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw_state.c_cc[VMIN] = 0;
    raw_state.c_cc[VTIME] = 0;

    if (::tcsetattr(stdin_fd_, TCSANOW, &raw_state) != 0) {
      return false;
    }

    terminal_configured_ = true;
    return true;
  }

  void restore_terminal()
  {
    if (!terminal_configured_) {
      return;
    }
    (void)::tcsetattr(stdin_fd_, TCSANOW, &original_terminal_state_);
    terminal_configured_ = false;
  }

  bool read_char(char & out_char)
  {
    fd_set read_fds;
    FD_ZERO(&read_fds);
    FD_SET(stdin_fd_, &read_fds);

    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    const int ready = ::select(stdin_fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
    if (ready <= 0) {
      return false;
    }

    char value = 0;
    const ssize_t bytes = ::read(stdin_fd_, &value, 1);
    if (bytes != 1) {
      return false;
    }

    out_char = value;
    return true;
  }

  KeyAction parse_key(char first_char)
  {
    if (first_char == ' ') {
      return KeyAction::TogglePause;
    }

    if (first_char == 'z' || first_char == 'Z' || first_char == 'w' || first_char == 'W') {
      return KeyAction::Forward;
    }
    if (first_char == 's' || first_char == 'S') {
      return KeyAction::Backward;
    }
    if (first_char == 'q' || first_char == 'Q' || first_char == 'a' || first_char == 'A') {
      return KeyAction::TurnLeft;
    }
    if (first_char == 'd' || first_char == 'D') {
      return KeyAction::TurnRight;
    }
    if (first_char == 'x' || first_char == 'X' || first_char == '0') {
      return KeyAction::Stop;
    }

    if (first_char != '\x1b') {
      return KeyAction::None;
    }

    char second_char = 0;
    char third_char = 0;
    if (!read_char(second_char) || !read_char(third_char)) {
      return KeyAction::None;
    }

    if (second_char != '[') {
      return KeyAction::None;
    }

    switch (third_char) {
      case 'A':
        return KeyAction::Forward;
      case 'B':
        return KeyAction::Backward;
      case 'C':
        return KeyAction::TurnRight;
      case 'D':
        return KeyAction::TurnLeft;
      default:
        return KeyAction::None;
    }
  }

  void publish_manual_twist(double linear, double angular)
  {
    geometry_msgs::msg::Twist cmd;
    cmd.linear.x = linear;
    cmd.angular.z = angular;
    manual_cmd_pub_->publish(cmd);
  }

  void handle_action(KeyAction action)
  {
    if (action == KeyAction::None) {
      return;
    }

    if (action == KeyAction::TogglePause) {
      std_msgs::msg::Empty toggle_msg;
      pause_toggle_pub_->publish(toggle_msg);
      manual_mode_active_ = !manual_mode_active_;
      publish_manual_twist(0.0, 0.0);
      RCLCPP_INFO(
        this->get_logger(),
        "%s",
        manual_mode_active_ ? "Mode manuel actif" : "Mode manuel inactif (reprise contour)");
      return;
    }

    if (!manual_mode_active_) {
      return;
    }

    switch (action) {
      case KeyAction::Forward:
        publish_manual_twist(manual_linear_speed_, 0.0);
        break;
      case KeyAction::Backward:
        publish_manual_twist(-manual_linear_speed_, 0.0);
        break;
      case KeyAction::TurnLeft:
        publish_manual_twist(0.0, manual_angular_speed_);
        break;
      case KeyAction::TurnRight:
        publish_manual_twist(0.0, -manual_angular_speed_);
        break;
      case KeyAction::Stop:
        publish_manual_twist(0.0, 0.0);
        break;
      case KeyAction::TogglePause:
      case KeyAction::None:
      default:
        break;
    }
  }

  void on_timer()
  {
    char pressed_key = 0;
    while (read_char(pressed_key)) {
      handle_action(parse_key(pressed_key));
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<TurtleBoundariesKeyboard>();
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    fprintf(stderr, "keyboard node error: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
