#include "robot_vision/teleop_node.hpp"

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <unistd.h>

#include <algorithm>
#include <filesystem>
#include <string>

#include <libevdev/libevdev.h>
#include <linux/input-event-codes.h>

#include <std_msgs/msg/empty.hpp>

using namespace std::chrono_literals;
namespace fs = std::filesystem;

TeleopNode::TeleopNode() : Node("teleop_node")
{
    declare_parameter<double>("linear_speed",    0.5);
    declare_parameter<double>("angular_speed",   1.0);
    declare_parameter<double>("kb_timeout_s",    0.25);
    declare_parameter<double>("gp_timeout_s",    2.0);
    declare_parameter<double>("gp_deadzone",     0.1);

    declare_parameter<std::string>("key_forward",  "z");
    declare_parameter<std::string>("key_backward", "s");
    declare_parameter<std::string>("key_left",     "q");
    declare_parameter<std::string>("key_right",    "d");
    declare_parameter<std::string>("key_switch",   " ");

    declare_parameter<int>("gp_axis_linear",   ABS_Y);
    declare_parameter<int>("gp_axis_angular",  ABS_X);
    declare_parameter<int>("gp_btn_switch",    BTN_SOUTH);

    linear_speed_   = get_parameter("linear_speed").as_double();
    angular_speed_  = get_parameter("angular_speed").as_double();
    kb_timeout_s_   = get_parameter("kb_timeout_s").as_double();
    gp_timeout_s_   = get_parameter("gp_timeout_s").as_double();
    gp_deadzone_    = get_parameter("gp_deadzone").as_double();

    auto parse_key = [](const std::string& s) -> char { return s.empty() ? ' ' : s[0]; };
    key_forward_  = parse_key(get_parameter("key_forward").as_string());
    key_backward_ = parse_key(get_parameter("key_backward").as_string());
    key_left_     = parse_key(get_parameter("key_left").as_string());
    key_right_    = parse_key(get_parameter("key_right").as_string());
    key_switch_   = parse_key(get_parameter("key_switch").as_string());

    gp_axis_linear_  = get_parameter("gp_axis_linear").as_int();
    gp_axis_angular_ = get_parameter("gp_axis_angular").as_int();
    gp_btn_switch_   = get_parameter("gp_btn_switch").as_int();

    cmd_vel_pub_   = create_publisher<geometry_msgs::msg::Twist>("/teleop/cmd_vel", 10);
    switch_pub_    = create_publisher<std_msgs::msg::Empty>("/teleop/mode_switch", 10);
    publish_timer_ = create_wall_timer(50ms, std::bind(&TeleopNode::onPublishTimer, this));

    kb_thread_ = std::thread(&TeleopNode::keyboardLoop, this);
    gp_thread_ = std::thread(&TeleopNode::gamepadLoop,  this);

    RCLCPP_INFO(get_logger(),
        "Teleop ready. Keyboard : fwd='%c' bwd='%c' left='%c' right='%c' switch='%s'",
        key_forward_, key_backward_, key_left_, key_right_,
        key_switch_ == ' ' ? "SPACE" : std::string(1, key_switch_).c_str());
}

TeleopNode::~TeleopNode()
{
    running_ = false;
    if (kb_thread_.joinable()) { kb_thread_.join(); }
    if (gp_thread_.joinable()) { gp_thread_.join(); }
    restoreTerminal();
    closeGamepad();
    cmd_vel_pub_->publish(geometry_msgs::msg::Twist{});
}

void TeleopNode::setupRawTerminal()
{
    if (::tcgetattr(STDIN_FILENO, &saved_termios_) != 0) {
        RCLCPP_WARN(get_logger(), "Configuration of terminal failed, keyboard input will not work properly");
        return;
    }
    struct termios raw = saved_termios_;
    raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
    raw.c_cc[VMIN]  = 1;
    raw.c_cc[VTIME] = 1;
    ::tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    terminal_configured_ = true;
}

void TeleopNode::restoreTerminal()
{
    if (terminal_configured_) {
        ::tcsetattr(STDIN_FILENO, TCSANOW, &saved_termios_);
        terminal_configured_ = false;
    }
}

void TeleopNode::keyboardLoop()
{
    setupRawTerminal();
    RCLCPP_DEBUG(get_logger(), "Keyboard thread started.");

    while (running_) {
        char c = '\0';
        if (::read(STDIN_FILENO, &c, 1) <= 0) { continue; }

        if (c == key_switch_) {
            requestSwitch();
            continue;
        }

        InputState new_state{};
        bool known = true;

        if      (c == key_forward_)  { new_state.linear_x  =  linear_speed_; }
        else if (c == key_backward_) { new_state.linear_x  = -linear_speed_; }
        else if (c == key_left_)     { new_state.angular_z =  angular_speed_; }
        else if (c == key_right_)    { new_state.angular_z = -angular_speed_; }
        else                         { known = false; }

        if (known) {
            std::lock_guard lock(state_mutex_);
            kb_state_       = new_state;
            last_kb_input_  = std::chrono::steady_clock::now();
        }
    }

    RCLCPP_DEBUG(get_logger(), "Keyboard thread stopped.");
}

bool TeleopNode::tryOpenGamepad()
{
    std::error_code ec;
    for (const auto& entry : fs::directory_iterator("/dev/input", ec)) {
        if (ec) { break; }
        if (entry.path().filename().string().find("event") == std::string::npos) { continue; }

        const int fd = ::open(entry.path().c_str(), O_RDONLY | O_NONBLOCK);
        if (fd < 0) { continue; }

        struct libevdev* dev = nullptr;
        if (libevdev_new_from_fd(fd, &dev) < 0) {
            ::close(fd);
            continue;
        }

        const bool has_linear = libevdev_has_event_code(dev, EV_ABS, static_cast<unsigned>(gp_axis_linear_));
        const bool has_angular = libevdev_has_event_code(dev, EV_ABS, static_cast<unsigned>(gp_axis_angular_));

        if (libevdev_has_event_type(dev, EV_ABS) && has_linear && has_angular) {
            gp_fd_ = fd; gp_dev_ = dev;
            RCLCPP_INFO(get_logger(), "Gamepad found : %s (%s)", libevdev_get_name(dev), entry.path().c_str());
            return true;
        }

        libevdev_free(dev);
        ::close(fd);
    }
    return false;
}

void TeleopNode::closeGamepad()
{
    if (gp_dev_) { libevdev_free(gp_dev_); gp_dev_ = nullptr; }
    if (gp_fd_ >= 0) { ::close(gp_fd_); gp_fd_ = -1; }
}

void TeleopNode::applyAxis(int code, int raw_value, InputState& state) const
{
    const int min_v = libevdev_get_abs_minimum(gp_dev_, static_cast<unsigned>(code));
    const int max_v = libevdev_get_abs_maximum(gp_dev_, static_cast<unsigned>(code));
    if (max_v == min_v) { return; }

    const double norm = 2.0 * (raw_value - min_v) / (max_v - min_v) - 1.0;
    const double val = std::abs(norm) < gp_deadzone_ ? 0.0 : norm;

    if (code == gp_axis_linear_) {
        state.linear_x = -val * linear_speed_;
    } else if (code == gp_axis_angular_) {
        state.angular_z = -val * angular_speed_;
    }
}

void TeleopNode::gamepadLoop()
{
    while (running_) {
        if (!tryOpenGamepad()) {
            RCLCPP_INFO_ONCE(get_logger(), "No gamepad found. Please connect one. Retrying every 5 seconds...");
            for (int i = 0; i < 50 && running_; ++i) {
                std::this_thread::sleep_for(100ms);
            }
            continue;
        }

        RCLCPP_INFO(get_logger(), "Entering gamepad input loop.");

        while (running_ && gp_fd_ >= 0) {
            struct pollfd pfd{gp_fd_, POLLIN, 0};
            const int poll_rc = ::poll(&pfd, 1, 100);

            if (poll_rc < 0) {
                if (errno == EINTR) { continue; }
                RCLCPP_WARN(get_logger(), "Error polling gamepad: %s. Closing device and retrying.", std::strerror(errno));
                closeGamepad();
                { std::lock_guard lock(state_mutex_); gp_state_ = {}; }
                break;
            }
            if (poll_rc == 0) { continue; }

            bool should_switch = false;
            struct input_event ev{};
            int rc;
            while ((rc = libevdev_next_event(gp_dev_, LIBEVDEV_READ_FLAG_NORMAL, &ev)) == LIBEVDEV_READ_STATUS_SUCCESS)
            {
                std::lock_guard lock(state_mutex_);
                last_gp_input_ = std::chrono::steady_clock::now();

                if (ev.type == EV_ABS) {
                    const int code = static_cast<int>(ev.code);
                    if (code == gp_axis_linear_ || code == gp_axis_angular_) {
                        applyAxis(code, ev.value, gp_state_);
                    }
                } else if (ev.type == EV_KEY && static_cast<int>(ev.code) == gp_btn_switch_ && ev.value == 1) {
                    should_switch = true;
                }
            }

            if (should_switch) { requestSwitch(); }

            if (rc == -ENODEV) {
                RCLCPP_WARN(get_logger(), "Gamepad disconnected. Closing device and retrying.");
                closeGamepad();
                { std::lock_guard lock(state_mutex_); gp_state_ = {}; }
                for (int i = 0; i < 50 && running_; ++i) {
                    std::this_thread::sleep_for(100ms);
                }
                break;
            }
        }
    }

    RCLCPP_DEBUG(get_logger(), "Gamepad thread stopped.");
}

void TeleopNode::onPublishTimer()
{
    if (switch_pending_.exchange(false)) {
        switch_pub_->publish(std_msgs::msg::Empty{});
        RCLCPP_INFO(get_logger(), "--- Mode switch requested ---");
    }

    const auto now = std::chrono::steady_clock::now();
    InputState active;

    {
        std::lock_guard lock(state_mutex_);

        if (last_kb_input_ != std::chrono::steady_clock::time_point{}) {
            const double dt = std::chrono::duration<double>(now - last_kb_input_).count();
            if (dt > kb_timeout_s_) { kb_state_ = {}; }
        }

        bool gp_active = false;
        if (last_gp_input_ != std::chrono::steady_clock::time_point{}) {
            const double dt = std::chrono::duration<double>(now - last_gp_input_).count();
            gp_active = dt < gp_timeout_s_;
        }

        if (gp_active) {
            active = gp_state_;
        } else {
            gp_state_ = {};
            active    = kb_state_;
        }
    }

    geometry_msgs::msg::Twist twist;
    twist.linear.x  = active.linear_x;
    twist.angular.z = active.angular_z;
    cmd_vel_pub_->publish(twist);
}

void TeleopNode::requestSwitch()
{
    switch_pending_.store(true);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<TeleopNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("teleop_node"), "Fatal Error : %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}