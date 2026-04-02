#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <termios.h>
#include <thread>

#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

struct libevdev;

class TeleopNode : public rclcpp::Node
{
public:
    TeleopNode();
    ~TeleopNode() override;

private:
    double linear_speed_   {0.5};
    double angular_speed_  {1.0};
    double kb_timeout_s_   {0.25};
    double gp_timeout_s_   {2.0};
    double gp_deadzone_    {0.1};

    char key_forward_  {'z'};
    char key_backward_ {'s'};
    char key_left_     {'q'};
    char key_right_    {'d'};
    char key_switch_   {' '};

    int gp_axis_linear_   {1};
    int gp_axis_angular_  {0};
    int gp_btn_switch_    {304};

    struct InputState {
        double linear_x  {0.0};
        double angular_z {0.0};
    };

    std::mutex state_mutex_;
    InputState kb_state_;
    InputState gp_state_;

    std::chrono::steady_clock::time_point last_kb_input_{};
    std::chrono::steady_clock::time_point last_gp_input_{};

    std::atomic<bool> switch_pending_ {false};

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr switch_pub_;
    rclcpp::TimerBase::SharedPtr publish_timer_;

    std::atomic<bool> running_ {true};
    std::thread kb_thread_;
    std::thread gp_thread_;

    struct termios saved_termios_ {};
    bool terminal_configured_ {false};

    int gp_fd_ {-1};
    struct libevdev* gp_dev_ {nullptr};

    void keyboardLoop();
    void gamepadLoop();
    void onPublishTimer();

    void setupRawTerminal();
    void restoreTerminal();

    [[nodiscard]] bool tryOpenGamepad();
    void closeGamepad();
    void applyAxis(int code, int raw_value, InputState& state) const;

    void requestSwitch();
};