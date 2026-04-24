#pragma once

#include <atomic>
#include <mutex>
#include <optional>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include "robot_vision/bt_utils/robot_mode.hpp"

/**
 * @brief Shared context for the behavior tree nodes, containing shared state and resources.
 */
struct BtContext
{
    std::atomic<RobotMode> mode{RobotMode::TELEOP};

    std::mutex cmd_mutex;
    geometry_msgs::msg::Twist follow_cmd;
    bool  has_follow_cmd{false};
    geometry_msgs::msg::Twist teleop_cmd;

    std::optional<std::string> pending_listen_cmd;
    std::atomic<bool> docking_active{false};
    RobotMode mode_before_dock{RobotMode::TELEOP};

    double impulse_duration_s    {1.5};
    double impulse_linear_speed  {0.3};
    double impulse_angular_speed {0.8};

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
};