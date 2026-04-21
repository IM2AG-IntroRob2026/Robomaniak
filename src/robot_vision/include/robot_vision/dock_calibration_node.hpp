#pragma once

#include <chrono>
#include <deque>
#include <memory>
#include <string>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class DockCalibrationNode : public rclcpp::Node
{
private:
    std::string mode_;
    int    samples_target_  {30};
    double known_distance_m_{1.0};
    double known_height_m_  {0.0};
    std::string log_path_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

    struct Sample { double x, y, z; };
    std::deque<Sample> samples_;

    rclcpp::Time start_time_;

public:
    DockCalibrationNode();

private:
    void onPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);

    void handleRaw(const Sample& s) const;
    void handlePitch(const Sample& s);
    void handleLog(const Sample& s);

    [[nodiscard]] static double estimatePitchRad(double avg_y_cv, double avg_z_cv, double known_dist_m, double known_height_m);
};