#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include "robot_vision/follow_node.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

using Detection2DArray = vision_msgs::msg::Detection2DArray;
using Detection2D = vision_msgs::msg::Detection2D;
using Twist = geometry_msgs::msg::Twist;
using namespace std::chrono_literals;

[[nodiscard]] TrackingStrategy strategyFromString(const std::string& s)
{
    if (s == "first") { return TrackingStrategy::FIRST; }
    if (s == "most_centered") { return TrackingStrategy::MOST_CENTERED; }
    if (s == "largest") { return TrackingStrategy::LARGEST; }

    throw std::invalid_argument("Unknown strategy" + s + ". Valid options: first, most_centered, largest.");
}

[[nodiscard]] SafetyStatus safetyStatusFromString(const std::string& s)
{
    if (s == "clear") { return SafetyStatus::CLEAR; }
    if (s == "ir_slow") { return SafetyStatus::IR_SLOW; }
    if (s == "ir_stop") { return SafetyStatus::IR_STOP; }
    if (s == "bump_recover") { return SafetyStatus::BUMP_RECOVER; }

    throw std::invalid_argument("Unknown safety status" + s + ". Valid options: clear, ir_slow, ir_stop, bump_recover.");
}


FollowNode::FollowNode() : Node("follow_node")
{
    this->declare_parameter<std::string>("strategy", "most_centered");
    this->declare_parameter<int>("image_width", 1920);
    this->declare_parameter<int>("image_height", 1080);

    this->declare_parameter<double>("kp_angular",        1.2);
    this->declare_parameter<double>("kd_angular",        0.15);
    this->declare_parameter<double>("max_angular_speed", 1.0);
    this->declare_parameter<double>("angular_dead_zone", 0.05);

    this->declare_parameter<double>("kp_linear",          0.003);
    this->declare_parameter<double>("kd_linear",          0.0003);
    this->declare_parameter<double>("max_linear_speed",   0.3);
    this->declare_parameter<double>("linear_dead_zone",   20.0);
    this->declare_parameter<double>("target_bbox_height", 300.0);

    this->declare_parameter<bool>  ("allow_reverse",     false);

    this->declare_parameter<double>("ir_warn_threshold", 25.0);
    this->declare_parameter<double>("ir_stop_threshold", 30.0);

    this->declare_parameter<bool>  ("reverse_on_bump",   false);
    this->declare_parameter<double>("bump_reverse_speed", 0.1);
    this->declare_parameter<double>("bump_reverse_dur",   0.6);

    this->declare_parameter<double>("lock_max_dist_px", 150.0);
    this->declare_parameter<int>   ("lock_lost_frames", 10);

    try {
        strategy_ = strategyFromString(this->get_parameter("strategy").as_string());
    } catch (const std::invalid_argument& e) {
        RCLCPP_FATAL(this->get_logger(), "%s", e.what());
        throw;
    }

    image_width_        = this->get_parameter("image_width").as_int();
    image_height_       = this->get_parameter("image_height").as_int();

    kp_angular_         = this->get_parameter("kp_angular").as_double();
    kd_angular_         = this->get_parameter("kd_angular").as_double();
    max_angular_speed_  = this->get_parameter("max_angular_speed").as_double();
    angular_dead_zone_  = this->get_parameter("angular_dead_zone").as_double();

    kp_linear_          = this->get_parameter("kp_linear").as_double();
    kd_linear_          = this->get_parameter("kd_linear").as_double();
    max_linear_speed_   = this->get_parameter("max_linear_speed").as_double();
    linear_dead_zone_   = this->get_parameter("linear_dead_zone").as_double();
    target_bbox_height_ = this->get_parameter("target_bbox_height").as_double();
    allow_reverse_      = this->get_parameter("allow_reverse").as_bool();

    ir_warn_threshold_  = this->get_parameter("ir_warn_threshold").as_double();
    ir_stop_threshold_  = this->get_parameter("ir_stop_threshold").as_double();

    reverse_on_bump_    = this->get_parameter("reverse_on_bump").as_bool();
    bump_reverse_speed_ = this->get_parameter("bump_reverse_speed").as_double();
    bump_reverse_dur_   = this->get_parameter("bump_reverse_dur").as_double();

    lock_max_dist_px_   = this->get_parameter("lock_max_dist_px").as_double();
    lock_lost_frames_   = this->get_parameter("lock_lost_frames").as_int();

    detections_sub_ = this->create_subscription<Detection2DArray>("/detection/detections", 10, std::bind(&FollowNode::onDetections, this, std::placeholders::_1));

    const rclcpp::QoS sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    ir_sub_     = this->create_subscription<IrIntensityVec>    ("/ir_intensity",     sensor_qos, std::bind(&FollowNode::onIr,     this, std::placeholders::_1));
    hazard_sub_ = this->create_subscription<HazardDetectionVec>("/hazard_detection", sensor_qos, std::bind(&FollowNode::onHazard, this, std::placeholders::_1));
    cmd_vel_pub_ = this->create_publisher<Twist>("/follow/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(),
        "FollowNode ready - strategy=%s  target_h=%.0fpx  "
        "reverse=%s  reverse_on_bump=%s",
        this->get_parameter("strategy").as_string().c_str(),
        target_bbox_height_,
        allow_reverse_   ? "y" : "n",
        reverse_on_bump_ ? "y" : "n");
}

void FollowNode::onIr(const IrIntensityVec::ConstSharedPtr& msg)
{
    if (msg->readings.empty()) { return; }

    double max_ir = 0.0;
    for (const auto& r : msg->readings) {
        max_ir = std::max(max_ir, static_cast<double>(r.value));
    }

    if (safety_status_ == SafetyStatus::BUMP_RECOVER) { return; }

    if (max_ir >= ir_stop_threshold_) {
        ir_safety_factor_ = 0.0;
        safety_status_ = SafetyStatus::IR_STOP;
    } else if (max_ir >= ir_warn_threshold_) {
        ir_safety_factor_ = 1.0 - (max_ir - ir_warn_threshold_) / (ir_stop_threshold_ - ir_warn_threshold_);
        safety_status_ = SafetyStatus::IR_SLOW;
        RCLCPP_DEBUG(this->get_logger(), "IR obstacle - safety_factor=%.2f  ir_max=%.0f", ir_safety_factor_, max_ir);
    } else {
        ir_safety_factor_ = 1.0;
        safety_status_ = SafetyStatus::CLEAR;
    }
}

void FollowNode::onHazard(const HazardDetectionVec::ConstSharedPtr& msg)
{
    using HazardType = irobot_create_msgs::msg::HazardDetection;

    for (const auto& hazard : msg->detections) {
        if (hazard.type == HazardType::BUMP) {
            if (safety_status_ == SafetyStatus::BUMP_RECOVER) { break; }

            RCLCPP_WARN(this->get_logger(), "Bump detected!");

            safety_status_ = SafetyStatus::BUMP_RECOVER;
            bump_recover_end_ = this->now() + rclcpp::Duration::from_seconds(bump_reverse_dur_);
            resetPD();
            break;
        }
    }
}

void FollowNode::onDetections(const Detection2DArray::ConstSharedPtr& msg)
{
    const rclcpp::Time now = msg->header.stamp.nanosec == 0 ? this->now() : rclcpp::Time(msg->header.stamp);

    if (safety_status_ == SafetyStatus::BUMP_RECOVER) {
        if (now < bump_recover_end_) {
            Twist t;
            t.linear.x = reverse_on_bump_ ? -bump_reverse_speed_ : 0.0;
            cmd_vel_pub_->publish(t);
            return;
        }

        safety_status_ = SafetyStatus::CLEAR;
        ir_safety_factor_ = 1.0;
        RCLCPP_INFO(this->get_logger(), "Bump recovery complete, resuming normal operation.");
    }

    const auto& dets = msg->detections;
    std::optional<std::size_t> target_idx = resolveTarget(dets);

    if (!target_idx.has_value()) {
        stopRobot();
        resetPD();
        return;
    }

    const auto& target = dets[*target_idx];

    double dt = 0.0;
    if (prev_time_.nanoseconds() > 0) { dt = (now - prev_time_).seconds(); }
    const bool dt_valid = (dt > 1e-6 && dt < 1.0);
    prev_time_ = now;

    double angular_z = turnTowardsTarget(target, dt, dt_valid);
    double linear_x = moveTowardsTarget(target, dt, dt_valid);
    if (linear_x > 0.0) { linear_x *= ir_safety_factor_; }

    Twist twist;
    twist.linear.x  = linear_x;
    twist.angular.z = angular_z;
    cmd_vel_pub_->publish(twist);
}

double FollowNode::turnTowardsTarget(const Detection2D& target, const double dt, const bool dt_valid)
{
    const double cx = target.bbox.center.position.x;
    const double image_center = image_width_ / 2.0;
    const double angular_error = (cx - image_center) / image_center;                             // Normalize the error to [-1, 1], where -1 means target at left edge, 0 means target at center, and +1 means target at right edge

    double angular_z = 0.0;
    if (std::abs(angular_error) >= angular_dead_zone_) {                                         // If error is significant enough
        const double derivative = dt_valid ? (angular_error - prev_angular_error_) / dt : 0.0;   // The derivative term is the difference in error divided by the time elapsed since the last update (or 0 if dt is not valid)
        angular_z = -(kp_angular_ * angular_error + kd_angular_ * derivative);                   // The PD control law: the angular velocity is proportional to the error and its derivative. The negative sign is because a positive error (target to the right) should produce a positive angular velocity (turn left).
        angular_z = std::clamp(angular_z, -max_angular_speed_, max_angular_speed_);              // Clamp the angular velocity to the maximum allowed speed
    }
    prev_angular_error_ = angular_error;                                                         // Store the current error for the next iteration

    RCLCPP_DEBUG(this->get_logger(), "error=%.3f  derivative=%.3f  omega=%.3f rad/s",
        angular_error, dt_valid ? (angular_error - prev_angular_error_) / dt : 0.0, angular_z);

    return angular_z;
}

double FollowNode::moveTowardsTarget(const Detection2D& target, const double dt, const bool dt_valid)
{
    const double h = target.bbox.size_y;
    const double linear_error = target_bbox_height_ - h;                                         // The error is the difference between the desired bbox height and the actual bbox height.
    // A positive error means the target is too far (bbox too small) and we need to move forward, while a negative error means the target is too close (bbox too large) and we need to move backward (if allowed).

    // Same computation as for angular control, but applied to linear velocity.
    double linear_x = 0.0;
    if (std::abs(linear_error) >= linear_dead_zone_) {
        const double d_lin = dt_valid ? (linear_error - prev_linear_error_) / dt : 0.0;
        linear_x = kp_linear_ * linear_error + kd_linear_ * d_lin;
        linear_x = std::clamp(linear_x, -max_linear_speed_, max_linear_speed_);

        if (!allow_reverse_ && linear_x < 0.0) { linear_x = 0.0; }                               // If reverse is not allowed and the computed linear velocity is negative (move backward), set it to zero instead.
    }
    prev_linear_error_ = linear_error;

    RCLCPP_DEBUG(this->get_logger(), "error=%.1fpx  derivative=%.1f  vx=%.3f m/s",
        linear_error, dt_valid ? (linear_error - prev_linear_error_) / dt : 0.0, linear_x);

    return linear_x;
}

[[nodiscard]] std::optional<std::size_t> FollowNode::resolveTarget(const std::vector<Detection2D>& dets)
{
    if (dets.empty()) {
        handleLockLoss();
        return std::nullopt;
    }

    if (locked_) {
        const auto idx = findClosestToLock(dets);
        if (idx.has_value()) {
            frames_lost_ = 0;
            locked_cx_ = dets[*idx].bbox.center.position.x;
            locked_cy_ = dets[*idx].bbox.center.position.y;
            return idx;
        }

        handleLockLoss();
        if (locked_) { return std::nullopt; }
    }

    const auto idx = selectByStrategy(dets);
    if (idx.has_value()) {
        locked_ = true;
        frames_lost_ = 0;
        locked_cx_ = dets[*idx].bbox.center.position.x;
        locked_cy_ = dets[*idx].bbox.center.position.y;
        RCLCPP_INFO(this->get_logger(), "Target locked - cx=%.1f cy=%.1f", locked_cx_, locked_cy_);
    }
    return idx;
}

[[nodiscard]] std::optional<std::size_t> FollowNode::findClosestToLock(const std::vector<Detection2D>& dets) const
{
    std::size_t best_idx  = 0;
    double best_dist = std::numeric_limits<double>::max();

    for (std::size_t i = 0; i < dets.size(); ++i) {
        const double dx = dets[i].bbox.center.position.x - locked_cx_;
        const double dy = dets[i].bbox.center.position.y - locked_cy_;
        const double dist = std::hypot(dx, dy);
        if (dist < best_dist) {
            best_dist = dist;
            best_idx  = i;
        }
    }

    if (best_dist <= lock_max_dist_px_) { return best_idx; }
    return std::nullopt;
}

void FollowNode::handleLockLoss()
{
    if (!locked_) { return; }
    ++frames_lost_;
    if (frames_lost_ >= lock_lost_frames_) {
        locked_ = false;
        frames_lost_ = 0;
        RCLCPP_INFO(this->get_logger(), "Lock lost after %d frames without target", lock_lost_frames_);
    }
}

[[nodiscard]] std::optional<std::size_t> FollowNode::selectByStrategy(const std::vector<Detection2D>& dets) const
{
    if (dets.empty()) { return std::nullopt; }

    switch (strategy_) {
        case TrackingStrategy::FIRST: {
                return std::size_t{0};
            }

        case TrackingStrategy::MOST_CENTERED: {
            const double center = image_width_ / 2.0;
            std::size_t best = 0;
            double best_dist = std::numeric_limits<double>::max();
            for (std::size_t i = 0; i < dets.size(); ++i) {
                const double d = std::abs(dets[i].bbox.center.position.x - center);
                if (d < best_dist) { best_dist = d; best = i; }
            }
            return best;
        }

        case TrackingStrategy::LARGEST: {
            std::size_t best = 0;
            double best_area = 0.0;
            for (std::size_t i = 0; i < dets.size(); ++i) {
                const double area = dets[i].bbox.size_x * dets[i].bbox.size_y;
                if (area > best_area) { best_area = area; best = i; }
            }
            return best;
        }
    }
    return std::nullopt;
}

void FollowNode::stopRobot()
{
    cmd_vel_pub_->publish(Twist{});
}

/**
 * @brief Resets the internal state of the PD regulator.
 */
void FollowNode::resetPD()
{
    prev_angular_error_ = 0.0;
    prev_linear_error_ = 0.0;
    prev_time_ = rclcpp::Time(0);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<FollowNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("follow_node"), "Fatal error: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}