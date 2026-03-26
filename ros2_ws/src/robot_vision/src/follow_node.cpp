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


FollowNode::FollowNode() : Node("follow_node")
{
    this->declare_parameter<std::string>("strategy", "most_centered");
    this->declare_parameter<int>("image_width", 1920);
    this->declare_parameter<int>("image_height", 1080);
    this->declare_parameter<double>("kp", 1.2);
    this->declare_parameter<double>("kd", 0.15);
    this->declare_parameter<double>("max_linear_speed", 1.0);
    this->declare_parameter<double>("max_angular_speed", 1.0);
    this->declare_parameter<double>("dead_zone", 0.05);
    this->declare_parameter<double>("max_ratio", 40.0);
    this->declare_parameter<double>("lock_max_dist_px", 150.0);
    this->declare_parameter<int>("lock_lost_frames", 10);

    try {
        strategy_ = strategyFromString(this->get_parameter("strategy").as_string());
    } catch (const std::invalid_argument& e) {
        RCLCPP_FATAL(this->get_logger(), "%s", e.what());
        throw;
    }

    image_width_       = this->get_parameter("image_width").as_int();
    image_height_      = this->get_parameter("image_height").as_int();
    kp_                = this->get_parameter("kp").as_double();
    kd_                = this->get_parameter("kd").as_double();
    max_linear_speed_  = this->get_parameter("max_linear_speed").as_double();
    max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
    dead_zone_         = this->get_parameter("dead_zone").as_double();
    max_ratio_         = this->get_parameter("max_ratio").as_double();
    lock_max_dist_px_  = this->get_parameter("lock_max_dist_px").as_double();
    lock_lost_frames_  = this->get_parameter("lock_lost_frames").as_int();

    detections_sub_ = this->create_subscription<Detection2DArray>("/detection/detections", 10, std::bind(&FollowNode::onDetections, this, std::placeholders::_1));
    cmd_vel_pub_ = this->create_publisher<Twist>("/follow/cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "FollowNode ready - strategy : %s - kp=%.2f kd=%.2f", this->get_parameter("strategy").as_string().c_str(), kp_, kd_);
}

void FollowNode::onDetections(const Detection2DArray::ConstSharedPtr& msg)
{
    const auto& dets = msg->detections;
    const rclcpp::Time now = msg->header.stamp.nanosec == 0 ? this->now() : rclcpp::Time(msg->header.stamp);
    std::optional<std::size_t> target_idx = resolveTarget(dets);

        if (!target_idx.has_value()) {
            stopRobot();
            resetPD();
            return;
        }

        const auto& target = dets[*target_idx];

        // On oriente le robot pour mettre la cible au centre de l'image
        double angular_z = turnTowardsTarget(target, now);
        // On fait avancer le robot tant que la cible n'est pas assez grande dans l'image (suivi)
        double linear_x = moveTowardsTarget(target);
                
        Twist twist;
        twist.linear.x  = linear_x;
        twist.angular.z = angular_z;
        cmd_vel_pub_->publish(twist);
    }

    double FollowNode::turnTowardsTarget(const Detection2D& target, const rclcpp::Time& now)
    {
        const double cx = target.bbox.center.position.x;

        const double image_center = image_width_ / 2.0;
        const double error = (cx - image_center) / image_center; // Betwen -1 (left edge) and 1 (right edge)

        if (std::abs(error) < dead_zone_) {
            stopRobot();
            prev_error_ = 0.0;
            prev_time_ = now;
            return 0.0;
        }

        double dt = 0.0;
        if (prev_time_.nanoseconds() > 0) {
            dt = (now - prev_time_).seconds();
        }

        const double derivative = (dt > 1e-6 && dt < 1.0) ? (error - prev_error_) / dt : 0.0;

        double angular_z = -(kp_ * error + kd_ * derivative);
        angular_z = std::clamp(angular_z, -max_angular_speed_, max_angular_speed_);

        prev_error_ = error;
        prev_time_  = now;

        RCLCPP_DEBUG(this->get_logger(),
            "error=%.3f  derivative=%.3f  omega=%.3f rad/s",
            error, derivative, angular_z);

        return angular_z;
    }

    double FollowNode::moveTowardsTarget(const Detection2D& target)
    {
        double linear_x = 0.0;
        const double w = target.bbox.size_x;
        const double h = target.bbox.size_y;
                
        // On calcule la taille de l'aire de la bbox
        const double area_bbox = w * h;

        // On calcule la taille de l'aire de l'image
        const double area_image = image_width_ * image_height_;

        // On fait le rapport entre les deux pour savoir si le robot doit avancer (suivi)
        const double ratio = area_bbox / area_image * 100.0;

        // On veut que le robot avance jusqu'à ce que le ratio atteigne max_ratio_
        if (ratio < max_ratio_) {
            linear_x = max_linear_speed_ * (1.0 - ratio / max_ratio_); // Vitesse proportionnelle à la distance (plus le ratio est petit, plus le robot avance vite).
        }

        RCLCPP_DEBUG(this->get_logger(), "ratio=%.2f%%  linear_x=%.2f m/s", ratio, linear_x);

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
    prev_error_ = 0.0;
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
