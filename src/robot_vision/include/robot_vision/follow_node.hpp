#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>

#include <geometry_msgs/msg/twist.hpp>
#include <irobot_create_msgs/msg/hazard_detection_vector.hpp>
#include <irobot_create_msgs/msg/ir_intensity_vector.hpp>
#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

using Detection2DArray = vision_msgs::msg::Detection2DArray;
using Detection2D = vision_msgs::msg::Detection2D;
using Twist = geometry_msgs::msg::Twist;
using HazardDetectionVec = irobot_create_msgs::msg::HazardDetectionVector;
using IrIntensityVec = irobot_create_msgs::msg::IrIntensityVector;
using namespace std::chrono_literals;

enum class TrackingStrategy
{
    FIRST,          ///< Premiere detection retournee par le modele.
    MOST_CENTERED,  ///< Detection la plus proche du centre horizontal de l'image.
    LARGEST,        ///< Detection avec la plus grande aire de boite englobante.
};

/**
 * @brief Converts a string into a tracking strategy.
 * @param s Strategy name ("first", "most_centered", "largest").
 * @return TrackingStrategy corresponding to the string.
 * @throws std::invalid_argument if the strategy is unknown.
 */
[[nodiscard]] TrackingStrategy strategyFromString(const std::string& s);

enum class SafetyStatus
{
    CLEAR,       ///< No hazard detected, safe to move.
    IR_SLOW,     ///< IR intensity above slow threshold, reduce speed but keep moving
    IR_STOP,     ///< IR intensity above stop threshold, immediate halt
    BUMP_RECOVER ///< Bumper touched, little backward movement to recover, then stop
};

[[nodiscard]] SafetyStatus safetyStatusFromString(const std::string& s);

/**
 * @brief ROS2 target tracking node based on 2D detections.
 * The node:
 * - receives detections on `/detection/detections`,
 * - selects a target according to a configurable strategy,
 * - maintains a target lock between frames,
 * - publishes an angular command (`/follow/cmd_vel`) via a PD controller
 * to recenter the target horizontally in the image.
 */
class FollowNode : public rclcpp::Node
{
private: 
    // Attributs
    TrackingStrategy strategy_{TrackingStrategy::MOST_CENTERED};
    int    image_width_                {1280};
    int    image_height_               {720};

    double kp_angular_                  {1.2};
    double kd_angular_                  {0.15};
    double max_angular_speed_           {1.0};
    double angular_dead_zone_           {0.05};

    double kp_linear_                   {0.003};
    double kd_linear_                   {0.0003};
    double max_linear_speed_            {0.3};
    double linear_dead_zone_            {20.0};
    double target_bbox_height_          {300.0};
    bool   allow_reverse_               {false};

    double ir_warn_threshold_           {100.0};
    double ir_stop_threshold_           {500.0};

    bool   reverse_on_bump_             {false};
    double bump_reverse_speed_          {0.1};
    double bump_reverse_dur_            {0.6};

    double lock_max_dist_px_            {150.0};
    int    lock_lost_frames_            {10};

    double       prev_angular_error_    {0.0};
    double       prev_linear_error_     {0.0};
    rclcpp::Time prev_time_             {0};

    bool   locked_      {false};
    double locked_cx_   {0.0};
    double locked_cy_   {0.0};
    int    frames_lost_ {0};

    SafetyStatus safety_status_    {SafetyStatus::CLEAR};
    double       ir_safety_factor_ {1.0};
    rclcpp::Time bump_recover_end_ {0};

    rclcpp::Subscription<Detection2DArray>::SharedPtr detections_sub_;
    rclcpp::Subscription<IrIntensityVec>::SharedPtr ir_sub_;
    rclcpp::Subscription<HazardDetectionVec>::SharedPtr hazard_sub_;
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;

public:
    // Constructor
    FollowNode();

private:
    // Methods
    void onIr(const IrIntensityVec::ConstSharedPtr& msg);
    void onHazard(const HazardDetectionVec::ConstSharedPtr& msg);

    /**
     * @brief Main callback for detection processing.
     *
     * Steps:
     * 1) Target selection/resolution (with locking if possible),
     * 2) Calculation of the normalized error with respect to the image center,
     * 3) Application of PD control to the angular velocity,
     * 4) Publication of the rotation command.
     * @param msg 2D detection message.
     */
    void onDetections(const Detection2DArray::ConstSharedPtr& msg);

    /**
     * @brief On calcule la vitesse angulaire à appliquer pour faire tourner le robot vers la cible.
     * @param target La detection cible à suivre.
     * @param now Le timestamp actuel pour le calcul de la dérivée.
     * @return La vitesse angulaire à appliquer pour faire tourner le robot vers la cible.
     */
    double turnTowardsTarget(const Detection2D& target, const double dt, const bool dt_valid);

    /**
     * @brief Calcule la vitesse linéaire à appliquer pour faire avancer le robot vers la cible.
     * Le robot avance tant que la taille de la bbox de la cible est inférieure à un certain ratio de l'image.
     * @param target La detection cible à suivre.
     * @return La vitesse linéaire à appliquer pour avancer vers la cible.
     */
    double moveTowardsTarget(const Detection2D& target, const double dt, const bool dt_valid);

    /**
     * @brief Determines the target index to track in current detections.
     * Priority:
     * - If no object: lock loss management.
     * - If active lock: attempts to associate with the closest object to the lock center.
     * - Otherwise: initial selection according to the strategy.
     * @param dets List of detections in the frame.
     * @return Optional index of the selected target in the detections vector, or std::nullopt if no target is selected.
     */
    [[nodiscard]] std::optional<std::size_t> resolveTarget(const std::vector<Detection2D>& dets);

    /**
     * @brief Searches for the detection closest to the last locked target.
     * @param dets List of candidate detections.
     * @return Index of the best detection if its distance <= `lock_max_dist_px_`, otherwise std::nullopt.
     */
    [[nodiscard]] std::optional<std::size_t> findClosestToLock(const std::vector<Detection2D>& dets) const;

    /**
     * @brief Manages the lost frame counter when the locked target is no longer found.
     *
     * Disables the lock when the `lock_lost_frames_` threshold is reached.
     */
    void handleLockLoss();

    /**
     * @brief Selects an initial target according to the active strategy.
     * @param dets List of candidate detections.
     * @return Index of the selected detection, or std::nullopt if the list is empty.
     */
    [[nodiscard]] std::optional<std::size_t> selectByStrategy(const std::vector<Detection2D>& dets) const;

    /**
     * @brief Publish a null command to stop the robot
     */
    void stopRobot();

    /**
     * @brief Resets the internal state of the PD regulator.
     */
    void resetPD();
};
