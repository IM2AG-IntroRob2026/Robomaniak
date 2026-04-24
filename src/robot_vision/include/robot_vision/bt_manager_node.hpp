#pragma once

#include <atomic>
#include <mutex>
#include <optional>
#include <string>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <irobot_create_msgs/action/dock.hpp>
#include <irobot_create_msgs/action/undock.hpp>
#include <irobot_create_msgs/msg/lightring_leds.hpp>
#include <irobot_create_msgs/msg/led_color.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <irobot_create_msgs/msg/hazard_detection_vector.hpp>
#include <irobot_create_msgs/msg/hazard_detection.hpp>

#include "robot_vision/bt_utils/approach_phase.hpp"
#include "robot_vision/bt_utils/bt_context.hpp"
#include "robot_vision/bt_utils/pose_2d.hpp"
#include "robot_vision/bt_utils/robot_mode.hpp"

#include "robot_vision/bt_utils/nodes/is_mode.hpp"
#include "robot_vision/bt_utils/nodes/follow_action.hpp"
#include "robot_vision/bt_utils/nodes/listen_action.hpp"
#include "robot_vision/bt_utils/nodes/teleop_action.hpp"

#include "robot_vision/librairies/led_manager.hpp"
#include "robot_vision/librairies/sound_manager.hpp"

using Twist = geometry_msgs::msg::Twist;

class BtManagerNode : public rclcpp::Node
{
private:
    // Attributs
    std::shared_ptr<BtContext> ctx_;

    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;

    rclcpp::Subscription<Twist>::SharedPtr follow_sub_;
    rclcpp::Subscription<Twist>::SharedPtr teleop_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr  cycle_switch_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr listen_cmd_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_request_sub_;
    rclcpp::TimerBase::SharedPtr bt_timer_;

    using DockAction = irobot_create_msgs::action::Dock;
    using UndockAction = irobot_create_msgs::action::Undock;
    using LightringLeds = irobot_create_msgs::msg::LightringLeds;
    using LedColor = irobot_create_msgs::msg::LedColor;
    using UndockGoalHandle = rclcpp_action::ClientGoalHandle<UndockAction>;

    std::shared_ptr<UndockGoalHandle> active_undock_goal_;
    std::mutex                        active_undock_goal_mutex_;

    rclcpp_action::Client<DockAction>::SharedPtr   dock_client_;
    rclcpp_action::Client<UndockAction>::SharedPtr undock_client_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr dock_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr undock_sub_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr             dock_detected_sub_;
    rclcpp::TimerBase::SharedPtr                                     approach_timer_;

    std::atomic<ApproachPhase> approach_phase_{ApproachPhase::IDLE};
    rclcpp::Time               approach_phase_start_{0, 0, RCL_ROS_TIME};
    rclcpp::Time               approach_overall_start_{0, 0, RCL_ROS_TIME};

    std::mutex                 odom_mutex_;
    std::optional<Pose2D>      current_odom_pose_;
    std::optional<Pose2D>      saved_dock_odom_pose_;

    std::mutex dock_pose_mutex_;
    std::optional<geometry_msgs::msg::PoseStamped> latest_dock_pose_cam_;
    rclcpp::Time last_dock_detection_time_{0, 0, RCL_ROS_TIME};

    // Params (approach)
    double approach_coarse_tolerance_m_ {0.8};
    double approach_trigger_distance_m_ {0.6};
    double approach_trigger_lateral_m_  {0.15};
    double approach_max_linear_speed_   {0.18};
    double approach_max_angular_speed_  {0.6};
    double approach_kp_rot_             {1.2};
    double approach_kp_fwd_             {0.45};
    double approach_search_speed_       {0.4};
    double approach_coarse_timeout_s_   {60.0};
    double approach_fine_timeout_s_     {30.0};
    double approach_search_timeout_s_   {25.0};
    double approach_lost_timeout_s_     {2.0};
    double approach_tick_hz_            {20.0};
    double approach_heading_threshold_  {0.15};

    double cam_x_m_         {0.0};
    double cam_y_m_         {0.0};
    double cam_z_m_         {0.10};
    double cam_roll_deg_    {0.0};
    double cam_pitch_deg_   {-20.0};
    double cam_yaw_deg_     {0.0};

    tf2::Transform tf_cam_to_base_;

    using HazardVec = irobot_create_msgs::msg::HazardDetectionVector;
    rclcpp::Subscription<HazardVec>::SharedPtr            hazard_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_reset_sub_;

    rclcpp::Time last_dock_request_time_   {0, 0, RCL_ROS_TIME};
    rclcpp::Time last_undock_request_time_ {0, 0, RCL_ROS_TIME};
    double       request_debounce_s_       {0.5};

    using DockGoalHandle = rclcpp_action::ClientGoalHandle<DockAction>;
    std::shared_ptr<DockGoalHandle> active_dock_goal_;
    std::mutex                      active_dock_goal_mutex_;

    double dock_pending_timeout_s_ {120.0};

    std::unique_ptr<LedManager> led_mgr_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr human_present_sub_;

    std::unique_ptr<SoundManager> sound_mgr_;

public:
    // Constructor & destructor
    BtManagerNode();
    ~BtManagerNode() override;

private:
    // Methods
    void buildTree();
    void tickBt();

    void cycleMode();

    void onDockRequest();
    void onUndockRequest();

    void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg);
    void onDockPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg);
    void onDockDetected(const std_msgs::msg::Bool::ConstSharedPtr& msg);

    void approachTick();
    void startApproach();
    void triggerDockAction();
    void finishApproach(bool success);
    void abortApproach(const char* reason);
    void transitionPhase(ApproachPhase new_phase);

    void tickCoarse();
    void tickFine();
    void tickSearch();

    void onHazard(const HazardVec::ConstSharedPtr& msg);
    void onEmergencyReset();
    void onHumanPresent(const std_msgs::msg::Bool::ConstSharedPtr& msg);

    void cancelActiveDockGoal();
    void resetAllState();
    void refreshModeLed();

    void onModeRequest(const std_msgs::msg::String::ConstSharedPtr& msg);
    void cancelActiveUndockGoal();
    void switchToTeleopAndCancelEverything(const char* reason);

    // Helpers
    [[nodiscard]] bool   dockDetectedRecently() const;
    [[nodiscard]] double phaseElapsedSec() const;
    [[nodiscard]] std::optional<Pose2D> currentOdomCopy() const;
    [[nodiscard]] std::optional<Pose2D> savedDockPoseCopy() const;
    [[nodiscard]] std::optional<geometry_msgs::msg::PoseStamped> latestDockPoseCopy() const;
    void publishZeroCmd();

    void buildCameraTransform();
    [[nodiscard]] tf2::Vector3 cameraToBase(const geometry_msgs::msg::Pose& pose_cam) const;

    // Math helpers
    [[nodiscard]] static double yawFromQuaternion(double x, double y, double z, double w) noexcept;
    [[nodiscard]] static double angleDiff(double a, double b) noexcept;
};