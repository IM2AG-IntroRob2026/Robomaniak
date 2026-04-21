#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/condition_node.h>
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

using Twist = geometry_msgs::msg::Twist;

enum class RobotMode { TELEOP, FOLLOW, LISTEN };

inline const char* modeToString(const RobotMode m) noexcept
{
    switch (m) {
        case RobotMode::TELEOP: return "TELEOP";
        case RobotMode::FOLLOW: return "FOLLOW";
        case RobotMode::LISTEN: return "LISTEN";
    }
    return "UNKNOWN";
}

inline RobotMode nextMode(const RobotMode m) noexcept
{
    switch (m) {
        case RobotMode::TELEOP: return RobotMode::FOLLOW;
        case RobotMode::FOLLOW: return RobotMode::LISTEN;
        case RobotMode::LISTEN: return RobotMode::TELEOP;
    }
    return RobotMode::TELEOP;
}

/**
 * Phases de la machine d'état d'approche qui s'intercale avant l'appel
 * à l'action /dock du Create 3. Elle étend la portée utile du dock
 * bien au-delà des ~30-50cm des IR natifs.
 *
 *   - IDLE         : pas d'approche en cours.
 *   - COARSE       : on se dirige vers la pose odométrique sauvegardée au dernier
 *                  undock/dock, en open-loop sur l'odométrie.
 *   - FINE         : marqueur ArUco visible, asservissement visuel en cap + distance.
 *   - SEARCH       : rotation sur place à la recherche du marqueur.
 *   - DOCK_PENDING : l'action /dock native est en cours, les IR prennent la main.
 */
enum class ApproachPhase : uint8_t
{
    IDLE,
    COARSE,
    FINE,
    SEARCH,
    DOCK_PENDING,
};

inline const char* approachPhaseToString(const ApproachPhase p) noexcept
{
    switch (p) {
        case ApproachPhase::IDLE:         return "IDLE";
        case ApproachPhase::COARSE:       return "COARSE";
        case ApproachPhase::FINE:         return "FINE";
        case ApproachPhase::SEARCH:       return "SEARCH";
        case ApproachPhase::DOCK_PENDING: return "DOCK_PENDING";
    }
    return "UNKNOWN";
}

struct Pose2D
{
    double x   {0.0};
    double y   {0.0};
    double yaw {0.0};
};

/**
 * @brief Shared context for the behavior tree nodes, containing shared state and resources.
 */
struct BtContext
{
    std::atomic<RobotMode> mode{RobotMode::TELEOP};

    std::mutex cmd_mutex;
    Twist follow_cmd;
    bool  has_follow_cmd{false};
    Twist teleop_cmd;

    std::optional<std::string> pending_listen_cmd;
    std::atomic<bool> docking_active{false};
    RobotMode mode_before_dock{RobotMode::TELEOP};

    double impulse_duration_s    {1.5};
    double impulse_linear_speed  {0.3};
    double impulse_angular_speed {0.8};

    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
};

class IsMode : public BT::ConditionNode
{
private:
    // Attributs
    std::shared_ptr<BtContext> ctx_;
public:
    // Constructor
    IsMode(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<BtContext> ctx);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};

class FollowAction : public BT::StatefulActionNode
{
private:
    std::shared_ptr<BtContext> ctx_;

public:
    // Constructor
    FollowAction(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<BtContext> ctx);
    
    // Methods
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

class TeleopAction : public BT::StatefulActionNode
{
private:
    // Attributs
    std::shared_ptr<BtContext> ctx_;

public:
    // Constructor
    TeleopAction(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<BtContext> ctx);

    // Methods
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

class ListenAction : public BT::StatefulActionNode
{
private:
    std::shared_ptr<BtContext> ctx_;

    struct Impulse {
        Twist cmd;
        std::chrono::steady_clock::time_point end_time;
    };
    std::optional<Impulse> active_impulse_;

public:
    ListenAction(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<BtContext> ctx);

    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    [[nodiscard]] Twist commandToTwist(const std::string& cmd) const;
};

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
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr  listen_switch_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr listen_cmd_sub_;
    rclcpp::TimerBase::SharedPtr bt_timer_;

    using DockAction = irobot_create_msgs::action::Dock;
    using UndockAction = irobot_create_msgs::action::Undock;
    using LightringLeds = irobot_create_msgs::msg::LightringLeds;
    using LedColor = irobot_create_msgs::msg::LedColor;

    rclcpp_action::Client<DockAction>::SharedPtr   dock_client_;
    rclcpp_action::Client<UndockAction>::SharedPtr undock_client_;

    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr dock_sub_;
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr undock_sub_;

    rclcpp::Publisher<LightringLeds>::SharedPtr led_pub_;

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

    static constexpr const char* TREE_XML = R"(
    <root BTCPP_format="4">
      <BehaviorTree ID="MainTree">
        <ReactiveFallback name="root">
          <ReactiveSequence name="follow_branch">
            <IsMode name="check_follow" expected="FOLLOW"/>
            <FollowAction name="do_follow"/>
          </ReactiveSequence>
          <ReactiveSequence name="listen_branch">
            <IsMode name="check_listen" expected="LISTEN"/>
            <ListenAction name="do_listen"/>
          </ReactiveSequence>
          <TeleopAction name="do_teleop"/>
        </ReactiveFallback>
      </BehaviorTree>
    </root>
    )";

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

    // Helpers
    [[nodiscard]] bool   dockDetectedRecently() const;
    [[nodiscard]] double phaseElapsedSec() const;
    [[nodiscard]] std::optional<Pose2D> currentOdomCopy() const;
    [[nodiscard]] std::optional<Pose2D> savedDockPoseCopy() const;
    [[nodiscard]] std::optional<geometry_msgs::msg::PoseStamped> latestDockPoseCopy() const;
    void publishZeroCmd();

    // LED helpers
    void publishLed(const LightringLeds& msg);
    [[nodiscard]] static LightringLeds makeLightring(uint8_t r, uint8_t g, uint8_t b);

    // Math helpers
    [[nodiscard]] static double yawFromQuaternion(double x, double y, double z, double w) noexcept;
    [[nodiscard]] static double angleDiff(double a, double b) noexcept;
};