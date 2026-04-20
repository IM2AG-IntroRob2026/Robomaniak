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
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
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

    void publishLed(const LightringLeds& msg);
    [[nodiscard]] static LightringLeds makeLightring(uint8_t r, uint8_t g, uint8_t b);
};