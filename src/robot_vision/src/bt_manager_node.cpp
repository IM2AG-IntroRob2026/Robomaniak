#include "robot_vision/bt_manager_node.hpp"

#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

using Twist = geometry_msgs::msg::Twist;

IsMode::IsMode(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx)
    : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

BT::PortsList IsMode::providedPorts()
{
    return { BT::InputPort<std::string>("expected") };
}

BT::NodeStatus IsMode::tick()
{
    const auto expected_str = getInput<std::string>("expected").value();

    RobotMode expected = RobotMode::TELEOP;
    if (expected_str == "FOLLOW") { expected = RobotMode::FOLLOW; }
    else if (expected_str == "LISTEN") { expected = RobotMode::LISTEN; }

    return ctx_->mode.load() == expected ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

FollowAction::FollowAction(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx)
    : BT::StatefulActionNode(name, config), ctx_(std::move(ctx)) {}

BT::PortsList FollowAction::providedPorts() { return {}; }

BT::NodeStatus FollowAction::onStart()   { return BT::NodeStatus::RUNNING; }

BT::NodeStatus FollowAction::onRunning()
{
    std::lock_guard<std::mutex> lock(ctx_->cmd_mutex);
    if (ctx_->has_follow_cmd) {
        ctx_->cmd_vel_pub->publish(ctx_->follow_cmd);
    }
    return BT::NodeStatus::RUNNING;
}

void FollowAction::onHalted()
{
    ctx_->cmd_vel_pub->publish(Twist{});
}

TeleopAction::TeleopAction(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx)
    : BT::StatefulActionNode(name, config), ctx_(std::move(ctx)) {}

BT::PortsList TeleopAction::providedPorts() { return {}; }

BT::NodeStatus TeleopAction::onStart()
{
    ctx_->cmd_vel_pub->publish(Twist{});
    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus TeleopAction::onRunning()
{
    std::lock_guard lock(ctx_->cmd_mutex);
    ctx_->cmd_vel_pub->publish(ctx_->teleop_cmd);
    return BT::NodeStatus::RUNNING;
}

void TeleopAction::onHalted()
{
    ctx_->cmd_vel_pub->publish(Twist{});
}

ListenAction::ListenAction(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx)
    : BT::StatefulActionNode(name, config), ctx_(std::move(ctx)) {}

BT::PortsList ListenAction::providedPorts() { return {}; }

BT::NodeStatus ListenAction::onStart()
{
    ctx_->cmd_vel_pub->publish(Twist{});
    active_impulse_.reset();

    std::lock_guard lock(ctx_->cmd_mutex);
    ctx_->pending_listen_cmd.reset();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ListenAction::onRunning()
{
    const auto now = std::chrono::steady_clock::now();

    {
        std::lock_guard lock(ctx_->cmd_mutex);
        if (ctx_->pending_listen_cmd.has_value()) {
            const std::string cmd = std::move(*ctx_->pending_listen_cmd);
            ctx_->pending_listen_cmd.reset();

            Impulse impulse;
            impulse.cmd = commandToTwist(cmd);
            impulse.end_time = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(ctx_->impulse_duration_s));
            active_impulse_ = impulse;
        }
    }

    if (active_impulse_.has_value()) {
        if (now < active_impulse_->end_time) {
            ctx_->cmd_vel_pub->publish(active_impulse_->cmd);
        } else {
            ctx_->cmd_vel_pub->publish(Twist{});
            active_impulse_.reset();
        }
    }

    return BT::NodeStatus::RUNNING;
}

void ListenAction::onHalted()
{
    active_impulse_.reset();
    ctx_->cmd_vel_pub->publish(Twist{});
}

Twist ListenAction::commandToTwist(const std::string& cmd) const
{
    Twist twist;
    if (cmd == "forward") { twist.linear.x  =  ctx_->impulse_linear_speed;  }
    else if (cmd == "left") { twist.angular.z  =  ctx_->impulse_angular_speed; }
    else if (cmd == "right") { twist.angular.z  = -ctx_->impulse_angular_speed; }
    return twist;
}

BtManagerNode::BtManagerNode() : Node("bt_manager_node")
{
    declare_parameter<double>("bt_tick_hz", 50.0);
    declare_parameter<double>("impulse_duration_s", 1.5);
    declare_parameter<double>("impulse_linear_speed", 0.3);
    declare_parameter<double>("impulse_angular_speed", 0.8);

    const double tick_hz = get_parameter("bt_tick_hz").as_double();

    ctx_                       = std::make_shared<BtContext>();
    ctx_->cmd_vel_pub          = create_publisher<Twist>("/cmd_vel", 10);
    ctx_->impulse_duration_s   = get_parameter("impulse_duration_s").as_double();
    ctx_->impulse_linear_speed = get_parameter("impulse_linear_speed").as_double();
    ctx_->impulse_angular_speed= get_parameter("impulse_angular_speed").as_double();

    follow_sub_ = create_subscription<Twist>(
        "/follow/cmd_vel", 10,
        [this](const Twist::ConstSharedPtr& msg) {
            std::lock_guard<std::mutex> lock(ctx_->cmd_mutex);
            ctx_->follow_cmd = *msg;
            ctx_->has_follow_cmd = true;
        });

    teleop_sub_ = create_subscription<Twist>(
        "/teleop/cmd_vel", 10,
        [this](const Twist::ConstSharedPtr& msg) {
            std::lock_guard<std::mutex> lock(ctx_->cmd_mutex);
            ctx_->teleop_cmd = *msg;
        });

    listen_cmd_sub_ = create_subscription<std_msgs::msg::String>(
        "/listen/command", 10,
        [this](const std_msgs::msg::String::ConstSharedPtr& msg) {
            if (ctx_->mode.load() != RobotMode::LISTEN) { return; }
            std::lock_guard<std::mutex> lock(ctx_->cmd_mutex);
            ctx_->pending_listen_cmd = msg->data;
        });

    cycle_switch_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/teleop/mode_switch", 10,
        [this](const std_msgs::msg::Empty::ConstSharedPtr&) {
            cycleMode();
        });

    listen_switch_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/listen/mode_switch", 10,
        [this](const std_msgs::msg::Empty::ConstSharedPtr&) {
            const RobotMode prev = ctx_->mode.exchange(RobotMode::LISTEN);
            if (prev != RobotMode::LISTEN) {
                RCLCPP_INFO(get_logger(),
                    "--- Wake word detected : %s -> LISTEN ---",
                    modeToString(prev));
            }
        });

    buildTree();

    const auto period = std::chrono::duration<double>(1.0 / tick_hz);
    bt_timer_ = this->create_wall_timer(period, std::bind(&BtManagerNode::tickBt, this));

    RCLCPP_INFO(get_logger(), "BtManagerNode ready (tick %.0f Hz). Switch via /teleop/mode_switch.", tick_hz);
}

BtManagerNode::~BtManagerNode()
{
    ctx_->cmd_vel_pub->publish(Twist{});
}

void BtManagerNode::cycleMode()
{
    RobotMode current = ctx_->mode.load();
    RobotMode next = nextMode(current);
    while (!ctx_->mode.compare_exchange_weak(current, next)) {
        next = nextMode(current);
    }
    RCLCPP_INFO(get_logger(), "--- Mode : %s -> %s ---", modeToString(current), modeToString(next));
}

void BtManagerNode::buildTree()
{
    auto ctx = ctx_;

    factory_.registerBuilder<IsMode>("IsMode",
        [ctx](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<IsMode>(name, config, ctx);
        });

    factory_.registerBuilder<FollowAction>("FollowAction",
        [ctx](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<FollowAction>(name, config, ctx);
        });

    factory_.registerBuilder<TeleopAction>("TeleopAction",
        [ctx](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<TeleopAction>(name, config, ctx);
        });

    factory_.registerBuilder<ListenAction>("ListenAction",
        [ctx](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<ListenAction>(name, config, ctx);
        });

    tree_ = factory_.createTreeFromText(TREE_XML);
}

void BtManagerNode::tickBt()
{
    tree_.tickOnce();
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<BtManagerNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("bt_manager"), "Fatal Error : %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}