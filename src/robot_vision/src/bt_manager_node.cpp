#include "robot_vision/bt_manager_node.hpp"

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

using Twist = geometry_msgs::msg::Twist;

IsFollowMode::IsFollowMode(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx)
    : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

BT::PortsList IsFollowMode::providedPorts() { return {}; }

BT::NodeStatus IsFollowMode::tick()
{
    return ctx_->follow_mode.load() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

FollowAction::FollowAction(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx)
    : BT::StatefulActionNode(name, config), ctx_(std::move(ctx)) {}

BT::PortsList FollowAction::providedPorts() { return {}; }

BT::NodeStatus FollowAction::onStart()
{
    return BT::NodeStatus::RUNNING;
}

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

BtManagerNode::BtManagerNode() : Node("bt_manager_node")
{
    this->declare_parameter<double>("bt_tick_hz", 50.0);
    const double tick_hz = this->get_parameter("bt_tick_hz").as_double();

    ctx_ = std::make_shared<BtContext>();
    ctx_->cmd_vel_pub = this->create_publisher<Twist>("/cmd_vel", 10);

    follow_sub_ = this->create_subscription<Twist>(
        "/follow/cmd_vel", 10,
        [this](const Twist::ConstSharedPtr& msg) {
            std::lock_guard<std::mutex> lock(ctx_->cmd_mutex);
            ctx_->follow_cmd = *msg;
            ctx_->has_follow_cmd = true;
        });

    teleop_sub_ = this->create_subscription<Twist>(
        "/teleop/cmd_vel", 10,
        [this](const Twist::ConstSharedPtr& msg) {
            std::lock_guard<std::mutex> lock(ctx_->cmd_mutex);
            ctx_->teleop_cmd = *msg;
        });

    switch_sub_ = this->create_subscription<std_msgs::msg::Empty>(
        "/teleop/mode_switch", 10,
        [this](const std_msgs::msg::Empty::ConstSharedPtr&) {
            const bool new_mode = !ctx_->follow_mode.load();
            ctx_->follow_mode.store(new_mode);
            RCLCPP_INFO(get_logger(), "--- Mode : %s ---", new_mode ? "FOLLOW" : "TELEOP");
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

void BtManagerNode::buildTree()
{
    auto ctx = ctx_;

    factory_.registerBuilder<IsFollowMode>("IsFollowMode",
        [ctx](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<IsFollowMode>(name, config, ctx);
        });

    factory_.registerBuilder<FollowAction>("FollowAction",
        [ctx](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<FollowAction>(name, config, ctx);
        });

    factory_.registerBuilder<TeleopAction>("TeleopAction",
        [ctx](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<TeleopAction>(name, config, ctx);
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