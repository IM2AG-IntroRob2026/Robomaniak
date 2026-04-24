#include "robot_vision/bt_utils/nodes/teleop_action.hpp"

#include <geometry_msgs/msg/twist.hpp>

using Twist = geometry_msgs::msg::Twist;

TeleopAction::TeleopAction(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<BtContext> ctx)
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

void TeleopAction::onHalted() { ctx_->cmd_vel_pub->publish(Twist{}); }