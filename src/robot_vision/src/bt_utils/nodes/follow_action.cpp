#include "robot_vision/bt_utils/nodes/follow_action.hpp"

#include <geometry_msgs/msg/twist.hpp>

using Twist = geometry_msgs::msg::Twist;

FollowAction::FollowAction(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<BtContext> ctx)
    : BT::StatefulActionNode(name, config), ctx_(std::move(ctx)) {}

BT::PortsList FollowAction::providedPorts() { return {}; }

BT::NodeStatus FollowAction::onStart() { return BT::NodeStatus::RUNNING; }

BT::NodeStatus FollowAction::onRunning()
{
    std::lock_guard<std::mutex> lock(ctx_->cmd_mutex);
    if (ctx_->has_follow_cmd) { ctx_->cmd_vel_pub->publish(ctx_->follow_cmd); }
    return BT::NodeStatus::RUNNING;
}

void FollowAction::onHalted() { ctx_->cmd_vel_pub->publish(Twist{}); }
