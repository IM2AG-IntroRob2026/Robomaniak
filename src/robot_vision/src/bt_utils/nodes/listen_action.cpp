#include "robot_vision/bt_utils/nodes/listen_action.hpp"

#include <geometry_msgs/msg/twist.hpp>

using Twist = geometry_msgs::msg::Twist;

ListenAction::ListenAction(const std::string &name, const BT::NodeConfiguration &config, std::shared_ptr<BtContext> ctx)
    : BT::StatefulActionNode(name, config), ctx_(std::move(ctx)) {
}

BT::PortsList ListenAction::providedPorts() { return {}; }

BT::NodeStatus ListenAction::onStart() {
    ctx_->cmd_vel_pub->publish(Twist{});
    active_impulse_.reset();

    std::lock_guard lock(ctx_->cmd_mutex);
    ctx_->pending_listen_cmd.reset();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ListenAction::onRunning() {
    const auto now = std::chrono::steady_clock::now();

    {
        std::lock_guard lock(ctx_->cmd_mutex);
        if (ctx_->pending_listen_cmd.has_value()) {
            const std::string cmd = std::move(*ctx_->pending_listen_cmd);
            ctx_->pending_listen_cmd.reset();

            Impulse impulse;
            impulse.cmd = commandToTwist(cmd);
            impulse.end_time = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                   std::chrono::duration<double>(ctx_->impulse_duration_s));
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

void ListenAction::onHalted() {
    active_impulse_.reset();
    ctx_->cmd_vel_pub->publish(Twist{});
}

Twist ListenAction::commandToTwist(const std::string &cmd) const {
    Twist twist;
    if (cmd == "forward") { twist.linear.x = ctx_->impulse_linear_speed; } else if (cmd == "left") {
        twist.angular.z = ctx_->impulse_angular_speed;
    } else if (cmd == "right") { twist.angular.z = -ctx_->impulse_angular_speed; }
    return twist;
}
