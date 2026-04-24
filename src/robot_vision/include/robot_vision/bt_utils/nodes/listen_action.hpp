#pragma once

#include <mutex>

#include <behaviortree_cpp_v3/action_node.h>

#include "robot_vision/bt_utils/bt_context.hpp"

using Twist = geometry_msgs::msg::Twist;

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