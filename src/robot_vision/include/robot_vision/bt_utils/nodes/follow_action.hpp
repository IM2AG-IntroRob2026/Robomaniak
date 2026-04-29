#pragma once

#include <mutex>

#include <behaviortree_cpp/action_node.h>

#include "robot_vision/bt_utils/bt_context.hpp"

class FollowAction : public BT::StatefulActionNode
{
private:
    std::shared_ptr<BtContext> ctx_;

public:
    FollowAction(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};