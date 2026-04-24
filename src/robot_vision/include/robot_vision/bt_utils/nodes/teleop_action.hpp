#pragma once

#include <mutex>

#include <behaviortree_cpp_v3/action_node.h>

#include "robot_vision/bt_utils/bt_context.hpp"

class TeleopAction : public BT::StatefulActionNode
{
private:
    std::shared_ptr<BtContext> ctx_;

public:
    TeleopAction(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<BtContext> ctx);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override ;
};