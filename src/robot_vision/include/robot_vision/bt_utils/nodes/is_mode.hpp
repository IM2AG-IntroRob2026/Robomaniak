#pragma once

#include <behaviortree_cpp/condition_node.h>

#include "robot_vision/bt_utils/bt_context.hpp"
#include "robot_vision/bt_utils/robot_mode.hpp"

class IsMode : public BT::ConditionNode
{
private:
    std::shared_ptr<BtContext> ctx_;

public:
    IsMode(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx);

    static BT::PortsList providedPorts();
    BT::NodeStatus tick() override;
};
