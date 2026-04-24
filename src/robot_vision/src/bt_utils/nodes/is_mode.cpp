#include "robot_vision/bt_utils/nodes/is_mode.hpp"

IsMode::IsMode(const std::string& name, const BT::NodeConfiguration& config, std::shared_ptr<BtContext> ctx)
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