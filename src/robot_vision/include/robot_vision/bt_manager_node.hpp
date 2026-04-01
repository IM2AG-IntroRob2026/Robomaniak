#include <atomic>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>

using Twist = geometry_msgs::msg::Twist;

/**
 * @brief Shared context for the behavior tree nodes, containing shared state and resources.
 */
struct BtContext
{
    std::atomic<bool> follow_mode{false};

    std::mutex cmd_mutex;
    Twist latest_cmd;
    bool has_cmd{false};

    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub;
};

class IsFollowMode : public BT::ConditionNode
{
private:
    // Attributs
    std::shared_ptr<BtContext> ctx_;
public:
    // Constructor
    IsFollowMode(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx);

    static BT::PortsList providedPorts();

    BT::NodeStatus tick() override;
};

class FollowAction : public BT::StatefulActionNode
{
private:
    std::shared_ptr<BtContext> ctx_;

public:
    // Constructor
    FollowAction(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx);
    
    // Methods
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

class IdleAction : public BT::StatefulActionNode
{
private:
    // Attributs
    std::shared_ptr<BtContext> ctx_;

public:
    // Constructor
    IdleAction(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx);

    // Methods
    static BT::PortsList providedPorts();
    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;
};

class BtManagerNode : public rclcpp::Node
{
private:
    // Attributs
    std::shared_ptr<BtContext> ctx_;
    std::string trigger_key_{" "};

    BT::BehaviorTreeFactory factory_;
    BT::Tree tree_;

    rclcpp::Subscription<Twist>::SharedPtr follow_sub_;
    rclcpp::TimerBase::SharedPtr bt_timer_;

    std::thread keyboard_thread_;
    std::atomic<bool> running_{true};

    struct termios saved_termios_{};
    bool terminal_configured_{false};
    static constexpr const char* TREE_XML = R"(
    <root BTCPP_format="4">
    <BehaviorTree ID="MainTree">
        <ReactiveFallback name="root">
          <ReactiveSequence name="follow_branch">
            <IsFollowMode name="check_mode"/>
            <FollowAction name="do_follow"/>
          </ReactiveSequence>
        <IdleAction name="do_idle"/>
        </ReactiveFallback>
    </BehaviorTree>
    </root>
    )";

public:
    // Constructor & destructor
    BtManagerNode();
    ~BtManagerNode() override;

private:
    // Methods
    void buildTree();
    void tickBt();
    void keyboardLoop();
    void setupRawTerminal();
    void restoreTerminal();
};