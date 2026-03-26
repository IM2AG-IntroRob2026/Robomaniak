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
public:
    IsFollowMode(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx)
        : BT::ConditionNode(name, config), ctx_(std::move(ctx)) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus tick() override
    {
        return ctx_->follow_mode.load() ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
    }

private:
    std::shared_ptr<BtContext> ctx_;
};

class FollowAction : public BT::StatefulActionNode
{
public:
    FollowAction(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx)
        : BT::StatefulActionNode(name, config), ctx_(std::move(ctx)) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override
    {
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override
    {
        std::lock_guard<std::mutex> lock(ctx_->cmd_mutex);
        if (ctx_->has_cmd) {
            ctx_->cmd_vel_pub->publish(ctx_->latest_cmd);
        }
        return BT::NodeStatus::RUNNING;
    }

    void onHalted() override
    {
        ctx_->cmd_vel_pub->publish(Twist{});
    }

private:
    std::shared_ptr<BtContext> ctx_;
};

class IdleAction : public BT::StatefulActionNode
{
public:
    IdleAction(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx)
        : BT::StatefulActionNode(name, config), ctx_(std::move(ctx)) {}

    static BT::PortsList providedPorts() { return {}; }

    BT::NodeStatus onStart() override
    {
        ctx_->cmd_vel_pub->publish(Twist{});
        return BT::NodeStatus::RUNNING;
    }

    BT::NodeStatus onRunning() override { return BT::NodeStatus::RUNNING; }

    void onHalted() override {}

private:
    std::shared_ptr<BtContext> ctx_;
};

class BtManagerNode : public rclcpp::Node
{
    static constexpr const char* TREE_XML = R"(
<root BTCPP_format="4">
  <BehaviorTree ID="MainTree">
    <ReactiveFallback name="root">
      <Sequence name="follow_branch">
        <IsFollowMode name="check_mode"/>
        <FollowAction name="do_follow"/>
      </Sequence>
      <IdleAction name="do_idle"/>
    </ReactiveFallback>
  </BehaviorTree>
</root>
)";

public:
    BtManagerNode() : Node("bt_manager_node")
    {
        this->declare_parameter<double>("bt_tick_hz", 50.0);
        this->declare_parameter<std::string>("trigger_key", " ");

        const double tick_hz = this->get_parameter("bt_tick_hz").as_double();
        trigger_key_ = this->get_parameter("trigger_key").as_string();
        if (trigger_key_.empty()) { trigger_key_ = " "; }

        ctx_ = std::make_shared<BtContext>();
        ctx_->cmd_vel_pub = this->create_publisher<Twist>("/cmd_vel", 10);

        follow_sub_ = this->create_subscription<Twist>(
            "/follow/cmd_vel", 10,
            [this](const Twist::ConstSharedPtr& msg) {
                std::lock_guard<std::mutex> lock(ctx_->cmd_mutex);
                ctx_->latest_cmd = *msg;
                ctx_->has_cmd = true;
            });

        buildTree();

        const auto period = std::chrono::duration<double>(1.0 / tick_hz);
        bt_timer_ = this->create_wall_timer(period, std::bind(&BtManagerNode::tickBt, this));

        keyboard_thread_ = std::thread(&BtManagerNode::keyboardLoop, this);

        RCLCPP_INFO(this->get_logger(),
            "BtManagerNode ready (tick %.0f Hz) - "
            "Press [%s] to switch between IDLE <-> FOLLOW",
            tick_hz,
            trigger_key_[0] == ' ' ? "SPACE" : trigger_key_.c_str());
    }

    ~BtManagerNode() override
    {
        running_ = false;
        if (keyboard_thread_.joinable()) keyboard_thread_.join();
        restoreTerminal();
        ctx_->cmd_vel_pub->publish(Twist{});
    }

private:
    void buildTree()
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

        factory_.registerBuilder<IdleAction>("IdleAction",
            [ctx](const std::string& name, const BT::NodeConfig& config) {
                return std::make_unique<IdleAction>(name, config, ctx);
            });

        tree_ = factory_.createTreeFromText(TREE_XML);
    }

    void tickBt()
    {
        tree_.tickOnce();
    }

    void keyboardLoop()
    {
        setupRawTerminal();
        RCLCPP_DEBUG(this->get_logger(), "Keyboard thread started.");

        while (running_) {
            char c = '\0';
            const ssize_t n = ::read(STDIN_FILENO, &c, 1);
            if (n <= 0) { continue; }

            if (c == trigger_key_[0]) {
                const bool new_mode = !ctx_->follow_mode.load();
                ctx_->follow_mode.store(new_mode);
                RCLCPP_INFO(this->get_logger(), "--- Mode : %s ---", new_mode ? "FOLLOW" : "IDLE");
            }
        }

        RCLCPP_DEBUG(this->get_logger(), "Keyboard thread stopping.");
    }

    void setupRawTerminal()
    {
        if (::tcgetattr(STDIN_FILENO, &saved_termios_) != 0) {
            RCLCPP_WARN(this->get_logger(), "Configuration of terminal failed, keyboard input will not work properly");
            return;
        }

        struct termios raw = saved_termios_;
        raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 1;  // retourner dès qu'1 octet est disponible
        raw.c_cc[VTIME] = 1; // ou après 0.1s (pour permettre l'arrêt propre)

        ::tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        terminal_configured_ = true;
    }

    void restoreTerminal()
    {
        if (terminal_configured_) {
            ::tcsetattr(STDIN_FILENO, TCSANOW, &saved_termios_);
            terminal_configured_ = false;
        }
    }

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
};

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