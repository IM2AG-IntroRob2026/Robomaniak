#include "robot_vision/bt_manager_node.hpp"

#include <algorithm>
#include <cmath>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <behaviortree_cpp/action_node.h>
#include <behaviortree_cpp/bt_factory.h>
#include <behaviortree_cpp/condition_node.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

using Twist = geometry_msgs::msg::Twist;

IsMode::IsMode(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx)
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

FollowAction::FollowAction(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx)
    : BT::StatefulActionNode(name, config), ctx_(std::move(ctx)) {}

BT::PortsList FollowAction::providedPorts() { return {}; }

BT::NodeStatus FollowAction::onStart()   { return BT::NodeStatus::RUNNING; }

BT::NodeStatus FollowAction::onRunning()
{
    std::lock_guard<std::mutex> lock(ctx_->cmd_mutex);
    if (ctx_->has_follow_cmd) { ctx_->cmd_vel_pub->publish(ctx_->follow_cmd); }
    return BT::NodeStatus::RUNNING;
}

void FollowAction::onHalted() { ctx_->cmd_vel_pub->publish(Twist{}); }

TeleopAction::TeleopAction(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx)
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

ListenAction::ListenAction(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<BtContext> ctx)
    : BT::StatefulActionNode(name, config), ctx_(std::move(ctx)) {}

BT::PortsList ListenAction::providedPorts() { return {}; }

BT::NodeStatus ListenAction::onStart()
{
    ctx_->cmd_vel_pub->publish(Twist{});
    active_impulse_.reset();

    std::lock_guard lock(ctx_->cmd_mutex);
    ctx_->pending_listen_cmd.reset();

    return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ListenAction::onRunning()
{
    const auto now = std::chrono::steady_clock::now();

    {
        std::lock_guard lock(ctx_->cmd_mutex);
        if (ctx_->pending_listen_cmd.has_value()) {
            const std::string cmd = std::move(*ctx_->pending_listen_cmd);
            ctx_->pending_listen_cmd.reset();

            Impulse impulse;
            impulse.cmd = commandToTwist(cmd);
            impulse.end_time = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(ctx_->impulse_duration_s));
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

void ListenAction::onHalted()
{
    active_impulse_.reset();
    ctx_->cmd_vel_pub->publish(Twist{});
}

Twist ListenAction::commandToTwist(const std::string& cmd) const
{
    Twist twist;
    if (cmd == "forward") { twist.linear.x = ctx_->impulse_linear_speed;  }
    else if (cmd == "left") { twist.angular.z = ctx_->impulse_angular_speed; }
    else if (cmd == "right") { twist.angular.z = -ctx_->impulse_angular_speed; }
    return twist;
}

BtManagerNode::BtManagerNode() : Node("bt_manager_node")
{
    declare_parameter<double>("bt_tick_hz", 50.0);
    declare_parameter<double>("impulse_duration_s", 1.5);
    declare_parameter<double>("impulse_linear_speed", 0.3);
    declare_parameter<double>("impulse_angular_speed", 0.8);
    declare_parameter<std::string>("dock_action",   "/dock");
    declare_parameter<std::string>("undock_action", "/undock");

    declare_parameter<double>("approach_coarse_tolerance_m", 0.8);
    declare_parameter<double>("approach_trigger_distance_m", 0.6);
    declare_parameter<double>("approach_trigger_lateral_m",  0.15);
    declare_parameter<double>("approach_max_linear_speed",   0.18);
    declare_parameter<double>("approach_max_angular_speed",  0.6);
    declare_parameter<double>("approach_kp_rot",             1.2);
    declare_parameter<double>("approach_kp_fwd",             0.45);
    declare_parameter<double>("approach_search_speed",       0.4);
    declare_parameter<double>("approach_coarse_timeout_s",   60.0);
    declare_parameter<double>("approach_fine_timeout_s",     30.0);
    declare_parameter<double>("approach_search_timeout_s",   25.0);
    declare_parameter<double>("approach_lost_timeout_s",     2.0);
    declare_parameter<double>("approach_tick_hz",            20.0);
    declare_parameter<double>("approach_heading_threshold",  0.15);

    declare_parameter<double>("camera_x_m",        0.0);
    declare_parameter<double>("camera_y_m",        0.0);
    declare_parameter<double>("camera_z_m",        0.10);
    declare_parameter<double>("camera_roll_deg",   0.0);
    declare_parameter<double>("camera_pitch_deg", -20.0);
    declare_parameter<double>("camera_yaw_deg",    0.0);

    declare_parameter<double>("request_debounce_s",     0.5);
    declare_parameter<double>("dock_pending_timeout_s", 120.0);

    const double tick_hz = get_parameter("bt_tick_hz").as_double();

    approach_coarse_tolerance_m_ = get_parameter("approach_coarse_tolerance_m").as_double();
    approach_trigger_distance_m_ = get_parameter("approach_trigger_distance_m").as_double();
    approach_trigger_lateral_m_  = get_parameter("approach_trigger_lateral_m").as_double();
    approach_max_linear_speed_   = get_parameter("approach_max_linear_speed").as_double();
    approach_max_angular_speed_  = get_parameter("approach_max_angular_speed").as_double();
    approach_kp_rot_             = get_parameter("approach_kp_rot").as_double();
    approach_kp_fwd_             = get_parameter("approach_kp_fwd").as_double();
    approach_search_speed_       = get_parameter("approach_search_speed").as_double();
    approach_coarse_timeout_s_   = get_parameter("approach_coarse_timeout_s").as_double();
    approach_fine_timeout_s_     = get_parameter("approach_fine_timeout_s").as_double();
    approach_search_timeout_s_   = get_parameter("approach_search_timeout_s").as_double();
    approach_lost_timeout_s_     = get_parameter("approach_lost_timeout_s").as_double();
    approach_tick_hz_            = get_parameter("approach_tick_hz").as_double();
    approach_heading_threshold_  = get_parameter("approach_heading_threshold").as_double();

    cam_x_m_       = get_parameter("camera_x_m").as_double();
    cam_y_m_       = get_parameter("camera_y_m").as_double();
    cam_z_m_       = get_parameter("camera_z_m").as_double();
    cam_roll_deg_  = get_parameter("camera_roll_deg").as_double();
    cam_pitch_deg_ = get_parameter("camera_pitch_deg").as_double();
    cam_yaw_deg_   = get_parameter("camera_yaw_deg").as_double();

    request_debounce_s_     = get_parameter("request_debounce_s").as_double();
    dock_pending_timeout_s_ = get_parameter("dock_pending_timeout_s").as_double();

    buildCameraTransform();

    RCLCPP_INFO(get_logger(),
        "Camera→base_link transform: pos=(%.2f, %.2f, %.2f) rpy=(%.1f, %.1f, %.1f) deg",
        cam_x_m_, cam_y_m_, cam_z_m_,
        cam_roll_deg_, cam_pitch_deg_, cam_yaw_deg_);

    ctx_ = std::make_shared<BtContext>();
    ctx_->cmd_vel_pub = create_publisher<Twist>("/cmd_vel", 10);

    led_mgr_ = std::make_unique<LedManager>(this);
    refreshModeLed();

    dock_client_   = rclcpp_action::create_client<DockAction>  (this, get_parameter("dock_action").as_string());
    undock_client_ = rclcpp_action::create_client<UndockAction>(this, get_parameter("undock_action").as_string());

    dock_sub_ = create_subscription<std_msgs::msg::Empty>("/teleop/dock", 10,
        [this](const std_msgs::msg::Empty::ConstSharedPtr&) { onDockRequest(); });

    undock_sub_ = create_subscription<std_msgs::msg::Empty>("/teleop/undock", 10,
        [this](const std_msgs::msg::Empty::ConstSharedPtr&) { onUndockRequest(); });

    ctx_->impulse_duration_s   = get_parameter("impulse_duration_s").as_double();
    ctx_->impulse_linear_speed = get_parameter("impulse_linear_speed").as_double();
    ctx_->impulse_angular_speed= get_parameter("impulse_angular_speed").as_double();

    follow_sub_ = create_subscription<Twist>(
        "/follow/cmd_vel", 10,
        [this](const Twist::ConstSharedPtr& msg) {
            std::lock_guard<std::mutex> lock(ctx_->cmd_mutex);
            ctx_->follow_cmd = *msg;
            ctx_->has_follow_cmd = true;
        });

    teleop_sub_ = create_subscription<Twist>(
        "/teleop/cmd_vel", 10,
        [this](const Twist::ConstSharedPtr& msg) {
            std::lock_guard<std::mutex> lock(ctx_->cmd_mutex);
            ctx_->teleop_cmd = *msg;
        });

    listen_cmd_sub_ = create_subscription<std_msgs::msg::String>(
        "/listen/command", 10,
        [this](const std_msgs::msg::String::ConstSharedPtr& msg) {
            if (ctx_->mode.load() != RobotMode::LISTEN) { return; }
            std::lock_guard<std::mutex> lock(ctx_->cmd_mutex);
            ctx_->pending_listen_cmd = msg->data;
        });

    cycle_switch_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/teleop/mode_switch", 10,
        [this](const std_msgs::msg::Empty::ConstSharedPtr&) { cycleMode(); });

    mode_request_sub_ = create_subscription<std_msgs::msg::String>(
        "/listen/mode_request", 10,
        std::bind(&BtManagerNode::onModeRequest, this, std::placeholders::_1));

    const rclcpp::QoS sensor_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/odom", sensor_qos,
        std::bind(&BtManagerNode::onOdom, this, std::placeholders::_1));

    dock_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
        "/dock/pose", 10,
        std::bind(&BtManagerNode::onDockPose, this, std::placeholders::_1));

    dock_detected_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/dock/detected", 10,
        std::bind(&BtManagerNode::onDockDetected, this, std::placeholders::_1));

    hazard_sub_ = create_subscription<HazardVec>(
        "/hazard_detection", sensor_qos,
        std::bind(&BtManagerNode::onHazard, this, std::placeholders::_1));

    emergency_reset_sub_ = create_subscription<std_msgs::msg::Empty>(
        "/teleop/emergency_reset", 10,
        [this](const std_msgs::msg::Empty::ConstSharedPtr&) { onEmergencyReset(); });

    human_present_sub_ = create_subscription<std_msgs::msg::Bool>(
        "/detection/human_present", 10,
        std::bind(&BtManagerNode::onHumanPresent, this, std::placeholders::_1));

    buildTree();

    const auto period = std::chrono::duration<double>(1.0 / tick_hz);
    bt_timer_ = this->create_wall_timer(period, std::bind(&BtManagerNode::tickBt, this));

    const auto approach_period = std::chrono::duration<double>(1.0 / approach_tick_hz_);
    approach_timer_ = this->create_wall_timer(approach_period, std::bind(&BtManagerNode::approachTick, this));

    RCLCPP_INFO(get_logger(), "BtManagerNode ready (BT @%.0fHz, approach @%.0fHz).", tick_hz, approach_tick_hz_);
}

BtManagerNode::~BtManagerNode()
{
    if (led_mgr_) { led_mgr_->clearAll(); }
    cancelActiveDockGoal();
    ctx_->cmd_vel_pub->publish(Twist{});
}

void BtManagerNode::cycleMode()
{
    if (ctx_->docking_active.load()) {
        switchToTeleopAndCancelEverything("user mode_switch");
        return;
    }

    RobotMode current = ctx_->mode.load();
    RobotMode next = nextMode(current);
    while (!ctx_->mode.compare_exchange_weak(current, next)) { next = nextMode(current); }
    RCLCPP_INFO(get_logger(), "--- Mode : %s -> %s ---", modeToString(current), modeToString(next));
    refreshModeLed();
}

void BtManagerNode::buildTree()
{
    auto ctx = ctx_;

    factory_.registerBuilder<IsMode>("IsMode",
        [ctx](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<IsMode>(name, config, ctx);
        });

    factory_.registerBuilder<FollowAction>("FollowAction",
        [ctx](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<FollowAction>(name, config, ctx);
        });

    factory_.registerBuilder<TeleopAction>("TeleopAction",
        [ctx](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<TeleopAction>(name, config, ctx);
        });

    factory_.registerBuilder<ListenAction>("ListenAction",
        [ctx](const std::string& name, const BT::NodeConfig& config) {
            return std::make_unique<ListenAction>(name, config, ctx);
        });

    tree_ = factory_.createTreeFromText(TREE_XML);
}

void BtManagerNode::tickBt()
{
    if (ctx_->docking_active.load()) { return; }
    tree_.tickOnce();
}

void BtManagerNode::onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr& msg)
{
    const auto& p = msg->pose.pose.position;
    const auto& q = msg->pose.pose.orientation;

    Pose2D pose;
    pose.x   = p.x;
    pose.y   = p.y;
    pose.yaw = yawFromQuaternion(q.x, q.y, q.z, q.w);

    std::lock_guard lock(odom_mutex_);
    current_odom_pose_ = pose;
}

void BtManagerNode::onDockPose(const geometry_msgs::msg::PoseStamped::ConstSharedPtr& msg)
{
    std::lock_guard lock(dock_pose_mutex_);
    latest_dock_pose_cam_      = *msg;
    last_dock_detection_time_  = this->now();
}

void BtManagerNode::onDockDetected(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    if (msg->data) {
        std::lock_guard lock(dock_pose_mutex_);
        last_dock_detection_time_ = this->now();
    }
}

void BtManagerNode::onDockRequest()
{
    const auto now = this->now();

    if (last_dock_request_time_.nanoseconds() != 0 && (now - last_dock_request_time_).seconds() < request_debounce_s_) {
        RCLCPP_WARN(get_logger(), "Dock request ignored: debounce (%.1fs).", request_debounce_s_);
        return;
    }
    last_dock_request_time_ = now;

    if (ctx_->docking_active.load()) {
        RCLCPP_WARN(get_logger(), "Dock request ignored: already docking.");
        return;
    }

    ctx_->mode_before_dock = ctx_->mode.load();
    ctx_->docking_active.store(true);
    ctx_->cmd_vel_pub->publish(Twist{});

    led_mgr_->setState(LedState::DOCK_APPROACH);

    RCLCPP_INFO(get_logger(), "--- Dock requested (mode saved: %s) ---", modeToString(ctx_->mode_before_dock));

    startApproach();
}

void BtManagerNode::onUndockRequest()
{
    const auto now = this->now();

    if (last_undock_request_time_.nanoseconds() != 0 && (now - last_undock_request_time_).seconds() < request_debounce_s_) {
        RCLCPP_WARN(get_logger(), "Undock request ignored: debounce (%.1fs).", request_debounce_s_);
        return;
    }
    last_undock_request_time_ = now;

    if (ctx_->docking_active.load()) {
        RCLCPP_WARN(get_logger(), "Undock request ignored: operation in progress.");
        return;
    }
    if (!undock_client_->action_server_is_ready()) {
        RCLCPP_ERROR(get_logger(), "Undock action server not ready.");
        led_mgr_->setStateTransient(LedState::ERROR, 2.0);
        return;
    }

    if (auto cur = currentOdomCopy(); cur.has_value()) {
        std::lock_guard lock(odom_mutex_);
        saved_dock_odom_pose_ = cur;
        RCLCPP_INFO(get_logger(),
            "Saved dock odom pose before undock: (%.2f, %.2f, %.2f rad)",
            cur->x, cur->y, cur->yaw);
    } else {
        RCLCPP_WARN(get_logger(), "No odometry yet; cannot save dock pose before undock.");
    }

    ctx_->mode_before_dock = ctx_->mode.load();
    ctx_->docking_active.store(true);
    ctx_->cmd_vel_pub->publish(Twist{});
    led_mgr_->setState(LedState::UNDOCKING);
    RCLCPP_INFO(get_logger(), "--- Undocking... (mode saved: %s) ---", modeToString(ctx_->mode_before_dock));

    auto opts = rclcpp_action::Client<UndockAction>::SendGoalOptions{};

    opts.goal_response_callback =
        [this](UndockGoalHandle::SharedPtr handle) {
            if (!handle) {
                RCLCPP_ERROR(get_logger(), "Undock goal REJECTED.");
                ctx_->docking_active.store(false);
                ctx_->mode.store(ctx_->mode_before_dock);
                led_mgr_->clearState(LedState::UNDOCKING);
                led_mgr_->setStateTransient(LedState::ERROR, 3.0);
                refreshModeLed();
                return;
            }
            std::lock_guard lock(active_undock_goal_mutex_);
            active_undock_goal_ = handle;
    };

    opts.result_callback = [this](const auto& result) {
        {
            std::lock_guard lock(active_undock_goal_mutex_);
            active_undock_goal_.reset();
        }
        ctx_->docking_active.store(false);
        ctx_->mode.store(ctx_->mode_before_dock);
        led_mgr_->clearState(LedState::UNDOCKING);
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
            RCLCPP_INFO(get_logger(), "--- Undock succeeded. Mode restored: %s ---", modeToString(ctx_->mode_before_dock));
            led_mgr_->setStateTransient(LedState::SUCCESS, 2.0);
        } else {
            RCLCPP_ERROR(get_logger(), "--- Undock FAILED. Mode restored: %s ---", modeToString(ctx_->mode_before_dock));
            led_mgr_->setStateTransient(LedState::ERROR, 3.0);
        }
        refreshModeLed();
    };

    undock_client_->async_send_goal(UndockAction::Goal{}, opts);
}

void BtManagerNode::startApproach()
{
    approach_overall_start_ = this->now();

    const bool recent_detection = dockDetectedRecently();
    std::optional<Pose2D> saved = savedDockPoseCopy();

    ApproachPhase initial;
    if (recent_detection) {
        initial = ApproachPhase::FINE;
        RCLCPP_INFO(get_logger(), "Approach start → FINE (dock already visible)");
    } else if (saved.has_value()) {
        initial = ApproachPhase::COARSE;
        RCLCPP_INFO(get_logger(), "Approach start → COARSE (target: %.2f, %.2f)", saved->x, saved->y);
    } else {
        initial = ApproachPhase::SEARCH;
        RCLCPP_INFO(get_logger(), "Approach start → SEARCH (no saved pose, no recent detection)");
    }
    transitionPhase(initial);
}

void BtManagerNode::transitionPhase(ApproachPhase new_phase)
{
    const auto prev = approach_phase_.exchange(new_phase);
    approach_phase_start_ = this->now();
    if (prev != new_phase) {
        RCLCPP_INFO(get_logger(), "Approach phase: %s → %s",
            approachPhaseToString(prev), approachPhaseToString(new_phase));
    }
}

void BtManagerNode::approachTick()
{
    if (!ctx_->docking_active.load()) { return; }
    const auto phase = approach_phase_.load();

    if (phase == ApproachPhase::DOCK_PENDING) {
        if (phaseElapsedSec() > dock_pending_timeout_s_) {
            RCLCPP_ERROR(get_logger(),
                "DOCK_PENDING timeout (%.1fs) → force abort + cancel goal",
                dock_pending_timeout_s_);
            cancelActiveDockGoal();
            finishApproach(false);
        }
        return;
    }

    if (phase == ApproachPhase::IDLE) { return; }

    switch (phase) {
        case ApproachPhase::COARSE: tickCoarse(); break;
        case ApproachPhase::FINE:   tickFine();   break;
        case ApproachPhase::SEARCH: tickSearch(); break;
        default: break;
    }
}

void BtManagerNode::tickCoarse()
{
    if (dockDetectedRecently()) { transitionPhase(ApproachPhase::FINE); return; }

    if (phaseElapsedSec() > approach_coarse_timeout_s_) {
        RCLCPP_WARN(get_logger(), "COARSE timeout (%.1fs) → fail", approach_coarse_timeout_s_);
        finishApproach(false);
        return;
    }

    auto cur = currentOdomCopy();
    auto dock = savedDockPoseCopy();
    if (!cur.has_value() || !dock.has_value()) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "COARSE: missing odom or saved dock pose → SEARCH");
        transitionPhase(ApproachPhase::SEARCH);
        return;
    }

    const double dx = dock->x - cur->x;
    const double dy = dock->y - cur->y;
    const double dist = std::hypot(dx, dy);

    if (dist < approach_coarse_tolerance_m_) {
        RCLCPP_INFO(get_logger(), "COARSE reached vicinity (dist=%.2fm) → SEARCH", dist);
        transitionPhase(ApproachPhase::SEARCH);
        return;
    }

    const double target_heading = std::atan2(dy, dx);
    const double heading_err    = angleDiff(target_heading, cur->yaw);

    Twist cmd;
    if (std::abs(heading_err) > approach_heading_threshold_) {
        cmd.angular.z = std::clamp(1.0 * heading_err,
            -approach_max_angular_speed_, approach_max_angular_speed_);
    } else {
        cmd.linear.x  = std::min(approach_max_linear_speed_, 0.3 * dist);
        cmd.angular.z = std::clamp(0.8 * heading_err,
            -approach_max_angular_speed_, approach_max_angular_speed_);
    }
    ctx_->cmd_vel_pub->publish(cmd);
}

void BtManagerNode::tickFine()
{
    const double since_detect = (this->now() - last_dock_detection_time_).seconds();
    if (since_detect > approach_lost_timeout_s_) {
        RCLCPP_WARN(get_logger(), "FINE: dock lost for %.1fs → SEARCH", since_detect);
        transitionPhase(ApproachPhase::SEARCH);
        return;
    }

    if (phaseElapsedSec() > approach_fine_timeout_s_) {
        RCLCPP_WARN(get_logger(), "FINE timeout (%.1fs) → fail", approach_fine_timeout_s_);
        finishApproach(false);
        return;
    }

    auto pose_opt = latestDockPoseCopy();
    if (!pose_opt.has_value()) { publishZeroCmd(); return; }

    const tf2::Vector3 p_base = cameraToBase(pose_opt->pose);
    const double forward = p_base.x();
    const double lateral = p_base.y();

    RCLCPP_DEBUG(get_logger(),
        "FINE: marker in base_link = (%.2f, %.2f, %.2f) | fwd=%.2f lat=%.2f",
        p_base.x(), p_base.y(), p_base.z(), forward, lateral);

    if (forward < approach_trigger_distance_m_ &&
        std::abs(lateral) < approach_trigger_lateral_m_) {
        RCLCPP_INFO(get_logger(),
            "FINE: in IR zone (fwd=%.2fm lat=%.2fm) → trigger native /dock",
            forward, lateral);
        publishZeroCmd();
        triggerDockAction();
        return;
    }

    Twist cmd;
    cmd.angular.z = std::clamp(approach_kp_rot_ * lateral,
        -approach_max_angular_speed_, approach_max_angular_speed_);

    if (std::abs(lateral) < 0.3) {
        const double fwd_error = forward - approach_trigger_distance_m_ * 0.8;
        cmd.linear.x = std::clamp(approach_kp_fwd_ * fwd_error,
            0.0, approach_max_linear_speed_);
    }
    ctx_->cmd_vel_pub->publish(cmd);
}

void BtManagerNode::tickSearch()
{
    if (dockDetectedRecently()) { transitionPhase(ApproachPhase::FINE); return; }

    if (phaseElapsedSec() > approach_search_timeout_s_) {
        RCLCPP_WARN(get_logger(), "SEARCH timeout (%.1fs) → fail", approach_search_timeout_s_);
        finishApproach(false);
        return;
    }

    Twist cmd;
    cmd.angular.z = approach_search_speed_;
    ctx_->cmd_vel_pub->publish(cmd);
}

void BtManagerNode::onHazard(const HazardVec::ConstSharedPtr& msg)
{
    using HazardType = irobot_create_msgs::msg::HazardDetection;

    const auto phase = approach_phase_.load();
    if (phase == ApproachPhase::IDLE) { return; }

    for (const auto& hazard : msg->detections) {
        if (hazard.type == HazardType::BUMP || hazard.type == HazardType::CLIFF) {
            const char* type_str = (hazard.type == HazardType::BUMP) ? "BUMP" : "CLIFF";
            RCLCPP_ERROR(get_logger(),
                "*** %s detected during approach (%s) -> ABORT DOCK ***",
                type_str, approachPhaseToString(phase));
            cancelActiveDockGoal();
            finishApproach(false);
            return;
        }
    }
}

void BtManagerNode::onEmergencyReset()
{
    RCLCPP_ERROR(get_logger(), "════════ EMERGENCY RESET REQUESTED ════════");
    resetAllState();
}

void BtManagerNode::onHumanPresent(const std_msgs::msg::Bool::ConstSharedPtr& msg)
{
    if (ctx_->mode.load() != RobotMode::FOLLOW) {
        led_mgr_->clearState(LedState::HUMAN_DETECTED);
        return;
    }
    if (msg->data) { led_mgr_->setState(LedState::HUMAN_DETECTED); }
    else { led_mgr_->clearState(LedState::HUMAN_DETECTED); }
}

void BtManagerNode::cancelActiveDockGoal()
{
    std::shared_ptr<DockGoalHandle> handle;
    {
        std::lock_guard lock(active_dock_goal_mutex_);
        handle = active_dock_goal_;
        active_dock_goal_.reset();
    }
    if (!handle) { return; }

    RCLCPP_WARN(get_logger(), "Cancelling active /dock goal...");
    try {
        dock_client_->async_cancel_goal(handle);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception while cancelling dock goal: %s", e.what());
    }
}

void BtManagerNode::resetAllState()
{
    // 1. Stoppe immédiatement tout mouvement
    publishZeroCmd();
    publishZeroCmd();

    // 2. Annule tout goal d'action en cours
    cancelActiveDockGoal();
    cancelActiveUndockGoal();

    // 3. Reset machine d'état d'approche
    approach_phase_.store(ApproachPhase::IDLE);

    // 4. Vide les commandes en attente
    {
        std::lock_guard lock(ctx_->cmd_mutex);
        ctx_->follow_cmd     = Twist{};
        ctx_->teleop_cmd     = Twist{};
        ctx_->has_follow_cmd = false;
        ctx_->pending_listen_cmd.reset();
    }

    // 5. Force état "neutre" : TELEOP, pas de docking
    ctx_->docking_active.store(false);
    ctx_->mode.store(RobotMode::TELEOP);
    ctx_->mode_before_dock = RobotMode::TELEOP;

    // 6. Feedback visuel : rouge clignotant pendant 2s, puis retour TELEOP
    led_mgr_->clearAll();
    led_mgr_->setStateTransient(LedState::EMERGENCY_RESET, 2.0);
    led_mgr_->setState(LedState::TELEOP);

    // 7. Republie un Twist zéro pour vider d'éventuels caches DDS
    publishZeroCmd();

    RCLCPP_WARN(get_logger(), "RESET COMPLETE: docking_active=false mode=TELEOP approach=IDLE");
}

void BtManagerNode::refreshModeLed()
{
    led_mgr_->clearState(LedState::TELEOP);
    led_mgr_->clearState(LedState::FOLLOW);
    led_mgr_->clearState(LedState::LISTEN);

    if (ctx_->mode.load() != RobotMode::FOLLOW) {
        led_mgr_->clearState(LedState::HUMAN_DETECTED);
    }

    switch (ctx_->mode.load()) {
        case RobotMode::TELEOP: led_mgr_->setState(LedState::TELEOP); break;
        case RobotMode::FOLLOW: led_mgr_->setState(LedState::FOLLOW); break;
        case RobotMode::LISTEN: led_mgr_->setState(LedState::LISTEN); break;
    }
}

void BtManagerNode::onModeRequest(const std_msgs::msg::String::ConstSharedPtr& msg)
{
    RobotMode requested;
    if (msg->data == "TELEOP")      { requested = RobotMode::TELEOP; }
    else if (msg->data == "FOLLOW") { requested = RobotMode::FOLLOW; }
    else if (msg->data == "LISTEN") { requested = RobotMode::LISTEN; }
    else {
        RCLCPP_WARN(get_logger(), "Unknown mode request '%s'", msg->data.c_str());
        return;
    }

    if (ctx_->docking_active.load()) {
        RCLCPP_WARN(get_logger(),
            "Mode request '%s' during docking → forcing TELEOP and aborting.",
            msg->data.c_str());
        switchToTeleopAndCancelEverything("voice mode_request");
        return;
    }

    const RobotMode prev = ctx_->mode.exchange(requested);
    if (prev != requested) {
        RCLCPP_INFO(get_logger(), "--- Voice mode change: %s → %s ---",
            modeToString(prev), modeToString(requested));
        refreshModeLed();
    }
}

void BtManagerNode::cancelActiveUndockGoal()
{
    std::shared_ptr<UndockGoalHandle> handle;
    {
        std::lock_guard lock(active_undock_goal_mutex_);
        handle = active_undock_goal_;
        active_undock_goal_.reset();
    }
    if (!handle) { return; }
    RCLCPP_WARN(get_logger(), "Cancelling active /undock goal...");
    try {
        undock_client_->async_cancel_goal(handle);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_logger(), "Exception cancelling undock: %s", e.what());
    }
}

void BtManagerNode::switchToTeleopAndCancelEverything(const char* reason)
{
    RCLCPP_WARN(get_logger(), "Stop & take control: %s", reason);

    // 1. Annule toutes les actions en cours
    cancelActiveDockGoal();
    cancelActiveUndockGoal();

    // 2. Reset machine d'état d'approche
    approach_phase_.store(ApproachPhase::IDLE);

    // 3. Stoppe le robot
    publishZeroCmd();
    publishZeroCmd();

    // 4. Force état neutre TELEOP
    ctx_->docking_active.store(false);
    ctx_->mode.store(RobotMode::TELEOP);
    ctx_->mode_before_dock = RobotMode::TELEOP;

    // 5. LED feedback
    led_mgr_->clearState(LedState::DOCK_APPROACH);
    led_mgr_->clearState(LedState::DOCK_PENDING);
    led_mgr_->clearState(LedState::UNDOCKING);
    led_mgr_->setStateTransient(LedState::ERROR, 1.0);
    refreshModeLed();
}

void BtManagerNode::triggerDockAction()
{
    if (!dock_client_->action_server_is_ready()) {
        RCLCPP_ERROR(get_logger(), "Dock action server not ready.");
        finishApproach(false);
        return;
    }

    transitionPhase(ApproachPhase::DOCK_PENDING);
    led_mgr_->clearState(LedState::DOCK_APPROACH);
    led_mgr_->setState(LedState::DOCK_PENDING);

    auto opts = rclcpp_action::Client<DockAction>::SendGoalOptions{};

    opts.goal_response_callback =
        [this](DockGoalHandle::SharedPtr handle) {
            if (!handle) {
                RCLCPP_ERROR(get_logger(), "Dock goal REJECTED by server!");
                finishApproach(false);
                return;
            }
            RCLCPP_INFO(get_logger(), "Dock goal accepted by server.");
            std::lock_guard lock(active_dock_goal_mutex_);
            active_dock_goal_ = handle;
        };

    opts.result_callback = [this](const auto& result) {
        {
            std::lock_guard lock(active_dock_goal_mutex_);
            active_dock_goal_.reset();
        }

        const bool ok = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
        if (ok) {
            if (auto cur = currentOdomCopy(); cur.has_value()) {
                std::lock_guard lock(odom_mutex_);
                saved_dock_odom_pose_ = cur;
                RCLCPP_INFO(get_logger(),
                    "Updated saved dock odom pose after successful dock: (%.2f, %.2f)",
                    cur->x, cur->y);
            }
        } else {
            auto code_str = "UNKNOWN";
            switch (result.code) {
                case rclcpp_action::ResultCode::ABORTED:  code_str = "ABORTED"; break;
                case rclcpp_action::ResultCode::CANCELED: code_str = "CANCELED"; break;
                default: break;
            }
            RCLCPP_ERROR(get_logger(), "Dock action did not succeed (code: %s)", code_str);
        }
        finishApproach(ok);
    };
    dock_client_->async_send_goal(DockAction::Goal{}, opts);
}

void BtManagerNode::finishApproach(bool success)
{
    approach_phase_.store(ApproachPhase::IDLE);
    publishZeroCmd();

    led_mgr_->clearState(LedState::DOCK_APPROACH);
    led_mgr_->clearState(LedState::DOCK_PENDING);

    ctx_->docking_active.store(false);
    ctx_->mode.store(ctx_->mode_before_dock);

    if (success) {
        RCLCPP_INFO(get_logger(), "--- Dock SUCCESS. Mode restored: %s ---", modeToString(ctx_->mode_before_dock));
        led_mgr_->setStateTransient(LedState::SUCCESS, 2.0);
    } else {
        RCLCPP_ERROR(get_logger(), "--- Dock FAILED. Mode restored: %s ---", modeToString(ctx_->mode_before_dock));
        led_mgr_->setStateTransient(LedState::ERROR, 3.0);
    }

    refreshModeLed();
    cancelActiveDockGoal();
}

void BtManagerNode::abortApproach(const char* reason)
{
    RCLCPP_WARN(get_logger(), "Approach aborted: %s", reason);
    finishApproach(false);
}

bool BtManagerNode::dockDetectedRecently() const
{
    std::lock_guard lock(const_cast<std::mutex&>(dock_pose_mutex_));
    if (last_dock_detection_time_.nanoseconds() == 0) { return false; }
    return (this->now() - last_dock_detection_time_).seconds() < approach_lost_timeout_s_;
}

double BtManagerNode::phaseElapsedSec() const
{
    if (approach_phase_start_.nanoseconds() == 0) { return 0.0; }
    return (this->now() - approach_phase_start_).seconds();
}

std::optional<Pose2D> BtManagerNode::currentOdomCopy() const
{
    std::lock_guard lock(const_cast<std::mutex&>(odom_mutex_));
    return current_odom_pose_;
}

std::optional<Pose2D> BtManagerNode::savedDockPoseCopy() const
{
    std::lock_guard lock(const_cast<std::mutex&>(odom_mutex_));
    return saved_dock_odom_pose_;
}

std::optional<geometry_msgs::msg::PoseStamped> BtManagerNode::latestDockPoseCopy() const
{
    std::lock_guard lock(const_cast<std::mutex&>(dock_pose_mutex_));
    return latest_dock_pose_cam_;
}

void BtManagerNode::publishZeroCmd()
{
    ctx_->cmd_vel_pub->publish(Twist{});
}

void BtManagerNode::buildCameraTransform()
{
    tf2::Matrix3x3 R_optical_to_rosframe(
         0.0,  0.0,  1.0,
        -1.0,  0.0,  0.0,
         0.0, -1.0,  0.0);

    constexpr double DEG_TO_RAD = M_PI / 180.0;
    tf2::Quaternion q_mount;
    q_mount.setRPY(
        cam_roll_deg_  * DEG_TO_RAD,
        cam_pitch_deg_ * DEG_TO_RAD,
        cam_yaw_deg_   * DEG_TO_RAD);
    tf2::Matrix3x3 R_mount(q_mount);

    tf2::Matrix3x3 R_total = R_mount * R_optical_to_rosframe;

    tf2::Quaternion q_total;
    R_total.getRotation(q_total);

    tf_cam_to_base_.setOrigin(tf2::Vector3(cam_x_m_, cam_y_m_, cam_z_m_));
    tf_cam_to_base_.setRotation(q_total);
}

tf2::Vector3 BtManagerNode::cameraToBase(const geometry_msgs::msg::Pose& pose_cam) const
{
    const tf2::Vector3 p_cam(
        pose_cam.position.x,
        pose_cam.position.y,
        pose_cam.position.z);

    return tf_cam_to_base_ * p_cam;
}

double BtManagerNode::yawFromQuaternion(double x, double y, double z, double w) noexcept
{
    return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double BtManagerNode::angleDiff(double a, double b) noexcept
{
    double d = a - b;
    while (d >  M_PI) { d -= 2.0 * M_PI; }
    while (d < -M_PI) { d += 2.0 * M_PI; }
    return d;
}

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