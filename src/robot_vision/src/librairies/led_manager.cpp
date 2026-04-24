#include "robot_vision/librairies/led_manager.hpp"

#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

const char* ledStateToString(LedState s) noexcept
{
    switch (s) {
        case LedState::EMERGENCY_RESET: return "EMERGENCY_RESET";
        case LedState::ERROR:           return "ERROR";
        case LedState::SUCCESS:         return "SUCCESS";
        case LedState::DOCK_PENDING:    return "DOCK_PENDING";
        case LedState::DOCK_APPROACH:   return "DOCK_APPROACH";
        case LedState::UNDOCKING:       return "UNDOCKING";
        case LedState::HUMAN_DETECTED:  return "HUMAN_DETECTED";
        case LedState::LISTEN:          return "LISTEN";
        case LedState::FOLLOW:          return "FOLLOW";
        case LedState::TELEOP:          return "TELEOP";
        case LedState::OFF:             return "OFF";
    }
    return "UNKNOWN";
}

LedManager::LedManager(rclcpp::Node* parent_node, const std::string& topic)
    : node_(parent_node), start_time_(parent_node->now())
{
    pub_   = node_->create_publisher<LightringLeds>(topic, 10);
    timer_ = node_->create_wall_timer(100ms, [this]() { onTimer(); });
    setState(LedState::TELEOP);
}

LedManager::PaletteEntry LedManager::palette(LedState s) noexcept
{
    switch (s) {
        case LedState::EMERGENCY_RESET: return {255,   0,   0, true,  0.3};   // rouge clignotant rapide
        case LedState::ERROR:           return {255,   0,   0, false, 0.0};   // rouge fixe
        case LedState::SUCCESS:         return {  0, 255,   0, false, 0.0};   // vert fixe
        case LedState::DOCK_PENDING:    return {255, 200,   0, true,  0.5};   // jaune clignotant
        case LedState::DOCK_APPROACH:   return {255, 100,   0, true,  1.0};   // orange clignotant lent
        case LedState::UNDOCKING:       return {  0, 150, 255, true,  0.5};   // cyan clignotant
        case LedState::HUMAN_DETECTED:  return {  0, 255,   0, false, 0.0};   // vert fixe
        case LedState::LISTEN:          return {  0,  80, 255, true,  1.5};   // bleu pulsation lente
        case LedState::FOLLOW:          return {120,   0, 200, false, 0.0};   // violet fixe
        case LedState::TELEOP:          return { 60,  60,  60, false, 0.0};   // blanc tamisé
        case LedState::OFF:             return {  0,   0,   0, false, 0.0};
    }
    return {0, 0, 0, false, 0.0};
}

void LedManager::setState(LedState s)
{
    const auto idx = static_cast<uint8_t>(s);
    is_transient_[idx] = false;
    active_states_.fetch_or(static_cast<uint16_t>(1u << idx));
}

void LedManager::setStateTransient(LedState s, double duration_s)
{
    const auto idx = static_cast<uint8_t>(s);
    transient_end_[idx] = node_->now() + rclcpp::Duration::from_seconds(duration_s);
    is_transient_[idx]  = true;
    active_states_.fetch_or(static_cast<uint16_t>(1u << idx));
}

void LedManager::clearState(LedState s)
{
    const auto idx = static_cast<uint8_t>(s);
    is_transient_[idx] = false;
    active_states_.fetch_and(static_cast<uint16_t>(~(1u << idx)));
}

void LedManager::clearAll()
{
    active_states_.store(0);
    std::fill(is_transient_.begin(), is_transient_.end(), false);
    pub_->publish(makeMsg(0, 0, 0));
    last_published_state_ = LedState::OFF;
}

LedState LedManager::resolveActiveState() const
{
    const uint16_t mask = active_states_.load();
    if (mask == 0) { return LedState::OFF; }

    for (uint8_t i = 0; i <= static_cast<uint8_t>(LedState::OFF); ++i) {
        if (mask & (1u << i)) { return static_cast<LedState>(i); }
    }
    return LedState::OFF;
}

void LedManager::onTimer()
{
    const auto now = node_->now();
    for (uint8_t i = 0; i < 16; ++i) {
        if (is_transient_[i] && now >= transient_end_[i]) {
            is_transient_[i] = false;
            active_states_.fetch_and(static_cast<uint16_t>(~(1u << i)));
        }
    }

    publishCurrent();
}

void LedManager::publishCurrent()
{
    const LedState state = resolveActiveState();
    const auto entry = palette(state);

    bool publish_on = true;
    if (entry.blink) {
        const double t = (node_->now() - start_time_).seconds();
        const double phase = std::fmod(t, entry.blink_period_s) / entry.blink_period_s;
        publish_on = (phase < 0.5);
    }

    if (state == last_published_state_ && !entry.blink) { return; }

    if (publish_on) {
        pub_->publish(makeMsg(entry.r, entry.g, entry.b));
    } else {
        pub_->publish(makeMsg(0, 0, 0));
    }
    last_published_state_ = state;
    last_blink_phase_on_  = publish_on;
}

LedManager::LightringLeds LedManager::makeMsg(uint8_t r, uint8_t g, uint8_t b)
{
    LedColor color;
    color.red   = r;
    color.green = g;
    color.blue  = b;

    LightringLeds msg;
    msg.override_system = true;
    msg.leds.fill(color);
    return msg;
}