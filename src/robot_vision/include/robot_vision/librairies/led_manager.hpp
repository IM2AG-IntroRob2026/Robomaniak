#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <irobot_create_msgs/msg/lightring_leds.hpp>
#include <irobot_create_msgs/msg/led_color.hpp>

enum class LedState : uint8_t
{
    EMERGENCY_RESET = 0,   ///< Reset d'urgence en cours - rouge clignotant rapide
    ERROR           = 1,   ///< Dernière action a échoué - rouge fixe (transitoire)
    SUCCESS         = 2,   ///< Dernière action a réussi - vert fixe (transitoire)
    DOCK_PENDING    = 3,   ///< /dock natif en cours - jaune clignotant
    DOCK_APPROACH   = 4,   ///< Phase d'approche du dock - orange clignotant lent
    UNDOCKING       = 5,   ///< /undock natif en cours - cyan clignotant
    HUMAN_DETECTED  = 6,   ///< Humain visible (mode FOLLOW seulement) - vert fixe
    LISTEN          = 7,   ///< Mode LISTEN, écoute active - bleu clignotant
    FOLLOW          = 8,   ///< Mode FOLLOW, pas d'humain visible - violet fixe
    TELEOP          = 9,   ///< Mode TELEOP, repos - blanc tamisé fixe
    OFF             = 10,  ///< LEDs éteintes - tout noir
};

const char* ledStateToString(LedState s) noexcept;

class LedManager
{
public:
    using LightringLeds = irobot_create_msgs::msg::LightringLeds;
    using LedColor      = irobot_create_msgs::msg::LedColor;

    explicit LedManager(rclcpp::Node* parent_node, const std::string& topic = "/cmd_lightring");

    void setState(LedState s);
    void setStateTransient(LedState s, double duration_s);
    void clearState(LedState s);
    void clearAll();

private:
    struct PaletteEntry
    {
        uint8_t r, g, b;
        bool    blink;
        double  blink_period_s;
    };

    static PaletteEntry palette(LedState s) noexcept;

    void onTimer();
    void publishCurrent();
    [[nodiscard]] LedState resolveActiveState() const;
    [[nodiscard]] static LightringLeds makeMsg(uint8_t r, uint8_t g, uint8_t b);

    rclcpp::Node* node_;
    rclcpp::Publisher<LightringLeds>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::atomic<uint16_t> active_states_{0};

    std::array<rclcpp::Time, 16> transient_end_;
    std::array<bool, 16>         is_transient_{};

    rclcpp::Time start_time_;
    LedState     last_published_state_{LedState::OFF};
    bool         last_blink_phase_on_{true};
};