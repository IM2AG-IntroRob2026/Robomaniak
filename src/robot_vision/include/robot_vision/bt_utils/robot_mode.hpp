#pragma once

enum class RobotMode { TELEOP, FOLLOW, LISTEN };

inline const char* modeToString(const RobotMode m) noexcept
{
    switch (m) {
        case RobotMode::TELEOP: return "TELEOP";
        case RobotMode::FOLLOW: return "FOLLOW";
        case RobotMode::LISTEN: return "LISTEN";
    }
    return "UNKNOWN";
}

inline RobotMode nextMode(const RobotMode m) noexcept
{
    switch (m) {
        case RobotMode::TELEOP: return RobotMode::FOLLOW;
        case RobotMode::FOLLOW: return RobotMode::LISTEN;
        case RobotMode::LISTEN: return RobotMode::TELEOP;
    }
    return RobotMode::TELEOP;
}