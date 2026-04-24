#include "robot_vision/librairies/sound_manager.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <utility>

#include <yaml-cpp/yaml.h>

using AudioNote = irobot_create_msgs::msg::AudioNote;

SoundManager::SoundManager(rclcpp::Node* parent_node, const std::string& topic) : node_(parent_node)
{
    pub_ = node_->create_publisher<AudioNoteVector>(topic, 10);
}

bool SoundManager::loadFromFile(const std::filesystem::path& path)
{
    if (!std::filesystem::exists(path)) {
        RCLCPP_WARN(node_->get_logger(),
            "SoundManager: config file not found '%s', no sequences loaded.",
            path.string().c_str());
        return false;
    }

    YAML::Node root;
    try {
        root = YAML::LoadFile(path.string());
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(),
            "SoundManager: failed to parse '%s': %s",
            path.string().c_str(), e.what());
        return false;
    }

    const auto& seqs = root["sequences"];
    if (!seqs || !seqs.IsMap()) {
        RCLCPP_WARN(node_->get_logger(),
            "SoundManager: no 'sequences' map found in '%s'.",
            path.string().c_str());
        return false;
    }

    std::size_t loaded = 0;
    for (const auto& kv : seqs) {
        const auto name  = kv.first.as<std::string>();
        const auto& notes = kv.second;

        if (!notes.IsSequence()) {
            RCLCPP_WARN(node_->get_logger(), "SoundManager: sequence '%s' is not a list, skipped.", name.c_str());
            continue;
        }

        AudioNoteVector msg;
        msg.append = false;
        msg.notes.reserve(notes.size());

        for (const auto& n : notes) {
            double freq_hz    = 0.0;
            double duration_s = 0.0;
            try {
                freq_hz    = n["frequency"].as<double>();
                duration_s = n["duration"].as<double>();
            } catch (const std::exception& e) {
                RCLCPP_WARN(node_->get_logger(),
                    "SoundManager: bad note in sequence '%s' (%s), skipped.",
                    name.c_str(), e.what());
                continue;
            }

            if (duration_s <= 0.0) {
                RCLCPP_WARN(node_->get_logger(),
                    "SoundManager: non-positive duration in '%s', note skipped.",
                    name.c_str());
                continue;
            }

            const double freq_clamped = std::clamp(freq_hz, MIN_FREQ_HZ, MAX_FREQ_HZ);
            if (std::abs(freq_clamped - freq_hz) > 1e-3) {
                RCLCPP_WARN(node_->get_logger(),
                    "SoundManager: frequency %.2f Hz in '%s' out of range [%g, %g], clamped to %.2f.",
                    freq_hz, name.c_str(), MIN_FREQ_HZ, MAX_FREQ_HZ, freq_clamped);
            }

            AudioNote note;
            note.frequency = static_cast<uint16_t>(std::lround(freq_clamped));

            const double secs_whole = std::floor(duration_s);
            note.max_runtime.sec     = static_cast<int32_t>(secs_whole);
            note.max_runtime.nanosec = static_cast<uint32_t>(std::lround((duration_s - secs_whole) * 1e9));

            msg.notes.push_back(note);
        }

        if (msg.notes.empty()) {
            RCLCPP_WARN(node_->get_logger(), "SoundManager: sequence '%s' has no valid notes, skipped.", name.c_str());
            continue;
        }

        sequences_.insert_or_assign(std::move(name), std::move(msg));
        ++loaded;
    }

    RCLCPP_INFO(node_->get_logger(),
        "SoundManager: loaded %zu sequence(s) from '%s'.",
        loaded, path.string().c_str());
    return loaded > 0;
}

void SoundManager::play(std::string_view name)
{
    const auto it = sequences_.find(std::string(name));
    if (it == sequences_.end()) {
        RCLCPP_WARN(node_->get_logger(),
            "SoundManager: sequence '%.*s' not loaded, nothing to play.",
            static_cast<int>(name.size()), name.data());
        return;
    }

    AudioNoteVector msg = it->second;
    msg.header.stamp = node_->now();
    pub_->publish(msg);
}

[[nodiscard]] bool SoundManager::has(std::string_view name) const noexcept
{
    return sequences_.find(std::string(name)) != sequences_.end();
}

void SoundManager::stop()
{
    AudioNoteVector msg;
    msg.append = false;
    msg.header.stamp = node_->now();
    pub_->publish(msg);
}