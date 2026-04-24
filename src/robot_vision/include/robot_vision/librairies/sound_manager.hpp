#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <irobot_create_msgs/msg/audio_note_vector.hpp>

/**
 * @brief Manages audio feedback through the iRobot Create 3 native speaker (/cmd_audio).
 *
 * Sequences are loaded once from a YAML file and cached as pre-built AudioNoteVector
 * messages, so triggering a sequence only re-stamps the header and publishes.
 *
 * Expected YAML layout:
 * @code
   sequences:
     target_lock:
       - { frequency: 523.25, duration: 0.08 }
       - { frequency: 659.25, duration: 0.08 }
       - { frequency: 783.99, duration: 0.15 }
   @endcode
 *
 * `frequency` is in Hz and clamped to [MIN_FREQ_HZ, MAX_FREQ_HZ].
 * `duration` is in seconds (strictly positive).
 * A sequence can contain any number of notes.
 */
class SoundManager
{
public:
    using AudioNoteVector = irobot_create_msgs::msg::AudioNoteVector;

    /**
     * @brief Creates the publisher on @p topic.
     * @param parent_node Must outlive this SoundManager.
     * @param topic       Defaults to "/cmd_audio"; can be remapped from the launch file.
     */
    explicit SoundManager(rclcpp::Node* parent_node, const std::string& topic = "/cmd_audio");

    /**
     * @brief Loads every sequence defined in @p path into the internal cache.
     * Existing sequences with the same names are overwritten.
     * Non-fatal on error: logs a warning and returns false.
     * @return true if at least one sequence was successfully loaded.
     */
    bool loadFromFile(const std::filesystem::path& path);

    /**
     * @brief Publishes the cached sequence @p name on the audio topic.
     * The message is sent with append=false, so it overrides any currently playing sound.
     * No-op (with a warning) if the sequence is unknown.
     */
    void play(std::string_view name);

    /**
     * @brief Returns true if a sequence with this name is currently loaded.
     */
    [[nodiscard]] bool has(std::string_view name) const noexcept;

    /**
     * @brief Immediately silences the robot by publishing an empty sequence with append=false.
     */
    void stop();

private:
    rclcpp::Node* node_;
    rclcpp::Publisher<AudioNoteVector>::SharedPtr pub_;

    std::unordered_map<std::string, AudioNoteVector> sequences_;

    static constexpr double MIN_FREQ_HZ = 0.0;
    static constexpr double MAX_FREQ_HZ = 8000.0;
};