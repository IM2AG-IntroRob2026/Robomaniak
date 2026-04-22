#pragma once

#include <atomic>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

#include <portaudio.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

struct SherpaOnnxKeywordSpotter;
struct SherpaOnnxOnlineStream;

class ListenNode : public rclcpp::Node
{
public:
    ListenNode();
    ~ListenNode() override;

private:
    int         sample_rate_     {16000};
    int         pa_sample_rate_  {16000};
    int         pa_device_index_ {-1};
    std::string encoder_path_;
    std::string decoder_path_;
    std::string joiner_path_;
    std::string tokens_path_;
    float       keyword_score_   {2.5f};
    float       score_threshold_ {0.01f};

    std::string kw_forward_ {"forward"};
    std::string kw_left_    {"left"};
    std::string kw_right_   {"right"};

    std::string kw_mode_teleop_  {"teleop"};
    std::string kw_mode_follow_  {"follow"};
    std::string kw_mode_listen_  {"listen"};

    std::string kw_dock_   {"dock"};
    std::string kw_undock_ {"undock"};

    std::string kw_forward_tokens_;
    std::string kw_left_tokens_;
    std::string kw_right_tokens_;

    std::string kw_mode_teleop_tokens_;
    std::string kw_mode_follow_tokens_;
    std::string kw_mode_listen_tokens_;

    std::string kw_dock_tokens_;
    std::string kw_undock_tokens_;

    std::unordered_set<std::string> vocab_;

    const SherpaOnnxKeywordSpotter* spotter_{nullptr};
    const SherpaOnnxOnlineStream*   stream_ {nullptr};

    PaStream* pa_stream_{nullptr};

    std::mutex        audio_mutex_;
    std::deque<float> audio_buffer_;
    static constexpr std::size_t AUDIO_BUFFER_MAX_SAMPLES = 16000 * 5;

    struct PendingPublish {
        enum class Kind { Command, ModeRequest, Dock, Undock, } kind;
        std::string data;
    };
    std::mutex                  publish_mutex_;
    std::vector<PendingPublish> publish_queue_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_request_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr  dock_pub_;
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr  undock_pub_;
    rclcpp::TimerBase::SharedPtr                        publish_timer_;

    std::atomic<bool> running_{true};
    std::thread       process_thread_;

    [[nodiscard]] bool initPortAudio();
    [[nodiscard]] bool initSherpa();
    void processLoop();
    void onPublishTimer();
    void handleKeyword(const std::string& keyword);
    void shutdownPortAudio();
    void shutdownSherpa();

    [[nodiscard]] bool loadVocabulary();
    [[nodiscard]] std::string buildKeywordsBuf() const;
    [[nodiscard]] std::string wordToTokens(const std::string& word, const std::string& tokens_override) const;

    static int paCallback(const void* input, void* output,
                          unsigned long frames_per_buffer,
                          const PaStreamCallbackTimeInfo* time_info,
                          PaStreamCallbackFlags status_flags,
                          void* user_data);
};