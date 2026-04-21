#include "robot_vision/listen_node.hpp"

#include <sherpa-onnx/c-api/c-api.h>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

ListenNode::ListenNode() : Node("listen_node")
{
    declare_parameter<int>        ("sample_rate",        16000);
    declare_parameter<int>        ("pa_device_index",    -1);
    declare_parameter<std::string>("encoder_path",       "");
    declare_parameter<std::string>("decoder_path",       "");
    declare_parameter<std::string>("joiner_path",        "");
    declare_parameter<std::string>("tokens_path",        "");
    declare_parameter<double>     ("keyword_score",      2.5);
    declare_parameter<double>     ("score_threshold",    0.01);
    declare_parameter<std::string>("kw_forward",         "forward");
    declare_parameter<std::string>("kw_left",            "left");
    declare_parameter<std::string>("kw_right",           "right");
    declare_parameter<std::string>("kw_forward_tokens",  "");
    declare_parameter<std::string>("kw_left_tokens",     "");
    declare_parameter<std::string>("kw_right_tokens",    "");
    declare_parameter<std::string>("kw_mode_teleop",     "teleop");
    declare_parameter<std::string>("kw_mode_follow",     "follow");
    declare_parameter<std::string>("kw_mode_listen",     "listen");
    declare_parameter<std::string>("kw_dock",            "dock");
    declare_parameter<std::string>("kw_undock",          "undock");
    declare_parameter<std::string>("kw_mode_teleop_tokens",  "");
    declare_parameter<std::string>("kw_mode_follow_tokens",  "");
    declare_parameter<std::string>("kw_mode_listen_tokens",  "");
    declare_parameter<std::string>("kw_dock_tokens",         "");
    declare_parameter<std::string>("kw_undock_tokens",       "");

    sample_rate_            = get_parameter("sample_rate").as_int();
    pa_device_index_        = get_parameter("pa_device_index").as_int();
    encoder_path_           = get_parameter("encoder_path").as_string();
    decoder_path_           = get_parameter("decoder_path").as_string();
    joiner_path_            = get_parameter("joiner_path").as_string();
    tokens_path_            = get_parameter("tokens_path").as_string();
    keyword_score_          = static_cast<float>(get_parameter("keyword_score").as_double());
    score_threshold_        = static_cast<float>(get_parameter("score_threshold").as_double());
    kw_forward_             = get_parameter("kw_forward").as_string();
    kw_left_                = get_parameter("kw_left").as_string();
    kw_right_               = get_parameter("kw_right").as_string();
    kw_forward_tokens_      = get_parameter("kw_forward_tokens").as_string();
    kw_left_tokens_         = get_parameter("kw_left_tokens").as_string();
    kw_right_tokens_        = get_parameter("kw_right_tokens").as_string();
    kw_mode_teleop_         = get_parameter("kw_mode_teleop").as_string();
    kw_mode_follow_         = get_parameter("kw_mode_follow").as_string();
    kw_mode_listen_         = get_parameter("kw_mode_listen").as_string();
    kw_dock_                = get_parameter("kw_dock").as_string();
    kw_undock_              = get_parameter("kw_undock").as_string();
    kw_mode_teleop_tokens_  = get_parameter("kw_mode_teleop_tokens").as_string();
    kw_mode_follow_tokens_  = get_parameter("kw_mode_follow_tokens").as_string();
    kw_mode_listen_tokens_  = get_parameter("kw_mode_listen_tokens").as_string();
    kw_dock_tokens_         = get_parameter("kw_dock_tokens").as_string();
    kw_undock_tokens_       = get_parameter("kw_undock_tokens").as_string();

    command_pub_      = create_publisher<std_msgs::msg::String>("/listen/command",      10);
    mode_request_pub_ = create_publisher<std_msgs::msg::String>("/listen/mode_request", 10);
    dock_pub_         = create_publisher<std_msgs::msg::Empty> ("/teleop/dock",         10);
    undock_pub_       = create_publisher<std_msgs::msg::Empty> ("/teleop/undock",       10);
    publish_timer_    = create_wall_timer(20ms, std::bind(&ListenNode::onPublishTimer,  this));

    if (!initSherpa()) {
        throw std::runtime_error("Sherpa-ONNX initialization failed. Check parameters and model files.");
    }
    if (!initPortAudio()) {
        RCLCPP_ERROR(get_logger(), "PortAudio initialization failed. Check audio device and parameters.");
         throw std::runtime_error("PortAudio initialization failed.");
    }

    process_thread_ = std::thread(&ListenNode::processLoop, this);
}

ListenNode::~ListenNode()
{
    running_ = false;
    if (process_thread_.joinable()) { process_thread_.join(); }
    shutdownPortAudio();
    shutdownSherpa();
}

bool ListenNode::loadVocabulary()
{
    std::ifstream file(tokens_path_);
    if (!file.is_open()) {
        RCLCPP_FATAL(get_logger(), "Failed to open tokens file: '%s'", tokens_path_.c_str());
        return false;
    }

    std::string line;
    while (std::getline(file, line)) {
        const auto sep = line.rfind(' ');
        if (sep == std::string::npos) { continue; }
        vocab_.insert(line.substr(0, sep));
    }

    RCLCPP_INFO(get_logger(), "Vocabulary loaded from '%s' - %zu tokens.", tokens_path_.c_str(), vocab_.size());
    return true;
}

std::string ListenNode::wordToTokens(const std::string& word, const std::string& tokens_override) const
{
    if (!tokens_override.empty()) { return tokens_override; }
    if (word.empty()) { return ""; }

    std::string upper;
    upper.reserve(word.size());
    for (unsigned char c : word) { upper += static_cast<char>(std::toupper(c)); }

    static constexpr std::string_view WORD_BOUNDARY{"\xe2\x96\x81"};

    std::vector<std::string> result;
    std::size_t pos = 0;
    bool first = true;

    // Longest-match tokenization
    while (pos < upper.size()) {
        std::string best;
        std::size_t best_len = 0;

        for (std::size_t len = upper.size() - pos; len >= 1; --len) {
            std::string sub = upper.substr(pos, len);
            std::string candidate = first ? (std::string{WORD_BOUNDARY} + sub) : sub;

            if (vocab_.count(candidate)) {
                best = std::move(candidate);
                best_len = len;
                break;
            }
        }

        if (best_len == 0) {
            best = first ? (std::string{WORD_BOUNDARY} + upper[pos]) : std::string(1, upper[pos]);
            best_len = 1;
            RCLCPP_WARN(get_logger(),
                "Tokenization fallback for '%s' at position %zu: '%s' not in vocab.",
                word.c_str(), pos, best.c_str());
        }

        result.push_back(std::move(best));
        pos += best_len;
        first = false;
    }

    std::ostringstream oss;
    for (std::size_t i = 0; i < result.size(); ++i) {
        if (i > 0) { oss << ' '; }
        oss << result[i];
    }
    return oss.str();
}

std::string ListenNode::buildKeywordsBuf() const
{
    std::ostringstream oss;
    oss << wordToTokens(kw_forward_,      kw_forward_tokens_)       << '\n'
        << wordToTokens(kw_left_,         kw_left_tokens_)          << '\n'
        << wordToTokens(kw_right_,        kw_right_tokens_)         << '\n'
        << wordToTokens(kw_mode_teleop_,  kw_mode_teleop_tokens_)   << '\n'
        << wordToTokens(kw_mode_follow_,  kw_mode_follow_tokens_)   << '\n'
        << wordToTokens(kw_mode_listen_,  kw_mode_listen_tokens_)   << '\n'
        << wordToTokens(kw_dock_,         kw_dock_tokens_)          << '\n'
        << wordToTokens(kw_undock_,       kw_undock_tokens_)        << '\n';
    return oss.str();
}

bool ListenNode::initSherpa()
{
    if (encoder_path_.empty() || decoder_path_.empty() || joiner_path_.empty() || tokens_path_.empty())
    {
        RCLCPP_FATAL(get_logger(),
            "Model paths must be set via parameters. Current values:\n"
            "  encoder_path: '%s'\n"
            "  decoder_path: '%s'\n"
            "  joiner_path:  '%s'\n"
            "  tokens_path:  '%s'",
            encoder_path_.c_str(), decoder_path_.c_str(),
            joiner_path_.c_str(),  tokens_path_.c_str());
        return false;
    }

    if (!loadVocabulary()) { return false; }

    const std::string keywords_buf = buildKeywordsBuf();
    RCLCPP_DEBUG(get_logger(), "keywords_buf :\n%s", keywords_buf.c_str());

    SherpaOnnxKeywordSpotterConfig config{};

    config.feat_config.sample_rate = sample_rate_;
    config.feat_config.feature_dim = 80;

    config.model_config.transducer.encoder = encoder_path_.c_str();
    config.model_config.transducer.decoder = decoder_path_.c_str();
    config.model_config.transducer.joiner  = joiner_path_.c_str();
    config.model_config.tokens             = tokens_path_.c_str();
    config.model_config.num_threads        = 2;
    config.model_config.provider           = "cpu";
    config.model_config.debug              = 0;

    config.max_active_paths    = 4;
    config.num_trailing_blanks = 1;
    config.keywords_score       = keyword_score_;
    config.keywords_threshold   = score_threshold_;
    config.keywords_buf        = keywords_buf.c_str();
    config.keywords_buf_size   = static_cast<int>(keywords_buf.size());

    spotter_ = SherpaOnnxCreateKeywordSpotter(&config);
    if (!spotter_) {
        RCLCPP_FATAL(get_logger(), "SherpaOnnxCreateKeywordSpotter() failed.");
        return false;
    }

    stream_ = SherpaOnnxCreateKeywordStream(spotter_);
    if (!stream_) {
        RCLCPP_FATAL(get_logger(), "SherpaOnnxCreateKeywordStream() failed.");
        SherpaOnnxDestroyKeywordSpotter(spotter_);
        spotter_ = nullptr;
        return false;
    }

    RCLCPP_INFO(get_logger(), "Sherpa-ONNX initialisé.");
    return true;
}

void ListenNode::shutdownSherpa()
{
    if (stream_) { SherpaOnnxDestroyOnlineStream(stream_); stream_ = nullptr; }
    if (spotter_) { SherpaOnnxDestroyKeywordSpotter(spotter_); spotter_ = nullptr; }
}

bool ListenNode::initPortAudio()
{
    PaError err = Pa_Initialize();
    if (err != paNoError) {
        RCLCPP_ERROR(get_logger(), "Pa_Initialize() : %s", Pa_GetErrorText(err));
        return false;
    }

    const PaDeviceIndex device = (pa_device_index_ < 0) ? Pa_GetDefaultInputDevice() : static_cast<PaDeviceIndex>(pa_device_index_);
    if (device == paNoDevice) {
        RCLCPP_ERROR(get_logger(), "No audio input device found. Check your audio configuration.");
        Pa_Terminate();
        return false;
    }

    RCLCPP_INFO(get_logger(), "Using audio input device #%d: %s", device, Pa_GetDeviceInfo(device)->name);

    PaStreamParameters params{};
    params.device                    = device;
    params.channelCount              = 1;
    params.sampleFormat              = paFloat32;
    params.suggestedLatency          = Pa_GetDeviceInfo(device)->defaultLowInputLatency;
    params.hostApiSpecificStreamInfo = nullptr;

    const int native_rate = static_cast<int>(Pa_GetDeviceInfo(device)->defaultSampleRate);
    const std::array<int, 3> rates_to_try = {sample_rate_, 44100, native_rate};

    constexpr unsigned long FRAMES_PER_BUFFER = 512;
    PaError open_err = paInvalidSampleRate;

    for (const int rate : rates_to_try) {
        open_err = Pa_OpenStream(&pa_stream_, &params, nullptr, static_cast<double>(rate), FRAMES_PER_BUFFER, paClipOff, &ListenNode::paCallback, this);
        if (open_err == paNoError) {
            pa_sample_rate_ = rate;
            break;
        }
        RCLCPP_DEBUG(get_logger(), "Pa_OpenStream() failed for sample rate %d: %s. Trying next rate...", rate, Pa_GetErrorText(open_err));
    }

    if (open_err != paNoError) {
        RCLCPP_ERROR(get_logger(), "Failed to open PortAudio stream with sample rates %d, 44100, or %d. Error: %s", sample_rate_, native_rate, Pa_GetErrorText(open_err));
        Pa_Terminate();
        return false;
    }

    err = Pa_StartStream(pa_stream_);
    if (err != paNoError) {
        RCLCPP_ERROR(get_logger(), "Pa_StartStream() : %s", Pa_GetErrorText(err));
        Pa_CloseStream(pa_stream_);
        pa_stream_ = nullptr;
        Pa_Terminate();
        return false;
    }

    RCLCPP_INFO(get_logger(), "PortAudio stream started with sample rate %d (requested %d).", pa_sample_rate_, sample_rate_);
    return true;
}

void ListenNode::shutdownPortAudio()
{
    if (pa_stream_) {
        Pa_StopStream(pa_stream_);
        Pa_CloseStream(pa_stream_);
        pa_stream_ = nullptr;
        Pa_Terminate();
    }
}

int ListenNode::paCallback(const void* input, void* /*output*/,
                           unsigned long frames_per_buffer,
                           const PaStreamCallbackTimeInfo* /*time_info*/,
                           PaStreamCallbackFlags /*status_flags*/,
                           void* user_data)
{
    auto* self = static_cast<ListenNode*>(user_data);
    const auto* s = static_cast<const float*>(input);
    if (!s) { return paContinue; }

    std::lock_guard lock(self->audio_mutex_);
    for (unsigned long i = 0; i < frames_per_buffer; ++i) {
        self->audio_buffer_.push_back(s[i]);
    }
    while (self->audio_buffer_.size() > AUDIO_BUFFER_MAX_SAMPLES) {
        self->audio_buffer_.pop_front();
    }
    return paContinue;
}

void ListenNode::processLoop()
{
    if (!spotter_ || !stream_) { return; }

    constexpr int CHUNK_SAMPLES = 512;
    std::vector<float> chunk;
    chunk.reserve(CHUNK_SAMPLES);

    while (running_) {
        chunk.clear();
        {
            std::lock_guard lock(audio_mutex_);
            const int to_read = std::min(static_cast<int>(audio_buffer_.size()), CHUNK_SAMPLES);
            for (int i = 0; i < to_read; ++i) {
                chunk.push_back(audio_buffer_.front());
                audio_buffer_.pop_front();
            }
        }

        if (chunk.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        float max_val = 0.0f;
        for (float f : chunk) { max_val = std::max(max_val, std::abs(f)); }
        if (max_val > 0.1f) {
            RCLCPP_DEBUG(get_logger(), "Audio level: %f", max_val);
        }

        SherpaOnnxOnlineStreamAcceptWaveform(stream_, pa_sample_rate_, chunk.data(), static_cast<int>(chunk.size()));

        bool reinit_stream = false;
        static int total_decode_steps = 0;

        static int chunk_count = 0;
        if (++chunk_count % 100 == 0) {
            RCLCPP_DEBUG(get_logger(), "Chunks: %d, decode_steps: %d, buffer: %zu",
                chunk_count, total_decode_steps, audio_buffer_.size());
        }

        while (SherpaOnnxIsKeywordStreamReady(spotter_, stream_)) {
            ++total_decode_steps;
            SherpaOnnxDecodeKeywordStream(spotter_, stream_);

            const SherpaOnnxKeywordResult* result = SherpaOnnxGetKeywordResult(spotter_, stream_);
            if (result) {
                const std::string kw(result->keyword ? result->keyword : "");
                if (!kw.empty()) {
                    RCLCPP_DEBUG(get_logger(), "Raw keyword result: '%s'", kw.c_str());
                }
                SherpaOnnxDestroyKeywordResult(result);
                if (!kw.empty()) {
                    handleKeyword(kw);
                    reinit_stream = true;
                    break;
                }
            }
        }

        if (reinit_stream) {
            SherpaOnnxDestroyOnlineStream(stream_);
            stream_ = SherpaOnnxCreateKeywordStream(spotter_);
            if (!stream_) {
                RCLCPP_ERROR(get_logger(), "Failed to reinitialize Sherpa-ONNX stream after keyword detection.");
                running_ = false;
                return;
            }
        }
    }
}

void ListenNode::handleKeyword(const std::string& keyword)
{
    std::string kw = keyword;
    std::transform(kw.begin(), kw.end(), kw.begin(), [](unsigned char c){ return std::tolower(c); });
    RCLCPP_INFO(get_logger(), "Keyword detected: '%s'", kw.c_str());

    PendingPublish pending;
    if (kw == kw_mode_teleop_) {
        pending.kind = PendingPublish::Kind::ModeRequest;
        pending.data = "TELEOP";
    } else if (kw == kw_mode_follow_) {
        pending.kind = PendingPublish::Kind::ModeRequest;
        pending.data = "FOLLOW";
    } else if (kw == kw_mode_listen_) {
        pending.kind = PendingPublish::Kind::ModeRequest;
        pending.data = "LISTEN";
    }
    else if (kw == kw_dock_)    { pending.kind = PendingPublish::Kind::Dock; }
    else if (kw == kw_undock_)  { pending.kind = PendingPublish::Kind::Undock; }
    else if (kw == kw_forward_) { pending.kind = PendingPublish::Kind::Command; pending.data = "forward"; }
    else if (kw == kw_left_)    { pending.kind = PendingPublish::Kind::Command; pending.data = "left";    }
    else if (kw == kw_right_)   { pending.kind = PendingPublish::Kind::Command; pending.data = "right";   }
    else {
        RCLCPP_WARN(get_logger(), "Unrecognized keyword '%s'. Ignoring.", kw.c_str());
        return;
    }

    std::lock_guard lock(publish_mutex_);
    publish_queue_.push_back(std::move(pending));
}

void ListenNode::onPublishTimer()
{
    std::vector<PendingPublish> to_publish;
    {
        std::lock_guard lock(publish_mutex_);
        to_publish.swap(publish_queue_);
    }
    for (const auto& p : to_publish) {
        switch (p.kind) {
            case PendingPublish::Kind::Command: {
                std_msgs::msg::String msg;
                msg.data = p.data;
                command_pub_->publish(msg);
                RCLCPP_INFO(get_logger(), "Command '%s' → /listen/command.", p.data.c_str());
                break;
            }

            case PendingPublish::Kind::ModeRequest: {
                std_msgs::msg::String msg;
                msg.data = p.data;
                mode_request_pub_->publish(msg);
                RCLCPP_INFO(get_logger(), "Mode request '%s' → /listen/mode_request.", p.data.c_str());
                break;
            }

            case PendingPublish::Kind::Dock:
                dock_pub_->publish(std_msgs::msg::Empty{});
                RCLCPP_INFO(get_logger(), "Voice dock command → /teleop/dock.");
                break;

            case PendingPublish::Kind::Undock:
                undock_pub_->publish(std_msgs::msg::Empty{});
                RCLCPP_INFO(get_logger(), "Voice undock command → /teleop/undock.");
                break;
        }
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<ListenNode>());
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("listen_node"), "Fatal error in ListenNode: %s", e.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}