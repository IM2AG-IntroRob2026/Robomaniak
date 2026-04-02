#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace robot_audio_pipeline
{

class SnowboyEngine
{
public:
  SnowboyEngine();
  ~SnowboyEngine();

  bool initialize(
    const std::string & resource_path,
    const std::vector<std::string> & model_paths,
    const std::vector<float> & sensitivities,
    float audio_gain,
    bool apply_frontend,
    int frame_duration_ms,
    std::string * error);

  bool process(
    const int16_t * pcm_frame,
    int32_t frame_length,
    int32_t * keyword_index,
    std::string * error) const;

  int sample_rate() const;
  int frame_length() const;
  bool is_ready() const;

private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace robot_audio_pipeline
