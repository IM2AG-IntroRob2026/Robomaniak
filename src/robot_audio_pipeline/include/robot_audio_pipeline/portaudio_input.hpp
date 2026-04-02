#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <portaudio.h>

namespace robot_audio_pipeline
{

class PortAudioInput
{
public:
  PortAudioInput() = default;
  ~PortAudioInput();

  bool open(int sample_rate, unsigned long frame_length, int device_index, std::string * error);
  bool read_frame(std::vector<int16_t> * pcm, std::string * error);
  void close();

  bool is_open() const {return stream_ != nullptr;}
  int sample_rate() const {return sample_rate_;}
  unsigned long frame_length() const {return frame_length_;}
  const std::string & device_name() const {return device_name_;}

private:
  bool initialize(std::string * error);
  static std::string format_error(const char * context, PaError code);

  PaStream * stream_{nullptr};
  bool portaudio_initialized_{false};
  int sample_rate_{0};
  unsigned long frame_length_{0};
  std::string device_name_{};
};

}  // namespace robot_audio_pipeline
