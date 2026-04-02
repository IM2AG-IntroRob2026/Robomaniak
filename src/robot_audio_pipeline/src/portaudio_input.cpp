#include "robot_audio_pipeline/portaudio_input.hpp"

#include <sstream>

namespace robot_audio_pipeline
{

PortAudioInput::~PortAudioInput()
{
  close();
}

bool PortAudioInput::initialize(std::string * error)
{
  if (portaudio_initialized_) {
    return true;
  }

  const PaError err = Pa_Initialize();
  if (err != paNoError) {
    if (error != nullptr) {
      *error = format_error("Pa_Initialize", err);
    }
    return false;
  }

  portaudio_initialized_ = true;
  return true;
}

bool PortAudioInput::open(
  int sample_rate,
  unsigned long frame_length,
  int device_index,
  std::string * error)
{
  close();

  if (!initialize(error)) {
    return false;
  }

  const PaDeviceIndex device =
    (device_index >= 0) ? static_cast<PaDeviceIndex>(device_index) : Pa_GetDefaultInputDevice();
  if (device == paNoDevice) {
    if (error != nullptr) {
      *error = "Aucun peripherique d'entree audio disponible";
    }
    return false;
  }

  const PaDeviceInfo * device_info = Pa_GetDeviceInfo(device);
  if (device_info == nullptr) {
    if (error != nullptr) {
      *error = "Impossible de lire les informations du peripherique audio";
    }
    return false;
  }
  if (device_info->maxInputChannels < 1) {
    if (error != nullptr) {
      *error = "Le peripherique selectionne n'a pas de canal d'entree";
    }
    return false;
  }

  PaStreamParameters input_params;
  input_params.device = device;
  input_params.channelCount = 1;
  input_params.sampleFormat = paInt16;
  input_params.suggestedLatency = device_info->defaultLowInputLatency;
  input_params.hostApiSpecificStreamInfo = nullptr;

  const PaError open_err = Pa_OpenStream(
    &stream_, &input_params, nullptr, sample_rate, frame_length, paNoFlag, nullptr, nullptr);
  if (open_err != paNoError) {
    if (error != nullptr) {
      *error = format_error("Pa_OpenStream", open_err);
    }
    stream_ = nullptr;
    return false;
  }

  const PaError start_err = Pa_StartStream(stream_);
  if (start_err != paNoError) {
    if (error != nullptr) {
      *error = format_error("Pa_StartStream", start_err);
    }
    close();
    return false;
  }

  sample_rate_ = sample_rate;
  frame_length_ = frame_length;
  device_name_ = (device_info->name != nullptr) ? device_info->name : "unknown_device";
  return true;
}

bool PortAudioInput::read_frame(std::vector<int16_t> * pcm, std::string * error)
{
  if (stream_ == nullptr || pcm == nullptr) {
    if (error != nullptr) {
      *error = "Flux PortAudio non initialise";
    }
    return false;
  }

  pcm->resize(frame_length_);
  const PaError err = Pa_ReadStream(stream_, pcm->data(), frame_length_);

  // Un overflow entree peut arriver sous charge; le frame reste exploitable.
  if (err == paInputOverflowed) {
    return true;
  }

  if (err != paNoError) {
    if (error != nullptr) {
      *error = format_error("Pa_ReadStream", err);
    }
    return false;
  }

  return true;
}

void PortAudioInput::close()
{
  if (stream_ != nullptr) {
    (void)Pa_StopStream(stream_);
    (void)Pa_CloseStream(stream_);
    stream_ = nullptr;
  }

  if (portaudio_initialized_) {
    (void)Pa_Terminate();
    portaudio_initialized_ = false;
  }
}

std::string PortAudioInput::format_error(const char * context, PaError code)
{
  std::ostringstream oss;
  oss << context << " failed: " << Pa_GetErrorText(code) << " (code=" << code << ")";
  return oss.str();
}

}  // namespace robot_audio_pipeline
