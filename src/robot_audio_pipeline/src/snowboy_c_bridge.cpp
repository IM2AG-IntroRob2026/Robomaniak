#include "robot_audio_pipeline/snowboy_c_bridge.h"

#include <exception>
#include <cstdio>
#include <new>
#include <string>

#include <snowboy-detect.h>

struct rb_snowboy_handle
{
  snowboy::SnowboyDetect * detector{nullptr};
};

namespace
{
void set_error(char * buffer, std::size_t size, const char * message)
{
  if (buffer == nullptr || size == 0U) {
    return;
  }
  std::snprintf(buffer, size, "%s", (message != nullptr) ? message : "unknown error");
}
}  // namespace

rb_snowboy_handle * rb_snowboy_create(
  const char * resource_path,
  const char * model_list,
  const char * sensitivity_csv,
  float audio_gain,
  int apply_frontend,
  char * error_buffer,
  std::size_t error_buffer_size)
{
  if (resource_path == nullptr || model_list == nullptr || sensitivity_csv == nullptr) {
    set_error(error_buffer, error_buffer_size, "Arguments Snowboy invalides");
    return nullptr;
  }

  auto * handle = new (std::nothrow) rb_snowboy_handle();
  if (handle == nullptr) {
    set_error(error_buffer, error_buffer_size, "Allocation rb_snowboy_handle impossible");
    return nullptr;
  }

  try {
    handle->detector = new snowboy::SnowboyDetect(std::string(resource_path), std::string(model_list));
    handle->detector->SetSensitivity(std::string(sensitivity_csv));
    handle->detector->SetAudioGain(audio_gain);
    handle->detector->ApplyFrontend(apply_frontend != 0);
  } catch (const std::exception & e) {
    set_error(error_buffer, error_buffer_size, e.what());
    delete handle->detector;
    delete handle;
    return nullptr;
  } catch (...) {
    set_error(error_buffer, error_buffer_size, "Exception inconnue Snowboy");
    delete handle->detector;
    delete handle;
    return nullptr;
  }

  return handle;
}

void rb_snowboy_destroy(rb_snowboy_handle * handle)
{
  if (handle == nullptr) {
    return;
  }
  delete handle->detector;
  handle->detector = nullptr;
  delete handle;
}

int rb_snowboy_sample_rate(const rb_snowboy_handle * handle)
{
  if (handle == nullptr || handle->detector == nullptr) {
    return 0;
  }
  return handle->detector->SampleRate();
}

int rb_snowboy_num_channels(const rb_snowboy_handle * handle)
{
  if (handle == nullptr || handle->detector == nullptr) {
    return 0;
  }
  return handle->detector->NumChannels();
}

int rb_snowboy_bits_per_sample(const rb_snowboy_handle * handle)
{
  if (handle == nullptr || handle->detector == nullptr) {
    return 0;
  }
  return handle->detector->BitsPerSample();
}

int rb_snowboy_run_detection(
  rb_snowboy_handle * handle,
  const int16_t * pcm_data,
  int data_length,
  int is_end,
  char * error_buffer,
  std::size_t error_buffer_size)
{
  if (handle == nullptr || handle->detector == nullptr || pcm_data == nullptr || data_length <= 0) {
    set_error(error_buffer, error_buffer_size, "Arguments RunDetection invalides");
    return -1;
  }

  try {
    return handle->detector->RunDetection(pcm_data, data_length, is_end != 0);
  } catch (const std::exception & e) {
    set_error(error_buffer, error_buffer_size, e.what());
    return -1;
  } catch (...) {
    set_error(error_buffer, error_buffer_size, "Exception inconnue RunDetection");
    return -1;
  }
}
