#include "robot_audio_pipeline/snowboy_engine.hpp"

#include <algorithm>
#include <array>
#include <sstream>
#include <string>

#include "robot_audio_pipeline/snowboy_c_bridge.h"

namespace robot_audio_pipeline
{

namespace
{
std::string join_models(const std::vector<std::string> & model_paths)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < model_paths.size(); ++i) {
    if (i != 0U) {
      oss << ",";
    }
    oss << model_paths[i];
  }
  return oss.str();
}

std::string join_sensitivities(const std::vector<float> & sensitivities)
{
  std::ostringstream oss;
  for (std::size_t i = 0; i < sensitivities.size(); ++i) {
    if (i != 0U) {
      oss << ",";
    }
    oss << std::clamp(sensitivities[i], 0.0F, 1.0F);
  }
  return oss.str();
}
}  // namespace

struct SnowboyEngine::Impl
{
  rb_snowboy_handle * handle{nullptr};
  int sample_rate{0};
  int frame_length{0};
};

SnowboyEngine::SnowboyEngine()
: impl_(std::make_unique<Impl>())
{
}

SnowboyEngine::~SnowboyEngine()
{
  if (impl_ != nullptr && impl_->handle != nullptr) {
    rb_snowboy_destroy(impl_->handle);
    impl_->handle = nullptr;
  }
}

bool SnowboyEngine::initialize(
  const std::string & resource_path,
  const std::vector<std::string> & model_paths,
  const std::vector<float> & sensitivities,
  float audio_gain,
  bool apply_frontend,
  int frame_duration_ms,
  std::string * error)
{
  if (resource_path.empty()) {
    if (error != nullptr) {
      *error = "Parametre 'resource_path' manquant";
    }
    return false;
  }

  if (model_paths.empty()) {
    if (error != nullptr) {
      *error = "Parametre 'hotword_model_paths' vide";
    }
    return false;
  }

  if (sensitivities.size() != model_paths.size()) {
    if (error != nullptr) {
      *error = "Le nombre de sensibilites doit etre egal au nombre de modeles";
    }
    return false;
  }

  const std::string model_list = join_models(model_paths);
  const std::string sensitivity_csv = join_sensitivities(sensitivities);

  if (impl_->handle != nullptr) {
    rb_snowboy_destroy(impl_->handle);
    impl_->handle = nullptr;
  }

  std::array<char, 512> error_buffer{};
  impl_->handle = rb_snowboy_create(
    resource_path.c_str(),
    model_list.c_str(),
    sensitivity_csv.c_str(),
    audio_gain,
    apply_frontend ? 1 : 0,
    error_buffer.data(),
    error_buffer.size());

  if (impl_->handle == nullptr) {
    if (error != nullptr) {
      *error = std::string("Creation SnowboyDetect impossible: ") + error_buffer.data();
    }
    return false;
  }

  impl_->sample_rate = rb_snowboy_sample_rate(impl_->handle);
  const int num_channels = rb_snowboy_num_channels(impl_->handle);
  const int bits_per_sample = rb_snowboy_bits_per_sample(impl_->handle);

  if (impl_->sample_rate <= 0) {
    if (error != nullptr) {
      *error = "Snowboy a retourne un sample_rate invalide";
    }
    rb_snowboy_destroy(impl_->handle);
    impl_->handle = nullptr;
    return false;
  }

  if (num_channels != 1 || bits_per_sample != 16) {
    if (error != nullptr) {
      *error = "Ce pipeline attend Snowboy en mono 16 bits";
    }
    rb_snowboy_destroy(impl_->handle);
    impl_->handle = nullptr;
    return false;
  }

  const int clamped_ms = std::clamp(frame_duration_ms, 20, 250);
  impl_->frame_length = std::max(64, (impl_->sample_rate * clamped_ms) / 1000);
  return true;
}

bool SnowboyEngine::process(
  const int16_t * pcm_frame,
  int32_t frame_length,
  int32_t * keyword_index,
  std::string * error) const
{
  if (!is_ready() || pcm_frame == nullptr || keyword_index == nullptr || frame_length <= 0) {
    if (error != nullptr) {
      *error = "Snowboy non initialise ou arguments invalides";
    }
    return false;
  }

  std::array<char, 512> error_buffer{};
  const int rv = rb_snowboy_run_detection(
    impl_->handle,
    pcm_frame,
    frame_length,
    0,
    error_buffer.data(),
    error_buffer.size());

  if (rv == -1) {
    if (error != nullptr) {
      *error = std::string("Snowboy RunDetection a retourne une erreur: ") + error_buffer.data();
    }
    return false;
  }

  // Snowboy retourne 1..N pour les hotwords detectes; 0 = none; <0 = non-evenement.
  if (rv > 0) {
    *keyword_index = rv - 1;
  } else {
    *keyword_index = -1;
  }
  return true;
}

int SnowboyEngine::sample_rate() const
{
  return impl_->sample_rate;
}

int SnowboyEngine::frame_length() const
{
  return impl_->frame_length;
}

bool SnowboyEngine::is_ready() const
{
  return impl_ != nullptr && impl_->handle != nullptr;
}

}  // namespace robot_audio_pipeline
