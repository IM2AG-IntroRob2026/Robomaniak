#pragma once

#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct rb_snowboy_handle rb_snowboy_handle;

rb_snowboy_handle * rb_snowboy_create(
  const char * resource_path,
  const char * model_list,
  const char * sensitivity_csv,
  float audio_gain,
  int apply_frontend,
  char * error_buffer,
  std::size_t error_buffer_size);

void rb_snowboy_destroy(rb_snowboy_handle * handle);

int rb_snowboy_sample_rate(const rb_snowboy_handle * handle);
int rb_snowboy_num_channels(const rb_snowboy_handle * handle);
int rb_snowboy_bits_per_sample(const rb_snowboy_handle * handle);

int rb_snowboy_run_detection(
  rb_snowboy_handle * handle,
  const int16_t * pcm_data,
  int data_length,
  int is_end,
  char * error_buffer,
  std::size_t error_buffer_size);

#ifdef __cplusplus
}
#endif
