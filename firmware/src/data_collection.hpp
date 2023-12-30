
//
// Created by christian on 9/6/23.
//

#ifndef THERMO_SCOPE_DATA_COLLECTION_HPP
#define THERMO_SCOPE_DATA_COLLECTION_HPP
#include <sys/cdefs.h>
#include <string>
#include "ring_buffer.hpp"
#include <pico/util/queue.h>

volatile extern bool data_collection_ready_for_dormant;
volatile extern bool data_collection_requesting_dormant;

class DataChannel {
  float mean_filter_total = 0;
  uint32_t mean_filter_count = 0;
  absolute_time_t last_sample_timestamp = nil_time;
  queue_t intercore_data_queue;
public:
  struct DataPoint {
    double value;
    absolute_time_t timestamp;
  };
  RingBuffer<float> graph_downsample_buffer{200};
  std::string channel_name;
  explicit DataChannel (std::string channel_name_in) : channel_name(channel_name_in) {
    queue_init(&intercore_data_queue, sizeof(struct DataPoint), 100);
  }

  void push_new_value (double new_value) {
    struct DataPoint new_point;
    new_point.value = new_value;
    new_point.timestamp = get_absolute_time();
    queue_add_blocking(&intercore_data_queue, &new_point);
  }

  void push_new_value (double new_value, absolute_time_t timestamp) {
    struct DataPoint new_point;
    new_point.value = new_value;
    new_point.timestamp = timestamp;
    queue_add_blocking(&intercore_data_queue, &new_point);
  }

  void core0_process_data();
};

DataChannel* data_collection_create_new_channel(std::string channel_name);

#endif //THERMO_SCOPE_DATA_COLLECTION_HPP
