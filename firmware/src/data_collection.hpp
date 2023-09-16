
//
// Created by christian on 9/6/23.
//

#ifndef THERMO_SCOPE_DATA_COLLECTION_HPP
#define THERMO_SCOPE_DATA_COLLECTION_HPP
#include <sys/cdefs.h>
#include <string>
#include "ring_buffer.hpp"
#include <pico/util/queue.h>

extern uint64_t data_collection_timestamp_offset;

class DataChannel {
  float mean_filter_total = 0;
  uint32_t mean_filter_count = 0;
  uint64_t last_sample_timestamp = 0;
  queue_t intercore_data_queue;
public:
  struct DataPoint {
    double value;
    uint64_t timestamp;
  };
  RingBuffer<DataPoint> influx_export_buffer{50};
  RingBuffer<float> graph_downsample_buffer{200};
  std::string channel_name;
  explicit DataChannel (std::string channel_name_in) : channel_name(channel_name_in) {
    queue_init(&intercore_data_queue, sizeof(struct DataPoint), 40);
  }

  void push_new_value (double new_value) {
    struct DataPoint new_point;
    new_point.value = new_value;
    new_point.timestamp = to_us_since_boot(get_absolute_time())*1000 + data_collection_timestamp_offset;
    queue_add_blocking(&intercore_data_queue, &new_point);
  }

  void push_new_value (double new_value, absolute_time_t timestamp) {
    struct DataPoint new_point;
    new_point.value = new_value;
    new_point.timestamp = to_us_since_boot(timestamp)*1000 + data_collection_timestamp_offset;
    queue_add_blocking(&intercore_data_queue, &new_point);
  }

  void core0_process_data() {
    struct DataPoint new_point;
    while (queue_try_remove(&intercore_data_queue, &new_point))
    {
      influx_export_buffer.push(new_point);
      mean_filter_count++;
      mean_filter_total += new_point.value;
      if ((mean_filter_count > 0) && ((new_point.timestamp - last_sample_timestamp) > 20*1000))
      {
        graph_downsample_buffer.push(mean_filter_total/mean_filter_count);
        mean_filter_count = 0;
        mean_filter_total = 0;
        last_sample_timestamp = new_point.timestamp;
      }
    }
  }
};

DataChannel* data_collection_create_new_channel(std::string channel_name);

void data_collection_update_gps_time(uint64_t gps_time_us, uint64_t local_time_us);

#endif //THERMO_SCOPE_DATA_COLLECTION_HPP
