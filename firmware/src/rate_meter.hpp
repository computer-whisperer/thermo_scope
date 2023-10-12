/* rate_meter.hpp:
 * A meter designed to track the long-term rate of change of a value.
 * It's currently written to work on delta updates only, but could be enhanced in the future.
 * Author: Christian Balcom
 * */


#ifndef THERMO_SCOPE_RATE_METER_HPP
#define THERMO_SCOPE_RATE_METER_HPP
#include "stdint.h"
#include "ring_buffer.hpp"

class RateMeter
{
private:
  static constexpr uint32_t ring_buffer_size = 40;
  RingBuffer<float> checkpoint_deltas{ring_buffer_size};

  float checkpoint_deltas_total = 0;

  static constexpr uint32_t ms_per_checkpoint = 8000;
  float oldest_value = 0;
  uint32_t oldest_time_ms = 0;

  uint32_t newest_checkpoint_time_ms = 0;

  float newest_value = 0;
  uint32_t newest_time_ms = 0;

  float current_delta_accumulation = 0;

public:

  RateMeter(float start_value, uint32_t start_time_ms)
  {
    oldest_value = start_value;
    oldest_time_ms = start_time_ms;
    newest_value = start_value;
    newest_time_ms = start_time_ms;
    newest_checkpoint_time_ms = start_time_ms;
  }

  uint32_t get_time_span()
  {
    return newest_time_ms - oldest_time_ms;
  }

  float get_rate(uint32_t current_time_ms)
  {
    uint32_t num_entries = checkpoint_deltas.get_num_entries();
    if (oldest_time_ms == newest_time_ms)
    {
      return 0;
    }
    else
    {
      uint32_t time_span_ms = current_time_ms - oldest_time_ms;
      return (current_delta_accumulation + checkpoint_deltas_total)/(float)time_span_ms;
    }
  }

  //! Add @param new_value_delta to the counter
  void add_value_delta(float new_value_delta, uint32_t time_ms)
  {
    if (newest_checkpoint_time_ms == 0)
    {
      oldest_time_ms = time_ms;
      oldest_value = 0;
      newest_checkpoint_time_ms = time_ms;
    }
    else if ((time_ms - newest_checkpoint_time_ms) > ms_per_checkpoint)
    {
      if (checkpoint_deltas.get_num_entries() == ring_buffer_size)
      {
        // Use the oldest entry to advance the oldest_ data
        oldest_time_ms += ms_per_checkpoint;
        oldest_value += *checkpoint_deltas.begin();
      }
      checkpoint_deltas.push(current_delta_accumulation);
      current_delta_accumulation = 0;
      newest_checkpoint_time_ms += ms_per_checkpoint;
      // To stop this from creeping, we recalculate this value whenever the checkpoint is advanced
      checkpoint_deltas_total = 0;
      for (auto v : checkpoint_deltas)
      {
        checkpoint_deltas_total += v;
      }
    }

    current_delta_accumulation += new_value_delta;
    newest_value += new_value_delta;
    newest_time_ms = time_ms;
  }
};

#endif //STARLING_SOFTWARE_RATE_METER_HPP
