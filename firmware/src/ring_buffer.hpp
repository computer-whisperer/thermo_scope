/* ring_buffer.hpp:
 * A ring buffer implementation that allows for range-based for loop usage, with templated inner type configuration.
 * Author: Christian Balcom
 * Copyright 2023 Kalogon
 * */

#ifndef STARLING_SOFTWARE_RING_BUFFER_HPP
#define STARLING_SOFTWARE_RING_BUFFER_HPP

#include <vector>
#include <queue>
#include <cstdint>


/*!
 * This is a flexible ring-buffer implementation that may use many different inner types.
 *
 * It can be used with cpp range-based for loops for convenience.
 *
 * It has an index-space that is typically 10 times the size of the ring buffer, which allows
 * for consuming iterators to be constructed which correctly respond to missed samples.
 *
 * @tparam T The type of value to fill the ring buffer with
 */
template<typename T>
class RingBuffer {
public:

  /*!
   * A helper class type that represents the index of an entry in the ring buffer, and mapping simple arithmetic operations
   * appropriately to account for wrapping.
   *
   * This helper class also automatically returns a reference to the value in the ring buffer at the given index when the c
   * dereference operator (*) is used.
   */
  class RingBufferIndex {
    RingBuffer<T>& parent;

  public:
    uint32_t index;
    RingBufferIndex(RingBuffer<T>& parent, uint32_t index) : parent(parent), index(index % parent.index_space_max_size){}

    RingBufferIndex(RingBufferIndex& other):
      parent(other.parent),
      index(other.index){}

    //! Prefix increment operator overload
    RingBufferIndex& operator++(){
      return this->operator+=(1);
    }

    //! Postfix increment operator overload
    RingBufferIndex operator++(int){
      RingBufferIndex ret = *this;
      this->operator++();
      return ret;
    }

    //! Prefix decrement operator overload
    RingBufferIndex& operator--(){
      return this->operator-=(1);
    }

    //! Postfix decrement operator overload
    RingBufferIndex operator--(int){
      RingBufferIndex ret = *this;
      this->operator--();
      return ret;
    }

    //! += operator overload
    RingBufferIndex& operator+= (int32_t n){
      index = ((int32_t)index + n) % parent.index_space_max_size;
      return *this;
    }

    //! -= operator overload
    RingBufferIndex& operator-= (int32_t n){
      index = ((int32_t)index - n) % parent.index_space_max_size;
      return *this;
    }

    //! + operator overload
    RingBufferIndex operator+ (int32_t n){
      n = n % parent.index_space_max_size;
      return RingBufferIndex(parent, (index + n));
    }

    //! - operator overload
    RingBufferIndex operator- (int32_t n){
      n = n % parent.index_space_max_size;
      return RingBufferIndex(parent, (index - n));
    }

    //! Dereference operator overload
    T& operator*(){
      return parent.buffer_data[index%parent.max_size];
    }

    //! == operator overload
    bool operator==(const RingBufferIndex& other) const {
      return index == other.index;
    }

    //! != operator overload
    bool operator!=(const RingBufferIndex& other) const {
      return index != other.index;
    }

    //! == operator, testing if two indexes share the same slot in the actual ring buffer
    bool eq_modulo(const RingBufferIndex& other) const {
      return (index % parent.max_size) == (other.index % parent.max_size);
    }
  };
private:
  std::vector<T> buffer_data;

  const uint32_t max_size;
  const uint32_t index_space_max_size;



  // The index where the next element will be inserted
  RingBufferIndex head;
  // The index where the oldest element exists
  RingBufferIndex tail;

public:
  bool is_empty = true;
  bool is_full = false;

  explicit RingBuffer(int32_t max_size) :
          buffer_data(max_size),
          max_size(max_size),
          index_space_max_size(max_size*100),
          head(*this, 0),
          tail(*this, 0){}

  //! Returns a RingBufferIndex for the oldest entry in the ring buffer. Used for range-based for loops.
  RingBufferIndex begin(){
    return tail;
  }

  //! Returns a RingBufferIndex for the newest entry in the ring buffer. Used for range-based for loops.
  RingBufferIndex end(){
    return head;
  }

  //! Pushes the new element onto the ring buffer, deleting the oldest element if the ring buffer is full.
  void push(T new_sample) {
    if (is_full) {
      tail++;
    }
    // Copy into buffer
    *head = new_sample;

    head++;

    is_empty = false;
    is_full = head.eq_modulo(tail);
  }

  //! Pushes the new element onto the ring buffer, deleting the oldest element if the ring buffer is full.
  T* push_and_return_ptr() {
    if (is_full) {
      tail++;
    }
    // Copy into buffer
    auto ptr = &buffer_data[head.index%max_size];

    head++;

    is_empty = false;
    is_full = head.eq_modulo(tail);

    return ptr;
  }

  //! Returns the value at the given index.
  T get_value(uint32_t index) {
    return buffer_data[index%max_size];
  }

  //! Returns a value based on age, where age=0 will return the latest value.
  T get_value_by_age(int32_t age) {
    return *(head - (age + 1));
  }

  //! Returns the oldest value from the ring buffer, and also deletes it.
  T pop_oldest() {
    if (is_empty) {
      return T();
    }
    auto old_tail = tail;

    tail++;

    is_full = false;
    is_empty = head.eq_modulo(tail);

    return *old_tail;
  }

  //! Returns the number of values currently stored in the ring buffer.
  uint32_t get_num_entries() {
    if (is_full) {
      return max_size;
    }
    else if (is_empty){
      return 0;
    }
    else {
      return ((head.index + index_space_max_size) - (tail.index + 1))%index_space_max_size;
    }
  }

  //! Returns the mean of the values currently stored in the ring buffer.
  float get_mean() {
    float sum = 0;
    uint32_t num = 0;
    for (auto value: *this) {
      sum += value;
      num++;
    }
    return sum / ((float) num);
  }

  //! Empties the ring buffer.
  void clear() {
    head.index = 0;
    tail.index = 0;
    this->is_full = false;
    this->is_empty = true;
  }

};

#endif //STARLING_SOFTWARE_RING_BUFFER_HPP
