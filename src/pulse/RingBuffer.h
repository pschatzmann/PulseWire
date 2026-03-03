#pragma once
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace pulsewire {

/**
 * @brief Efficient lock-free ring buffer for storing data.
 *
 * Implements a FIFO circular buffer for storage and retrieval.
 * Used internally for TX and RX buffering.
 * Supports push, read, peek, and bulk operations with minimal memory movement.
 *
 * - Data is written at the tail and read from the head.
 * - Buffer automatically wraps around when full.
 * - Provides methods for available space, bulk read/write, and peeking.
 *
 * @tparam T The type of elements stored in the ring buffer.
 *
 * @copyright GPLv3
 */
template <typename T>
class RingBuffer {
 public:
  RingBuffer(size_t size = 128) { resize(size); }

  ~RingBuffer() {
    delete[] _buffer;
    _buffer = nullptr;
  }

  void resize(size_t new_size) {
    delete[] _buffer;
    _buffer = new T[new_size];
    _capacity = new_size;
    clear();
  }

  bool write(const T& value) {
    if (isFull()) return false;
    _buffer[_tail] = value;
    _tail = (_tail + 1) % _capacity;
    _count++;
    return true;
  }

  int writeArray(const T* data, size_t len) {
    int written = 0;
    for (size_t i = 0; i < len; ++i) {
      if (write(data[i])) {
        ++written;
      } else {
        break;
      }
    }
    return written;
  }

  size_t available() const { return _count; }

  void clear() {
    _head = 0;
    _tail = 0;
    _count = 0;
  }

  bool isFull() const { return _count == _capacity; }

  bool isEmpty() const { return _count == 0; }

  size_t size() const { return _capacity; }

  /// Read and remove the next element. Returns true if successful.
  bool read(T& out) {
    if (_count == 0) return false;
    out = _buffer[_head];
    _head = (_head + 1) % _capacity;
    _count--;
    return true;
  }

  /// Read and remove the next byte (uint8_t specialization compatibility).
  /// Returns -1 if empty.
  int read() {
    if (_count == 0) return -1;
    T value = _buffer[_head];
    _head = (_head + 1) % _capacity;
    _count--;
    return static_cast<int>(value);
  }

  /// Read up to len elements into dest, returns number of elements read.
  int readArray(T* dest, size_t len) {
    int n = 0;
    while (n < static_cast<int>(len) && _count > 0) {
      dest[n] = _buffer[_head];
      _head = (_head + 1) % _capacity;
      _count--;
      ++n;
    }
    return n;
  }

  /// Peek at the next element without removing it. Returns true if successful.
  bool peek(T& out) const {
    if (_count == 0) return false;
    out = _buffer[_head];
    return true;
  }

  /// Peek (uint8_t specialization compatibility). Returns -1 if empty.
  int peek() const {
    if (_count == 0) return -1;
    return static_cast<int>(_buffer[_head]);
  }

  /// Returns available space for writing.
  size_t availableForWrite() const { return _capacity - _count; }

 protected:
  T* _buffer = nullptr;
  size_t _capacity = 0;
  size_t _head = 0;
  size_t _tail = 0;
  size_t _count = 0;
};

}  // namespace pulsewire