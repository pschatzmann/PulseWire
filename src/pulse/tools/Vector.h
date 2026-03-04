#pragma once
#include <stddef.h>
#include <stdint.h>
#include "TransceiverConfig.h"
#if defined(HAS_STL)
#  include <vector>
#endif
namespace pulsewire {

#if defined(HAS_STL)

// Use global std::vector, not pulsewire::std
template <typename T>
using Vector = ::std::vector<T>;

#else

/**
 * @brief Small, header-only vector replacement for non-STL environments.
 *
 * Provides a subset of std::vector API that the project uses:
 *  - constructors, destructor, copy/move
 *  - push_back, emplace_back
 *  - erase(iterator)
 *  - begin/end, data, size, empty, back
 *  - reserve, resize, clear, capacity
 */
template <typename T>
class Vector {
 public:
  /// Type alias for value type
  using value_type = T;
  /// Iterator type (pointer)
  using iterator = T*;
  /// Const iterator type (pointer)
  using const_iterator = const T*;

  /// Default constructor: creates empty vector
  Vector() : _data(nullptr), _size(0), _capacity(0) {}

  /// Construct vector with n default-initialized elements
  explicit Vector(size_t n) : _data(nullptr), _size(0), _capacity(0) {
    resize(n);
  }

  /// Copy constructor
  Vector(const Vector& other) : _data(nullptr), _size(0), _capacity(0) {
    if (other._size) {
      reserve(other._size);
      for (size_t i = 0; i < other._size; ++i) _data[i] = other._data[i];
      _size = other._size;
    }
  }

  /// Move constructor
  Vector(Vector&& other) noexcept
      : _data(other._data), _size(other._size), _capacity(other._capacity) {
    other._data = nullptr;
    other._size = 0;
    other._capacity = 0;
  }

  /// Copy assignment
  Vector& operator=(const Vector& other) {
    if (this != &other) {
      clear();
      reserve(other._size);
      for (size_t i = 0; i < other._size; ++i) _data[i] = other._data[i];
      _size = other._size;
    }
    return *this;
  }

  /// Move assignment
  Vector& operator=(Vector&& other) noexcept {
    if (this != &other) {
      delete[] _data;
      _data = other._data;
      _size = other._size;
      _capacity = other._capacity;
      other._data = nullptr;
      other._size = 0;
      other._capacity = 0;
    }
    return *this;
  }

  /// Destructor: frees memory
  ~Vector() { delete[] _data; }

  /// Add element to end of vector
  void push_back(const T& value) {
    if (_size >= _capacity) reserve(_capacity ? _capacity * 2 : 4);
    _data[_size++] = value;
  }
  /// Iterator to first element
  iterator begin() { return _data; }
  /// Iterator to one past last element
  iterator end() { return _data + _size; }
  /// Const iterator to first element
  const_iterator begin() const { return _data; }
  /// Const iterator to one past last element
  const_iterator end() const { return _data + _size; }

  /// Pointer to underlying data
  T* data() { return _data; }
  /// Const pointer to underlying data
  const T* data() const { return _data; }

  /// Access element by index
  T& operator[](size_t idx) { return _data[idx]; }
  /// Access element by index (const)
  const T& operator[](size_t idx) const { return _data[idx]; }

  /// Number of elements in vector
  size_t size() const { return _size; }
  /// True if vector is empty
  bool empty() const { return _size == 0; }
  /// Remove all elements
  void clear() { _size = 0; }

  /// Access last element
  T& back() { return _data[_size - 1]; }
  /// Access last element (const)
  const T& back() const { return _data[_size - 1]; }

  /// Erase single element at iterator pos, return iterator to next element
  iterator erase(iterator pos) {
    size_t idx = pos - _data;
    if (idx >= _size) return end();
    for (size_t i = idx; i + 1 < _size; ++i) _data[i] = _data[i + 1];
    --_size;
    return _data + idx;
  }

  /// Erase range [first, last), return iterator to next element
  iterator erase(iterator first, iterator last) {
    if (first >= last) return first;
    size_t idx = first - _data;
    size_t idx2 = last - _data;
    if (idx >= _size) return end();
    if (idx2 > _size) idx2 = _size;
    size_t count = idx2 - idx;
    for (size_t i = idx; i + count < _size; ++i) _data[i] = _data[i + count];
    _size -= count;
    return _data + idx;
  }

  /// Resize vector to n elements
  void resize(size_t n) {
    if (n > _capacity) reserve(n);
    for (size_t i = _size; i < n; ++i) _data[i] = T();
    _size = n;
  }

  /// Reserve space for at least cap elements
  void reserve(size_t cap) {
    if (cap <= _capacity) return;
    T* new_data = new T[cap];
    for (size_t i = 0; i < _size; ++i) new_data[i] = _data[i];
    delete[] _data;
    _data = new_data;
    _capacity = cap;
  }

  /// Current allocated capacity
  size_t capacity() const { return _capacity; }

 private:
  T* _data;
  size_t _size;
  size_t _capacity;
};

#endif
}  // namespace pulsewire
