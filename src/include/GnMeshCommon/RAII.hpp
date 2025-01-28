#pragma once

//#include <mimalloc.h>
#include <utility>
#include <cassert>

struct OpaqueRAII {

  void *p;

  OpaqueRAII() noexcept : p(nullptr) {}

  OpaqueRAII(const size_t sz)
      : p(malloc(sz))

  {
    assert(p != nullptr && "failed to get mem from os");
  }

  // TODO do a special construct fn for mimalloc std allocator

  template <class T, class... Args>
  static OpaqueRAII construct(T *&casted, Args &&...args) {
    OpaqueRAII result;
    T *someptr = (T *)malloc(sizeof(T));
    new (someptr) T(std::forward<Args>(args)...);
    result.p = (void *)someptr;
    casted = someptr;
    return result;
  }
  template <class T, class... Args>
  static OpaqueRAII construct(Args &&...args) {
    T *decoy = nullptr;
    return OpaqueRAII::construct<T>(std::forward<Args>(args)..., decoy);
  }

  ~OpaqueRAII() { free(p); }

  OpaqueRAII(OpaqueRAII const &) = delete;
  OpaqueRAII &operator=(OpaqueRAII const &) = delete;
  OpaqueRAII &operator=(OpaqueRAII &&other) {
    std::swap(p, other.p);
    return *this;
  }
  OpaqueRAII(OpaqueRAII &&other) noexcept {
    p = other.p;
    other.p = nullptr;
  }

  template <typename T> const T *convert_to() const noexcept {
    return static_cast<const T *>(p);
  }

  template <typename T> T *convert_to() noexcept { return static_cast<T *>(p); }

  bool isValid() const noexcept { return p != nullptr; }

  explicit operator bool() const noexcept { return isValid(); }
};
