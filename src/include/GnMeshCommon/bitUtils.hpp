#pragma once
#if defined(WIN32)
#include <intrin.h>
#endif
GNM_NAMESPACE_OPEN

static constexpr bool isPowerOfTwo(unsigned n) {
  return (n > 0 && ((n & (n - 1)) == 0));
}

static constexpr unsigned powOffset(unsigned n) {
  return n <= 1 ? 0 : 1 + powOffset((n + 1) / 2);
}


#if defined(WIN32)
static auto upperPowerOfTwo_intrin(size_t x) { return x == 1 ? 1 : 1 << (64 - __lzcnt64(x - 1)); }
#endif

static constexpr auto upperPowerOfTwo(unsigned v) {

  v--;
  v |= v >> 1;
  v |= v >> 2;
  v |= v >> 4;
  v |= v >> 8;
  v |= v >> 16;
  v++;
  return v;
}

GNM_NAMESPACE_CLOSE
