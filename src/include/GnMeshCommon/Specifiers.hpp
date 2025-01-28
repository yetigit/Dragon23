#pragma once


#if defined(_MSC_VER)
#  define GNM_INLINE __forceinline
#else
#  define GNM_INLINE inline __attribute__((always_inline)) __attribute__((__unused__))
#endif



// since C++14 https://en.cppreference.com/w/cpp/language/attributes/deprecated
#define GNM_DEPRECATED(msg) [[deprecated(msg)]]


#define GNM_NOEXCEPT noexcept


