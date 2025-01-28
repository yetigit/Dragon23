
#pragma once
/*

 \warning the static array is not meant to be used on its own
 it is strictely meant for use in the block array class ,
 which has its ways of dealing with things

*/

#include <cstring>
#include <cstdint>

#include <GnMeshCommon/Namespace.hpp>
#include <GnMeshCommon/Specifiers.hpp>
#include <GnMeshCommon/TParam.hpp>

#include <algorithm>

GNM_NAMESPACE_OPEN

template <class T, size_t _Alignment> class StaticArray {

  static constexpr unsigned Alignment_ = _Alignment;
  static_assert((Alignment_ >= alignof(T)), "bad alignment");

public:
  using Element_t = T;

  struct alignas(Alignment_) AlignedT {
    AlignedT() {}
    AlignedT(const T &inObj) : obj(inObj) {}
    AlignedT(T &&inObj) : obj(std::move(inObj)) {}

    template <class... Args>
    AlignedT(Args &&...args) : obj(std::forward<Args>(args)...) {}

    T obj;
  };

public:
  StaticArray(const StaticArray &other) = delete;
  StaticArray &operator=(StaticArray &&other) = delete;
  StaticArray &operator=(StaticArray const &other) = delete;

  //////////////////////////////////////////////////////////////////////////

  GNM_INLINE T &operator[](const int idx) GNM_NOEXCEPT {
    return ((AlignedT *)this + idx)->obj;
  }
  GNM_INLINE const T &operator[](const int idx) const GNM_NOEXCEPT {
    return ((AlignedT const *)this + idx)->obj;
  }

  template <class U>
  GNM_INLINE auto
  copyConstructInPlace(const int idx,
                       U &&value) noexcept(noexcept(T(std::forward<U>(value))))
      -> typename std::enable_if<
          std::is_same<typename std::decay<U>::type, T>::value, void>::type

  {
    new ((AlignedT *)this + idx) AlignedT(std::forward<U>(value));
  }

  template <class... Args>
  GNM_INLINE void constructInPlace(const int idx, Args &&...args) noexcept(
      noexcept(T(std::forward<Args>(args)...)))

  {
    new ((AlignedT *)this + idx) AlignedT(std::forward<Args>(args)...);
  }

  //
};
GNM_NAMESPACE_CLOSE
