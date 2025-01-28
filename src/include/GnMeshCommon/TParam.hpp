
#pragma once

#include <GnMeshCommon/Namespace.hpp>
#include <type_traits>

GNM_NAMESPACE_OPEN

struct Use_SmallVector_Hint {}; // no behavior class



namespace meta {

template <class T, T _value> struct WrapTValue {
  using type = T;
  static constexpr T value = _value;
};

template <class T> struct IsWrapTValue : std::false_type {};
template <class U, U _value>
struct IsWrapTValue<WrapTValue<U, _value>> : std::true_type {};

template <class T, class WrapT> struct IsWrapTValueTy : std::false_type {};
template <class T, T _value>
struct IsWrapTValueTy<T, WrapTValue<T, _value>> : std::true_type {};

} // namespace meta

template <bool boolexpr>
using EnableIf = typename std::enable_if<boolexpr>::type;



template <size_t value> constexpr auto szarg() {
  return meta::WrapTValue<size_t, value>{};
}


GNM_NAMESPACE_CLOSE
