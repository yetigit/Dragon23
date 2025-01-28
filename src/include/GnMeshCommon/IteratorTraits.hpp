
#pragma once

GNM_NAMESPACE_OPEN

namespace detail {

template <class... _SomeTs> using dummy_tmpl0 = void;

using std::distance;
template <class _Ty, class = void> struct has_distancefn : std::false_type {};

template <class _Ty>
struct has_distancefn<_Ty, detail::dummy_tmpl0<decltype(distance(
                               std::declval<_Ty>(), std::declval<_Ty>()))>>
    : std::true_type {};

template <class T>
constexpr static bool has_distancefn_v = has_distancefn<T>::value;
} // namespace detail

GNM_NAMESPACE_CLOSE