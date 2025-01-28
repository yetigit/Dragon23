
#pragma once

#include <GnMeshCommon/Namespace.hpp>

#define DRGN_DECL_METHOD_DETECTOR_STRUCT(clsname, todetect)                    \
  template <class _detail_args_Cls, class... _detail_args> struct clsname {    \
    template <                                                                 \
        class _detail_T = _detail_args_Cls,                                    \
        class _detail_Result = decltype(std::declval<_detail_T>().todetect(    \
            static_cast<_detail_args>(std::declval<_detail_args>())...))>      \
    static std::true_type test(int);                                           \
    static std::false_type test(...);                                          \
    static constexpr bool value = decltype(test(0))::value;                    \
  }

GNM_NAMESPACE_OPEN

namespace meta {
DRGN_DECL_METHOD_DETECTOR_STRUCT(det_plus_operator, operator+);

} // namespace meta

GNM_NAMESPACE_CLOSE
