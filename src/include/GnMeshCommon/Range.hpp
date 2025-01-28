
#pragma once

#include <GnMeshCommon/IteratorTraits.hpp>
#include <type_traits>

GNM_NAMESPACE_OPEN

template <class IteratorT> struct IterationRange;

/*!
 * \class IterationRange
 *
 * \brief wraps a begin and end iterator
 * \tparam IteratorT
 * \warning always check boolop before you attempt iteration
 */
template <class IteratorT> struct IterationRange {

  IteratorT mBegin, mEnd;
  IterationRange(const IteratorT &in_begin, const IteratorT &in_end)
      : mBegin(in_begin), mEnd(in_end) {}

  IterationRange(IterationRange const &) = default;

  IteratorT begin() { return mBegin; }
  IteratorT end() { return mEnd; }
};

struct NullIterationRange {};

template <class IterT> auto makeIterationRange(IterT begin, IterT end) {
  return IterationRange<IterT>(begin, end);
}

GNM_NAMESPACE_CLOSE