

#pragma once

#include <type_traits>
#include <GnMesh/Storage/StaticArray.hpp>
#include <GnMeshCommon/bitUtils.hpp>
#include <GnMeshCommon/IntTypes.hpp>

#include <cstdint>
#include <cassert>
#include <vector>

GNM_NAMESPACE_OPEN

// observation  :
// you cannot push back a convertible type ,whatever you push back must be of
// the explicit type
template <class T, size_t Alignment, class Allocator> class BlockArray {

  using Self_t = BlockArray;

public:
  using Block_t = StaticArray<T, Alignment>;

  using AlignedT = typename Block_t::AlignedT;

public:
  std::vector<Block_t *> mBlocks;
  unsigned offset;

  const int blockNumElements_;
  const int nShiftBits_;
#define blockIndexBitmask_ (blockNumElements_ - 1)

public:
  GNM_INLINE IdxPair splitIndex(const int i) const GNM_NOEXCEPT {
    assert(i >= 0);
    return {i >> nShiftBits_, i & blockIndexBitmask_};
  }

private:
  static int clamp(int value) noexcept {
    return value < 0 ? 0 : (value > 1 ? 1 : value);
  }

public:
  BlockArray(const unsigned in_blockNumElements)

      : mBlocks(), offset(0), blockNumElements_(in_blockNumElements),
        nShiftBits_(powOffset(blockNumElements_)) {
    assert((in_blockNumElements % 2) == 0);
    push_new_block();
  }

  void push_new_block() {
    mBlocks.push_back(static_cast<Block_t *>(Allocator::allocate()));
  }

  ~BlockArray() {
#if 0
    for (auto &cur_ptr : mBlocks) {
      cur_ptr = nullptr;
    }
#endif
  }

public:
  BlockArray(const BlockArray &) = delete;

  BlockArray(BlockArray &&tomove) = delete;

  BlockArray &operator=(const BlockArray &_tocopy) = delete;

  BlockArray &operator=(BlockArray &&_tomove) = delete;

  void deleteBlock(int i) { mBlocks[i] = nullptr; }

  void resize(unsigned const newsize) {
    if (size() > newsize) {

      auto ij = splitIndex(newsize - 1);

      int diff = numBlocks() - (ij.i + 1);
      if (diff) {
        for (size_t i = 0; i < diff; i++) {
          mBlocks.pop_back();
        }
      }

    } else if (size() < newsize) {

      auto ij = splitIndex(newsize - 1);
      int diff = (ij.i + 1) - numBlocks();

      if (diff) {
        for (size_t i = 0; i < diff; i++) {
          push_new_block();
        }
      }
    }

    offset = newsize;
  }

  bool isEmpty() const GNM_NOEXCEPT { return !size(); }

  size_t size() const GNM_NOEXCEPT { return offset; }

  int getBlockSize() const GNM_NOEXCEPT { return blockNumElements_; }
  size_t totalByteSize() const GNM_NOEXCEPT {
    return static_cast<size_t>(numBlocks() * blockNumElements_ * sizeof(T));
  }



  std::vector<int> erase(int *indices, int sz) {
    assert(sz <= size());

    std::vector<int> ret;
    for (size_t i = 0; i < size(); i++) {
      ret.push_back(i);
    }

    size_t newsize = size() - sz;

    std::vector<int> sorted;
    sorted.resize(sz);
    memcpy(sorted.data(), indices, sizeof(int) * sz);

    std::sort(sorted.begin(), sorted.end());

    int comol = 1;
    for (size_t i = 0; i < sorted.size() - 1; i++) {
      int cur = sorted[i];
      int counto = cur + (sorted[i + 1] - sorted[i]);
      for (size_t j = cur + 1; j < counto; j++) {
        ret[j] -= comol;
      }
      comol += 1;
    }
    {

      int cur = sorted.back();
      for (size_t j = cur + 1; j < size(); j++) {
        ret[j] -= comol;
      }
    }

    std::reverse(sorted.begin(), sorted.end());

    for (size_t i = 0; i < sorted.size(); i++) {

      int id = sorted[i];
      for (size_t j = id + 1; j < size(); j++) {
        (*this)[j - 1] = (*this)[j];
      }
    }

    auto ij = splitIndex(newsize);
    if (newsize) {
      mBlocks.resize(ij.i + 1);
    } else {
      mBlocks.resize(0);
    }
    offset = newsize;

    return ret;
  }

  /*!
   * \brief copy-construct a value into the last free element of the blocks or
   * create a new block for the new value
   *
   * \param[in] value
   * \return void
   *
   * \note
   * \warning
   */
  template <class U>
  GNM_INLINE auto pushBack(U &&value) -> typename std::enable_if<
      std::is_same<typename std::decay<U>::type, T>::value, void>::type

  {

    auto ij = splitIndex(offset);

    if (ij.i == numBlocks()) {
      push_new_block();
    }

    mBlocks[ij.i]->copyConstructInPlace(ij.j, std::forward<U>(value));

    ++offset;
  }

  /*!
   * \brief construct a value into the last free element of the blocks or
   * create a new block for the new value
   *
   * \param[in] parameter pack...
   * \return void
   *
   * \note
   * \warning
   */
  template <class... Args> GNM_INLINE void emplaceBack(Args &&...args) {

    auto ij = splitIndex(offset);

    if (ij.i == numBlocks()) {
      push_new_block();
    }

    mBlocks[ij.i]->constructInPlace(ij.j, std::forward<Args>(args)...);

    ++offset;
  }

  size_t numBlocks() const GNM_NOEXCEPT { return mBlocks.size(); }

  void popBack() {
    auto ij = splitIndex(offset);
    if (!ij.j && ij.i) {
      mBlocks.pop_back();
    }

    --offset;
  }

  T &front() { return reinterpret_cast<AlignedT *>(mBlocks.front())->obj; }
  T &back() {
    return reinterpret_cast<AlignedT *>(
               mBlocks.back())[((offset - 1) & blockIndexBitmask_)]
        .obj;
  }
  const T &front() const {
    return reinterpret_cast<AlignedT const *>(mBlocks.front())->obj;
  }
  const T &back() const {
    return reinterpret_cast<AlignedT const *>(
               mBlocks.back())[((offset - 1) & blockIndexBitmask_)]
        .obj;
  }

public:
  void defaultInit() {

    for (size_t i = 0; i < this->size(); i++) {
      new (&(*this)[i]) T();
    }
  }

  GNM_INLINE T &operator[](const int i) {
    const auto ij = splitIndex(i);
    return ((AlignedT *)(mBlocks[ij.i]))[ij.j].obj;
  }
  GNM_INLINE const T &operator[](const int i) const {
    const auto ij = splitIndex(i);
    return ((AlignedT const *)(mBlocks[ij.i]))[ij.j].obj;
  }

  void clear() {}
};
//

GNM_NAMESPACE_CLOSE
