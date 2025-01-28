#pragma once

#include <mimalloc.h>
#include <cstdio>
#include <cassert>

#include <vector>

#include <GnMeshCommon/TParam.hpp> // wrapvalue

GNM_NAMESPACE_OPEN

struct LinearBlockConfig {
  unsigned blockSize = 0u;
  unsigned numBlock = 0u;
};

// TODO  do a copy over if realloc fn fails
// TODO do a realloc function that also saves some 'map' ie two size_t's
// one from the past whole block , and the second from the new one
// so that all we have to do to recover the pointers is copy over
template <class Identity = void> class LinearBlockAlloc {

#define BLOCK_ALLOC_ALIGNMENT 16

public:
  size_t getTotalSize() const noexcept {

    return mConfig.numBlock * mConfig.blockSize;
  }

  // to fallback to new
  struct OperatorNewRAII {

    void *p;

    OperatorNewRAII() noexcept : p(nullptr) {}

    OperatorNewRAII(size_t const sz)
        : p(::operator new(sz))

    {
      assert(p != nullptr && "failed to get mem from os");
    }

    ~OperatorNewRAII() { ::operator delete(p); }

    OperatorNewRAII(OperatorNewRAII const &) = delete;
    OperatorNewRAII &operator=(OperatorNewRAII const &) = delete;
    OperatorNewRAII &operator=(OperatorNewRAII &&) = delete;
    OperatorNewRAII(OperatorNewRAII &&other) noexcept {
      p = other.p;
      other.p = nullptr;
    }
  };

  static LinearBlockAlloc *pSingleton;

#ifndef NDEBUG
  static unsigned singleton_counter;
#endif

  static LinearBlockAlloc *get() noexcept { return pSingleton; }

  static LinearBlockAlloc *init(const LinearBlockConfig in_config) noexcept {
    pSingleton = new LinearBlockAlloc(in_config);

#ifndef NDEBUG
    ++singleton_counter;
#ifndef SEVERAL_LINEAR_BLOCK_ALLOCATOR_PER_INSTANCE
    //assert(singleton_counter == 1);
#endif
#endif

    pSingleton->preAlloc();
    return pSingleton;
  }

  static void deinit() {
    if (pSingleton) {
		delete pSingleton;
    }
  }

public:
  // we fallback to new if we go out of blocks
  std::vector<OperatorNewRAII> mNews;
  const LinearBlockConfig mConfig;
  void *pStart;
  char *pOffset;

private:
  LinearBlockAlloc(const LinearBlockConfig in_config)

      : mNews(), mConfig(in_config), pStart(nullptr), pOffset(nullptr) {

#ifndef NDEBUG
    printf("LinearBlockAlloc<blockSize=%d, numBlocks=%d>()\n",
           (int)mConfig.blockSize, (int)mConfig.numBlock);
#endif
  }

public:
  ~LinearBlockAlloc() { mi_free_aligned(pStart, BLOCK_ALLOC_ALIGNMENT); 
pStart = nullptr; 
pOffset = nullptr;

  }

  void preAlloc() {
    pStart = mi_malloc_aligned(getTotalSize(), BLOCK_ALLOC_ALIGNMENT);
    assert(pStart != nullptr && "failed to get mem");
#ifndef NDEBUG
    printf("prealloc: %d, from LinearBlockAlloc<blockSize=%d, numBlocks=%d>\n",
           (int)getTotalSize(), (int)mConfig.blockSize, (int)mConfig.numBlock);
#endif
    pOffset = (char *)pStart;
  }

  bool isOwned(void *p) const noexcept {
    return (char *)pStart <= (char *)p && p < ((char *)pStart + getTotalSize());
  }

  /*!
   * \brief
   *
   * \param[in] allocsize
   * \return ptr to begin of block
   *
   * \note
   * \warning we allocate the whole block size regardless just make sure the
   * size is not greater than it
   */
  void *allocBlock(const size_t allocSize) {

#ifndef NDEBUG
    printf("req: %d, from LinearBlockAlloc<%d, %d>\n", (int)allocSize,
           (int)mConfig.blockSize, (int)mConfig.numBlock);
#endif
    assert(allocSize <= mConfig.blockSize);

    void *ret = (void *)pOffset;

    pOffset += mConfig.blockSize;
    if (pOffset <= ((char *)pStart + getTotalSize())) {
      return ret;
    } else {

#ifndef NDEBUG
      printf("fallback to new\n");
#endif
      mNews.emplace_back(allocSize);
      return mNews.back().p;
    }
  }

  /*!
   * \brief return allocated memory, most of the time multiple of blocsz
   *
   * \return size
   */
  size_t used() const noexcept { return pOffset - (char *)pStart; }

  // we do not release anything
  void releaseBlock() {}

public:
  static void *allocate() {
    LinearBlockAlloc *active_instance = LinearBlockAlloc::get();
    return active_instance->allocBlock(active_instance->mConfig.blockSize);
  }
  static void destroy(void *, const size_t) noexcept {
    //
  }

  // struct OpNew {

  //  static void *operator new(const size_t sz) {
  //    return LinearBlockAlloc::get()->allocBlock(sz);
  //  }

  //  // no op
  //  static void operator delete(void *, const size_t) noexcept {}

  //  static void *operator new[](const size_t) = delete;
  //  static void operator delete[](void *, const size_t) = delete;
  //};
};

template <class Identity>
LinearBlockAlloc<Identity> *LinearBlockAlloc<Identity>::pSingleton = nullptr;

#ifndef NDEBUG
template <class Identity>
unsigned LinearBlockAlloc<Identity>::singleton_counter = 0;
#endif

GNM_NAMESPACE_CLOSE
