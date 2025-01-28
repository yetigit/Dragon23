#pragma once
#include <ctime>
#include <cstdlib>
#include <iostream>
#include <vector>
/*!
 * \class BitArray
 *
 * \brief
 * \tparam
 * \note
 */
class BitArray {

public:
  unsigned char *mem;
  size_t m_num_chunks;
  size_t m_reqSize;

  BitArray() : mem(nullptr), m_num_chunks(0), m_reqSize(0) {}
  BitArray(size_t numBits, const unsigned char initValue)
      : mem(nullptr), m_num_chunks(0), m_reqSize(0) {
    this->reserve_and_set(numBits, initValue);
  }
  ~BitArray() {
    if (mem) {
      free((void *)mem);
    }
  }

  template <class STREAM_t> void print(STREAM_t &outs) {
    for (size_t i = 0; i < m_reqSize; i++) {
    }
  }
  static BitArray ones(size_t numBits) {
    return BitArray(numBits, (unsigned char)~0u);
  }
  // there must be a smart way
  auto as_intvec() const {
    std::vector<int> ret; 
    ret.reserve(m_reqSize);
    for (size_t i = 0; i < m_reqSize; i++) {
      ret.push_back(isBitOn ( i)); 
    }
    return ret;
  }
  // there must be a smart way 
  size_t num_zeros() const {
    size_t ret = 0;
    for (size_t i = 0; i < m_reqSize; i++) {
      ret += isBitOff(i);
    }
    return ret;
  }

  size_t num_ones() const { 
      return m_reqSize - num_zeros();
  }

  static BitArray random(size_t numBits) {
    srand(time(0));
    BitArray result;
    //
    assert(result.mem == nullptr);
    result.m_reqSize = numBits;
    result.m_num_chunks = (size_t)ceil(double(numBits) / (sizeof(char) * 8));
    result.mem = (unsigned char *)malloc(result.m_num_chunks * sizeof(char));

    //
    for (size_t i = 0; i < numBits; i++) {

      auto randy = (unsigned int)rand();
      auto cur_chunk = (i / (sizeof(char) * 8));
      result.mem[cur_chunk] = randy;
    }

    return result;
  }

  static BitArray zeros(size_t numBits) {
    return BitArray(numBits, (unsigned char)0u);
  }

  size_t size() const { return m_reqSize; }

  BitArray(BitArray &&other) noexcept
  { 
     mem = nullptr; 
     m_num_chunks = 0 ;
     m_reqSize = 0;

     this->operator=(std::move(other)); 
  }
  BitArray &operator=(BitArray &&other) noexcept {
    std::swap(mem, other.mem);
    std::swap(m_num_chunks, other.m_num_chunks);
    std::swap(m_reqSize, other.m_reqSize);
    return *this;
  }

  BitArray &operator=(BitArray const &other) = delete;
  BitArray(BitArray const &other) = delete;

  void reserve(size_t numBits) {
    assert(mem == nullptr);
    m_reqSize = numBits;
    m_num_chunks = (size_t)ceil(double(numBits) / (sizeof(char) * 8));

    mem = (unsigned char *)malloc(m_num_chunks * sizeof(char));
    assert(mem && "failed to malloc");
  }

  void reserve_and_set(const size_t numBits, const unsigned char initValue) {

    assert(mem == nullptr);
    m_reqSize = numBits;
    m_num_chunks = (size_t)ceil(double(numBits) / (sizeof(char) * 8));
    mem = (unsigned char *)malloc(m_num_chunks * sizeof(char));

    assert(mem && "failed to malloc");

    std::fill(mem, mem + m_num_chunks, initValue);
  }

  GNM_INLINE void turnBitOn(const size_t nthBit) {
    const size_t chunk = size_t(nthBit / (sizeof(char) * 8));
    const auto offset = (unsigned char)(nthBit % (sizeof(char) * 8));
    mem[chunk] |= ((unsigned char)1<< offset);
  }

  GNM_INLINE void turnBitOff(const size_t nthBit) {
    const size_t chunk = size_t(nthBit / (sizeof(char) * 8));

    const auto offset = (unsigned char)(nthBit % (sizeof(char) * 8));
    mem[chunk] |= ((unsigned char)1<< offset);
  }

  GNM_INLINE void setBit(const size_t nthBit, unsigned char val) {
    const size_t chunk = size_t(nthBit / (sizeof(char) * 8));
    const auto offset = (unsigned char)(nthBit % (sizeof(char) * 8));
    mem[chunk] = (mem[chunk] & ~((unsigned char)1<< offset)) | (val << offset);
  }

  GNM_INLINE bool isBitOn(const size_t nthBit) const {
    const size_t chunk = size_t(nthBit / (sizeof(char) * 8));
    const auto offset = (unsigned char)(nthBit % (sizeof(char) * 8));
    return mem[chunk] & ((unsigned char)1<< offset);
  }

  GNM_INLINE bool isBitOff(const size_t nthBit) const {
    return !isBitOn(nthBit);
  }

  //
};
