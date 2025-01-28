#pragma once

struct HdlTrait {};

/// @brief Base struct covering methods for all handle types
struct BaseHdl : HdlTrait {
  /// @brief index to the item
  int id;
  /// @note default constructed handles are invalid
  BaseHdl() : id(-1) {}
  BaseHdl(const int idx) : id(idx) {}
  /// @brief checks if this handle is valid or not
  /// @return 1==yes
  int isValid() const noexcept { return id >= 0; }
  /// @brief invalidate handle by setting it to -1
  /// @return 
  void invalidate() noexcept { id = -1; }
  explicit operator bool() const noexcept { return isValid(); }

  bool operator<(const BaseHdl other) const noexcept { return id < other.id; }

   operator int() const  { return id; }

  bool operator==(BaseHdl const &other) const noexcept {
    return id == other.id;
  }
  bool operator!=(BaseHdl const &other) const noexcept {
    return id != other.id;
  }
  BaseHdl &operator=(BaseHdl const &other) noexcept = default;
};


/// @brief Edge handle used to refer to an Edge index
struct EdgeHdl : BaseHdl {
  using BaseHdl::BaseHdl;
};


/// @brief Vert Handle used to refer to an Vert index
struct VertHdl : BaseHdl {
  using BaseHdl::BaseHdl;
};

/// @brief Fvert handle used to refer to an Fvert index
struct FvertHdl : BaseHdl {
  using BaseHdl::BaseHdl;
};

/// @brief Face handle used to refer to an Face index
struct FaceHdl : BaseHdl {
  using BaseHdl::BaseHdl;
};


/// @brief Special handle used to refer to both the Edge of a Fvert and the Vert at its start.
///       This type is still 4 bytes
/// @note the Edge index of this handle is 30 bits wide
struct FvEdgeHdl : HdlTrait {
#define FVEH_V 0x4000'0000u
#define FVEH_VALID 0x8000'0000u
#define FVEH_META(symbol1, symbol2) (symbol1 FVEH_V | symbol2 FVEH_VALID)
#define FVEHDL(idx, symbol1, symbol2)                                          \
  FvEdgeHdl(static_cast<unsigned>(idx) | FVEH_META(symbol1, symbol2))

  union {
    unsigned idx;
    struct {
      /// @brief refers to the Edge index
      unsigned id : 30;
      /// @brief refers to the Edge relative Vert index
      unsigned v : 1;
      unsigned valid : 1;
    };
  };

  // implicit conversion to Edge handle
  operator EdgeHdl() const noexcept { return EdgeHdl(id); }
  // default constructor makes an invalid handle
  FvEdgeHdl() : idx(0u) {}
  FvEdgeHdl(const unsigned in_idx) : idx(in_idx) {}

  bool operator==(const FvEdgeHdl r) const { return id == r.id && v == r.v; }
  bool operator==(const EdgeHdl e) const { return id == e.id; }

  FvEdgeHdl &operator=(const FvEdgeHdl &other) {
    id = other.id;
    v = other.v;
    valid = other.valid;
    return *this;
  }
  FvEdgeHdl(unsigned inId, unsigned inV, unsigned in_Valid)
      : idx(inId | (inV * FVEH_V | in_Valid * FVEH_VALID)) {
    assert(inV < 2 && in_Valid < 2);
  }
  int isValid() const noexcept { return valid; }
  void invalidate() noexcept { idx &= ~FVEH_VALID; }
  explicit operator bool() const noexcept { return (bool)isValid(); }
};
/// @brief  helper struct to send pair of FvertHdl 
struct FvertHdlPair {
  FvertHdl first, second;
};

/// @brief  helper struct to send pair of VertHdl 
struct VertHdlPair {
  VertHdl first, second;
};

/// @brief  helper struct to send pair of FaceHdl 
struct FaceHdlPair {
  FaceHdl first, second;
};
/// @brief helper struct to send triplet of FvertHdl
struct FvertHdlTriple {

  union {
    FvertHdl buf[3];

    struct {
      FvertHdl v0;
      FvertHdl v1;
      FvertHdl v2;
    };
  };
  FvertHdlTriple() : buf{FvertHdl(0)} {}
};
/// @brief helper struct to send triplet of VertHdl
struct VertHdlTriple {

  union {
    VertHdl buf[3];

    struct {
      VertHdl v0;
      VertHdl v1;
      VertHdl v2;
    };
  };
  VertHdlTriple() : buf{VertHdl(0)} {}
  /// @brief checks if the triplet is equivalent 
  /// @param set1 
  /// @param set2 
  /// @return 1==yes 
  static int lexico_equality(const VertHdlTriple &set1,
                             const VertHdlTriple &set2) {

    return (set1.v0 == set2.v0 || set1.v0 == set2.v1 || set1.v0 == set2.v2) &&
           (set1.v1 == set2.v0 || set1.v1 == set2.v1 || set1.v1 == set2.v2) &&
           (set1.v2 == set2.v0 || set1.v2 == set2.v1 || set1.v2 == set2.v2);
  }
};
