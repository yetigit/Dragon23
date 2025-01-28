/*
    TODO ZONE
cleanup argument names because some are just like ??

*/
#pragma once

#include <GnMesh/Mesh/BlockArrayConfig.hpp>

#include <GnMeshCommon/SmallVector.h> // small vector
//#include <GnMeshCommon/bitUtils.hpp>

#include <GnMesh/Geom/GnmEigen.PCH.hpp>
#include <GnMeshCommon/IntTypes.hpp>
#include <GnMesh/Mesh/Handles.hpp>
#include <GnMesh/Mesh/Items.hpp>
#include <GnMesh/Mesh/Construct.hpp>
#include <GnMesh/Mesh/Edit.hpp>

//

/*!
 * \class BM_BlkArray
 *
 * \brief
 * \tparam
 * \note
 */
template <class BlkAllocConfigs_T> struct BM_BlkArray {

  using VertBlockArray_t = typename BlkAllocConfigs_T::VertBlockArray_t;
  using EdgeBlockArray_t = typename BlkAllocConfigs_T::EdgeBlockArray_t;
  using FaceBlockArray_t = typename BlkAllocConfigs_T::FaceBlockArray_t;
  using FVertBlockArray_t = typename BlkAllocConfigs_T::FVertBlockArray_t;

  VertBlockArray_t mVertBlock;
  EdgeBlockArray_t mEdgeBlock;
  FaceBlockArray_t mFaceBlock;
  FVertBlockArray_t mFvertBlock;

  BM_BlkArray()
      : mVertBlock(BlkAllocConfigs_T::numVertPerBlock()),
        mEdgeBlock(BlkAllocConfigs_T::numEdgePerBlock()),
        mFaceBlock(BlkAllocConfigs_T::numFacePerBlock()),
        mFvertBlock(BlkAllocConfigs_T::numFvertPerBlock()) {}

  //
};

struct GeoAttrib {

  enum {

    kInt,
    kFloat

  };
};
struct GeoAttribInfo {

  void *p = nullptr;
  int type;
  unsigned num;
};

template <class BlkAllocConfigs_T>
struct BM_BlkArrayGeo : BM_BlkArray<BlkAllocConfigs_T> {
  using Vector_t = Eigen::Vector_t;
  using Vector2_t = Eigen::Vector_t;

  std::vector<Vector_t> mPoints;
  std::vector<Vector_t> mVertNormals;
  std::vector<Vector2_t> mUVs;

  std::vector<GeoAttribInfo> attributes;

  void *allocateAttributeData(const int data_size) {
    return ::operator new(data_size);
  }

  void *createAttribute(int &handle, const int attribute_type, const void *data,
                        const int data_size, const unsigned num_element) {

    attributes.push_back(GeoAttribInfo{nullptr, attribute_type, num_element});
    attributes.back().p = allocateAttributeData(data_size);
    memcpy(attributes.back().p, data, data_size);
    handle = attributes.size() - 1;

    return attributes.back().p;
  }

  template <class T> T *getAttribute(int handle, unsigned &sz) {

    auto &ref = attributes[handle];
    sz = ref.num;
    return (T *)ref.p;
  }

  BM_BlkArrayGeo() {}
};
