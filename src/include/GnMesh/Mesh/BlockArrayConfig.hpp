#pragma once

#include <GnMesh/Storage/BlockArray.hpp>
#include <GnMesh/Storage/LinearBlockAlloc.hpp>
#include <GnMeshCommon/bitUtils.hpp>
#include <GnMesh/Mesh/Items.hpp>

struct MeshStoreConfig {

  unsigned numVertPerBlock, numVertBlock;

  unsigned numEdgePerBlock, numEdgeBlock;

  unsigned numFacePerBlock, numFaceBlock;

  unsigned numFvertPerBlock, numFvertBlock;
};
/*!
 * \class BM_BlkArray
 *
 * \brief
 * \tparam
 * \note
 */
struct BlkAllocConfig01 {

  struct IVert {};
  struct IEdge {};
  struct IFace {};
  struct IFvert {};

  using VertBlockArray_t = GNM_NAMESPACE::BlockArray<
      Vert, alignof(Vert),
      GNM_NAMESPACE::LinearBlockAlloc<BlkAllocConfig01::IVert>>;

  using EdgeBlockArray_t = GNM_NAMESPACE::BlockArray<
      Edge, alignof(Edge),
      GNM_NAMESPACE::LinearBlockAlloc<BlkAllocConfig01::IEdge>>;
  using FaceBlockArray_t = GNM_NAMESPACE::BlockArray<
      Face, alignof(Face),
      GNM_NAMESPACE::LinearBlockAlloc<BlkAllocConfig01::IFace>>;

  using FVertBlockArray_t = GNM_NAMESPACE::BlockArray<
      Fvert, alignof(Fvert),
      GNM_NAMESPACE::LinearBlockAlloc<BlkAllocConfig01::IFvert>>;

  static void init(MeshStoreConfig in_config) {

#ifndef GNM_BLK_ARR_NONUDGE

    in_config.numVertPerBlock =
        GNM_NAMESPACE::upperPowerOfTwo(in_config.numVertPerBlock);
    in_config.numVertBlock =
        GNM_NAMESPACE::upperPowerOfTwo(in_config.numVertBlock);

    in_config.numEdgePerBlock =
        GNM_NAMESPACE::upperPowerOfTwo(in_config.numEdgePerBlock);
    in_config.numEdgeBlock =
        GNM_NAMESPACE::upperPowerOfTwo(in_config.numEdgeBlock);
    in_config.numFvertPerBlock =
        GNM_NAMESPACE::upperPowerOfTwo(in_config.numFvertPerBlock);
    in_config.numFvertBlock =
        GNM_NAMESPACE::upperPowerOfTwo(in_config.numFvertBlock);

    in_config.numFacePerBlock =
        GNM_NAMESPACE::upperPowerOfTwo(in_config.numFacePerBlock);
    in_config.numFaceBlock =
        GNM_NAMESPACE::upperPowerOfTwo(in_config.numFaceBlock);

#endif
    GNM_NAMESPACE::LinearBlockAlloc<BlkAllocConfig01::IVert>::init(
        {static_cast<unsigned>(in_config.numVertPerBlock * sizeof(Vert)),
         in_config.numVertBlock});
    GNM_NAMESPACE::LinearBlockAlloc<BlkAllocConfig01::IEdge>::init(
        {static_cast<unsigned>(in_config.numEdgePerBlock * sizeof(Edge)),
         in_config.numEdgeBlock});
    GNM_NAMESPACE::LinearBlockAlloc<BlkAllocConfig01::IFace>::init(
        {static_cast<unsigned>(in_config.numFacePerBlock * sizeof(Face)),
         in_config.numFaceBlock});
    GNM_NAMESPACE::LinearBlockAlloc<BlkAllocConfig01::IFvert>::init(
        {static_cast<unsigned>(in_config.numFvertPerBlock * sizeof(Fvert)),
         in_config.numFvertBlock});
  }

  static void destroy() {
  
    GNM_NAMESPACE::LinearBlockAlloc<BlkAllocConfig01::IVert>::deinit(
     
     );
    GNM_NAMESPACE::LinearBlockAlloc<BlkAllocConfig01::IEdge>::deinit(
       
      );
    GNM_NAMESPACE::LinearBlockAlloc<BlkAllocConfig01::IFace>::deinit(
        
        );
    GNM_NAMESPACE::LinearBlockAlloc<BlkAllocConfig01::IFvert>::deinit(
        
        );
  
  }

  static unsigned numVertPerBlock() noexcept {
    return GNM_NAMESPACE::LinearBlockAlloc<BlkAllocConfig01::IVert>::get()
               ->mConfig.blockSize /
           sizeof(Vert);
  }
  static unsigned numEdgePerBlock() noexcept {
    return GNM_NAMESPACE::LinearBlockAlloc<BlkAllocConfig01::IEdge>::get()
               ->mConfig.blockSize /
           sizeof(Edge);
  }
  static unsigned numFacePerBlock() noexcept {
    return GNM_NAMESPACE::LinearBlockAlloc<BlkAllocConfig01::IFace>::get()
               ->mConfig.blockSize /
           sizeof(Face);
  }

  static unsigned numFvertPerBlock() noexcept {
    return GNM_NAMESPACE::LinearBlockAlloc<BlkAllocConfig01::IFvert>::get()
               ->mConfig.blockSize /
           sizeof(Fvert);
  }

  //
};

template <class IVertT, class IEdgeT, class IFaceT, class IFvertT>
struct BlkAllocConfigX {

  using VertBlockArray_t =
      GNM_NAMESPACE::BlockArray<Vert, alignof(Vert),
                                GNM_NAMESPACE::LinearBlockAlloc<IVertT>>;

  using EdgeBlockArray_t =
      GNM_NAMESPACE::BlockArray<Edge, alignof(Edge),
                                GNM_NAMESPACE::LinearBlockAlloc<IEdgeT>>;
  using FaceBlockArray_t =
      GNM_NAMESPACE::BlockArray<Face, alignof(Face),
                                GNM_NAMESPACE::LinearBlockAlloc<IFaceT>>;

  using FVertBlockArray_t =
      GNM_NAMESPACE::BlockArray<Fvert, alignof(Fvert),
                                GNM_NAMESPACE::LinearBlockAlloc<IFvertT>>;

  static void init(const MeshStoreConfig &in_config) {
    GNM_NAMESPACE::LinearBlockAlloc<IVertT>::init(
        {static_cast<unsigned>(in_config.numVertPerBlock * sizeof(Vert)),
         in_config.numVertBlock});
    GNM_NAMESPACE::LinearBlockAlloc<IEdgeT>::init(
        {static_cast<unsigned>(in_config.numEdgePerBlock * sizeof(Edge)),
         in_config.numEdgeBlock});
    GNM_NAMESPACE::LinearBlockAlloc<IFaceT>::init(
        {static_cast<unsigned>(in_config.numFacePerBlock * sizeof(Face)),
         in_config.numFaceBlock});
    GNM_NAMESPACE::LinearBlockAlloc<IFvertT>::init(
        {static_cast<unsigned>(in_config.numFvertPerBlock * sizeof(Fvert)),
         in_config.numFvertBlock});
  }

  static unsigned numVertPerBlock() noexcept {
    return GNM_NAMESPACE::LinearBlockAlloc<IVertT>::get()->mConfig.blockSize /
           sizeof(Vert);
  }
  static unsigned numEdgePerBlock() noexcept {
    return GNM_NAMESPACE::LinearBlockAlloc<IEdgeT>::get()->mConfig.blockSize /
           sizeof(Edge);
  }
  static unsigned numFacePerBlock() noexcept {
    return GNM_NAMESPACE::LinearBlockAlloc<IFaceT>::get()->mConfig.blockSize /
           sizeof(Face);
  }

  static unsigned numFvertPerBlock() noexcept {
    return GNM_NAMESPACE::LinearBlockAlloc<IFvertT>::get()->mConfig.blockSize /
           sizeof(Fvert);
  }

  //
};
