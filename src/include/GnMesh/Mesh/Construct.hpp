#pragma once

//#include <GnMesh/Storage/BlockArray.hpp>
//#include <GnMeshCommon/TParam.hpp>
//
#include <GnMeshCommon/SmallVector.h> // small vector
//#include <GnMesh/Storage/LinearBlockAlloc.hpp>
////#include <GnMeshCommon/bitUtils.hpp>

#include <robin_hood.h>
#include <algorithm>
#include <GnMeshCommon/IntTypes.hpp>
#include <GnMesh/Mesh/Handles.hpp>
#include <GnMesh/Mesh/Items.hpp>
#include <GnMesh/Mesh/Queries.hpp>
#include <GnMeshCommon/BitArray.hpp>

#define DRGN_LAST_ID_AS_FIRST 0

template <class FaceBkl, class FvertBlk, class EdgeBlk, class VertBlk>
void construct_add_to_edge_cycle(const VertHdl vhdl, const EdgeHdl ehdl,
                                 FaceBkl &faceBlk, FvertBlk &fvertBlk,
                                 EdgeBlk &edgeBlk, VertBlk &vertBlk) {

  EdgeHdl ve_id = get_vert_edge(vhdl, fvertBlk,  vertBlk);
  Edge &vert_edge = edgeBlk[ve_id.id];

  // first identify which of the entry is for this vert 0 or 1
  if (vert_edge.v0 == vhdl) {

    // if the entry is valid we just want to insert between
    if (vert_edge.next0.isValid()) {

      // printf("AAAAAAA11\n");

      // now we insert between before_last and last (always firsthdl)
      EdgeHdl firsth = ve_id;
      EdgeHdl cur = firsth;
      EdgeHdl curnext = cur;

      do {

        // std::cout << "construct_add_to_edge_cycle ..\n";
        cur = curnext;
        Edge *cur_edge = &edgeBlk[cur.id];
        curnext = EdgeHdl(
            reinterpret_cast<int *>(cur_edge)[2 + int(vhdl == cur_edge->v1)]);
      } while (curnext != firsth);
      EdgeHdl last = cur;

      Edge *cur_edge = &edgeBlk[last.id];
      reinterpret_cast<int *>(cur_edge)[2 + int(vhdl == cur_edge->v1)] =
          ehdl.id;

      cur_edge = &edgeBlk[ehdl.id];
      reinterpret_cast<int *>(cur_edge)[2 + int(vhdl == cur_edge->v1)] =
          firsth.id;

    } else {
      // printf("AAAAAAA22\n");
      // assert(ve_id == ehdl);
      // assign next to it
      vert_edge.next0 = ehdl;
      // if the entry is invalid we want to create a closed cycle ,
      // so here we indirectly refer back to itself

      Edge &in_edge = edgeBlk[ehdl.id];
      // this assumes the edge type is 16 bytes and made of 4 int equivalent
      reinterpret_cast<int *>(&in_edge)[2 + int(vhdl == in_edge.v1)] = ve_id.id;
    }
  } else if (vert_edge.v1 == vhdl) {

    // assert(vert_edge.v1 == vhdl);

    if (vert_edge.next1.isValid()) {

      // printf("BBBBBBB11\n");

      // now we insert between before_last and last (always firsthdl)
      EdgeHdl firsth = ve_id;
      EdgeHdl cur = firsth;
      EdgeHdl curnext = cur;

      do {

        // std::cout << "construct_add_to_edge_cycle ..\n";

        cur = curnext;
        Edge *cur_edge = &edgeBlk[cur.id];
        curnext = EdgeHdl(
            reinterpret_cast<int *>(cur_edge)[2 + int(vhdl == cur_edge->v1)]);
      } while (curnext != firsth);
      EdgeHdl last = cur;

      Edge *cur_edge = &edgeBlk[last.id];
      reinterpret_cast<int *>(cur_edge)[2 + int(vhdl == cur_edge->v1)] =
          ehdl.id;

      cur_edge = &edgeBlk[ehdl.id];
      reinterpret_cast<int *>(cur_edge)[2 + int(vhdl == cur_edge->v1)] =
          firsth.id;

    } else {

      // printf("BBBBBBB22\n");

      // assert(ve_id == ehdl);
      // assign next to it
      vert_edge.next1 = ehdl;

      // if the entry is invalid we want to create a closed cycle ,
      // so here we indirectly refer back to itself

      Edge &in_edge = edgeBlk[ehdl.id];
      // this assumes the edge type is 16 bytes and made of 4 int equivalent
      reinterpret_cast<int *>(&in_edge)[2 + int(vhdl == in_edge.v1)] = ve_id.id;
    }
  }

  //
}

/*!
 * \brief
 *
 * \param[in] a description
 * \param[in] b description
 * \param[out] c description
 * \return description
 *
 * \note
 * \warning
 */
template <class FvertBlk>
void construct_add_to_radial_cycle(const FvertHdl fvhdl, const FvertHdl nexthdl,
                                   FvertBlk &fvertBlk) {

  FvertHdl firsth = fvhdl;
  FvertHdl cur = firsth;
  FvertHdl curnext = cur;

  do {
    cur = curnext;

    // assert(cur.isValid() && "cur bad");

    curnext = fvertBlk[cur.id].radial;
  } while (curnext != firsth);
  FvertHdl last = cur;

  fvertBlk[last.id].radial = nexthdl;
  fvertBlk[nexthdl.id].radial = firsth;

  //
}

/**
 * \brief Makes a Brep Mesh from basic index mesh data.
 * \param counts counts buffer
 * \param indices indices buffer
 * \param len_counts number of counts
 * \param len_indices number of indices
 * \param out_faceBlk empty face block array
 * \param out_fvertBlk empty fvert block array
 * \param out_edgeBlk empty edge block array
 * \param out_vertBlk empty vert block array
 */

template <class FaceBkl, class FvertBlk, class EdgeBlk, class VertBlk>
void structure_from_indices(const int* counts /* polygons vertex count like in maya, one count per polygon , in the case of a triangle mesh it is numPolygons * 3 : [ 3, 3, 3, 3, ... ] */,
    const int* indices /* vertex indices per polygons like in maya , for one triangle it will be like [0, 1, 2   ] */,
    const int len_counts /*size of the count array */, const int len_indices /*  size of the indices array */,

    FaceBkl& faceBlk, FvertBlk& fvertBlk,
    EdgeBlk& edgeBlk, VertBlk& vertBlk,

    int known_numVerts = 0 /* if you know the exact number of shared vertices this mesh has , you could spare some computation and memory */, int known_numEdges = 0 /* if you know the exact number of edges this mesh has , you could spare some computation and memory */,
    int is_manifold = 0 /* / not implemented / if the mesh is a manifold we can build it faster and make memory economy  */) {

  struct Construct_edge_face {
    FvEdgeHdl e;
    FvertHdl f;
  };
  // TODO
  //   if  *counts is null we assume we are going to create a triangle mesh
  int do_trimesh = counts == nullptr;
  using robinhood_map_t =
      robin_hood::unordered_flat_map<size_t, Construct_edge_face,
                                     VertPair_Hash>;

  robinhood_map_t vert_pair_to_edge;

  if (known_numEdges) {

    vert_pair_to_edge.reserve(known_numEdges);

  } else {

    vert_pair_to_edge.reserve(size_t(len_indices * 0.5));
  }

  BitArray vert_checks;

  if (known_numVerts) {
    vert_checks = std::move(BitArray::zeros(known_numVerts));
    vertBlk.resize(known_numVerts);

  } else {
    vert_checks = std::move(BitArray::zeros(len_indices));
    vertBlk.resize(len_indices);
  }

  int madeup_numVerts = 0;

  int pos = 0;
  int new_edge_index = 0;
  // len_counts == number of faces
  for (int i = 0; i < len_counts; ++i) {
    const int ithFace = i;
    const int ithCount = counts[i];
    // set range

    const int pos_start = pos;
    const int pos_end = pos + ithCount;

#if DRGN_LAST_ID_AS_FIRST
    // start at last pos to not do a +1 mod
    int pos_tmp = pos_end - 1;
#endif

    faceBlk.pushBack(Face{FvertHdl(pos_start)});

#if DRGN_LAST_ID_AS_FIRST

    for (; pos < pos_end; pos_tmp = pos++) {

      int cur = indices[pos_tmp];
      int cur_next = indices[pos];
#else
    for (; pos < pos_end; pos++) {

      int cur = indices[pos];
      int cur_next = indices[pos + 1 < pos_end ? pos + 1 : pos_start];

#endif

      size_t vert_pair_key = ordered_pair64(cur, cur_next);
      auto is_in_map = vert_pair_to_edge.find(vert_pair_key);

      int edge_id;
      int pos_plus_one = (pos + 1) != pos_end ? pos + 1 : pos_start;

      // does edge exists or not
      if (is_in_map == vert_pair_to_edge.end()) {
        // create first edge

        // new edge id
        edge_id = new_edge_index++;

        // new edge id
        auto fvert_edge_hdl = FVEHDL(edge_id, !, );

        // put it in the map
        vert_pair_to_edge[vert_pair_key].e = fvert_edge_hdl;
        vert_pair_to_edge[vert_pair_key].f = FvertHdl(pos);

        edgeBlk.pushBack(
            Edge{VertHdl(cur), VertHdl(cur_next), {}, {}, FvertHdl(pos)});

        // push the fverts
        fvertBlk.pushBack(Fvert{fvert_edge_hdl, FaceHdl(ithFace),
                                FvertHdl(pos_plus_one),
                                FvertHdl(pos) /* refers to self for now */});

        if (vert_checks.isBitOff(cur)) {
          ++madeup_numVerts;

          vertBlk[cur].fv.id = pos;

          vert_checks.turnBitOn(cur);
        }

        if (vert_checks.isBitOff(cur_next)) {
          ++madeup_numVerts;

          vertBlk[cur_next].fv.id = pos;

          vert_checks.turnBitOn(cur_next);
        }

        // inserts edge into its respective indices cycle
        construct_add_to_edge_cycle(VertHdl(cur), EdgeHdl{edge_id}, faceBlk,
                                    fvertBlk, edgeBlk, vertBlk);

        construct_add_to_edge_cycle(VertHdl(cur_next), EdgeHdl{edge_id},
                                    faceBlk, fvertBlk, edgeBlk, vertBlk);

      } else {

        Construct_edge_face &edge_data = is_in_map->second;

        // flip the vert relative index into the edge because we created the

        edge_data.e.v = edgeBlk[edge_data.e.id].v1.id == cur;

        fvertBlk.pushBack(Fvert{edge_data.e, FaceHdl(ithFace),
                                FvertHdl(pos_plus_one), FvertHdl(pos)});

        FvertHdl edge_fvert = edge_data.f;
        int x = fvertBlk[edge_fvert.id].radial.id;
        construct_add_to_radial_cycle(edge_fvert, FvertHdl(fvertBlk.size() - 1),
                                      fvertBlk);
      }

      //
    }
  }

  if (!known_numVerts) {
    vertBlk.resize(madeup_numVerts);
  }
}

//

template <class FaceBkl, class FvertBlk, class EdgeBlk, class VertBlk>
void structure_from_edges(const int* counts /* polygons vertex count like in maya, one count per polygon , in the case of a triangle mesh it is numPolygons * 3 : [ 3, 3, 3, 3, ... ] */,
    const int* indices /* vertex indices per polygons like in maya , for one triangle it will be like [0, 1, 2   ] */,
    const int len_counts /*size of the count array */, const int len_indices /*  size of the indices array */,

    const int * edges ,   /* per face edges */

    FaceBkl& faceBlk, FvertBlk& fvertBlk,
    EdgeBlk& edgeBlk, VertBlk& vertBlk,

    int known_numVerts = 0 /* if you know the exact number of shared vertices this mesh has , you could spare some computation and memory */, int known_numEdges = 0 /* if you know the exact number of edges this mesh has , you could spare some computation and memory */,
    int is_manifold = 0 /* / not implemented / if the mesh is a manifold we can build it faster and make memory economy  */) {

  // TODO
  //   if  *counts is null we assume we are going to create a triangle mesh
  int do_trimesh = counts == nullptr;

  BitArray vert_checks;
  BitArray edge_checks;

  if (known_numVerts) {
    vert_checks = std::move(BitArray::zeros(known_numVerts));
    vertBlk.resize(known_numVerts);

  } else {
    vert_checks = std::move(BitArray::zeros(len_indices));
    vertBlk.resize(len_indices);
  }

  if (known_numEdges) {
    edge_checks = std::move(BitArray::zeros(len_indices));
    edgeBlk.resize(known_numEdges);
  } else {
    edge_checks = std::move(BitArray::zeros(len_indices));
    edgeBlk.resize(len_indices);
  }

  struct Fvinfo_struct {
    unsigned id : 31;
    unsigned v : 1;
  };


  int madeup_numVerts = 0;
  int madeup_numEdges = 0;

  edgeBlk.defaultInit();

  int pos = 0;

  for (int i = 0; i < len_counts; ++i) {
    const int ithFace = i;
    const int ithCount = counts[i];
    // set range

    const int pos_start = pos;
    const int pos_end = pos + ithCount;

    faceBlk.pushBack(Face{FvertHdl(pos_start)});

    for (; pos < pos_end; pos++) {

      int pos_plus_one = (pos + 1) != pos_end ? pos + 1 : pos_start;

      int cur = indices[pos];
      int cur_next = indices[pos_plus_one];

      int cur_e = edges[pos];

      // auto& jim = edgeBlk[cur_e].v0;

      // does edge exists or not
      if (!edgeBlk[cur_e].v0.isValid()) {
        // create first edge

        // new edge id
        auto fvert_edge_hdl = FVEHDL(cur_e, !, );


        // put it in the map
        // vert_pair_to_edge[vert_pair_key].e = fvert_edge_hdl;
        // vert_pair_to_edge[vert_pair_key].f = FvertHdl(pos);

        edgeBlk[cur_e] =
            Edge{VertHdl(cur), VertHdl(cur_next), {}, {}, FvertHdl(pos)};

        // push the fverts
        fvertBlk.pushBack(Fvert{fvert_edge_hdl, FaceHdl(ithFace),
                                FvertHdl(pos_plus_one),
                                FvertHdl(pos) /* refers to self for now */});

        ++madeup_numEdges;
        edge_checks.turnBitOn(cur_e);

        if (vert_checks.isBitOff(cur)) {
          ++madeup_numVerts;

          vertBlk[cur].fv.id = pos;

          vert_checks.turnBitOn(cur);
        }

        if (vert_checks.isBitOff(cur_next)) {
          ++madeup_numVerts;

          vertBlk[cur_next].fv.id = pos;

          vert_checks.turnBitOn(cur_next);
        }

        // inserts edge into its respective indices cycle
        construct_add_to_edge_cycle(VertHdl(cur), EdgeHdl{cur_e}, faceBlk,
                                    fvertBlk, edgeBlk, vertBlk);

        construct_add_to_edge_cycle(VertHdl(cur_next), EdgeHdl{cur_e}, faceBlk,
                                    fvertBlk, edgeBlk, vertBlk);
      } else {


        auto fvert_edge_hdl = FVEHDL(cur_e, !, );
        fvert_edge_hdl.v = edgeBlk[cur_e].v1.id == cur;

        fvertBlk.pushBack(Fvert{fvert_edge_hdl, FaceHdl(ithFace),
                                FvertHdl(pos_plus_one), FvertHdl(pos)});

        FvertHdl edge_fvert = edgeBlk[cur_e].fv;
        construct_add_to_radial_cycle(edge_fvert, FvertHdl(fvertBlk.size() - 1),
                                      fvertBlk);
      }

      //
    }
  }

  if (!known_numVerts) {
    vertBlk.resize(madeup_numVerts);
  }

  if (!known_numEdges) {
    edgeBlk.resize(madeup_numEdges);
  }
}

//