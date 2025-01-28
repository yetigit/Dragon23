#pragma once


//#include <GnMesh/Storage/BlockArray.hpp>
#include <GnMeshCommon/TParam.hpp>

#include <GnMeshCommon/SmallVector.h> //
//#include <GnMeshCommon/bitUtils.hpp>

#include <GnMeshCommon/IntTypes.hpp>

#include <GnMesh/Mesh/Items.hpp>
#include <GnMeshCommon/Specifiers.hpp>
#include <GnMeshCommon/BitArray.hpp>

#include <fmt/core.h>
#include <fmt/ranges.h>


///


using gnm_index_buffer_t = std::vector<int>; 


/////////// SELECTION /////////////////// 


template <class EdgeBlk, class VertBlk, class FvertBlk>
EdgeHdl edge_loopsel_next(EdgeHdl ehdl, VertHdl by, EdgeBlk const &edgeBlk,
                          FvertBlk const &fvertBlk, VertBlk const &vertBlk) {

  if (vert_edge_ring(vertBlk, fvertBlk, edgeBlk, by).size() != 4) {
    return EdgeHdl{};
  }

  FvertHdl edgeFv = edgeBlk[ehdl].fv;
  if (get_v(edgeBlk, fvertBlk, edgeFv) == by) {
    FvertHdl opp = fvertBlk[edgeFv].radial;
    edgeFv = opp;
  }
  FvertHdl stepped_fv = fvertBlk[fvertBlk[fvertBlk[edgeFv].next].radial].next;
  if (get_v(edgeBlk, fvertBlk, stepped_fv) != by) {
    return EdgeHdl{};
  }
  EdgeHdl stepped_edge = fvertBlk[stepped_edge].e;

  return stepped_edge;
}


template <class EdgeBlk, class VertBlk, class FvertBlk>
void to_edge_loop(EdgeHdl e0, EdgeBlk const &edgeBlk, FvertBlk const &fvertBlk,
                  VertBlk const &vertBlk, std::vector<EdgeHdl> &selection,
                  int steps = 1) {

  VertHdl by = edgeBlk[e0].v0;
  EdgeHdl cur = e0;
  int absoluteMax = edgeBlk.size();
  if (steps == 0) {
    steps = absoluteMax;
  }

  bool comes_to_self = false;

  while (steps && absoluteMax) {
    EdgeHdl next = edge_loopsel_next(cur, by, edgeBlk, fvertBlk, vertBlk);
    if (!next.isValid()) {
      return;
    }
    if (selection.size() && next == selection[0]) {
      comes_to_self = true;
      break;
    }

    VertHdl other_by = get_other_vert(next, by, edgeBlk);
    selection.push_back(next);
    cur = next;
    by = other_by;
    --steps;
    --absoluteMax;
  }

  if (comes_to_self == false) {
      // TODO go the other way  
  }

}

///////////////  SANITY ///////////////////////

void assignSanityMsg_(std::string *sanityMsg, const char * msg ) { 
  if (sanityMsg) {
    *sanityMsg = msg;
  }
}

template <class FaceBlk, class FvertBlk , class EdgeBlk >
int sanity_check_faces(
    const FaceBlk &faceBlk, 
    const EdgeBlk & edgeBlk, 
    const FvertBlk &fvertBlk, 
    std::vector<unsigned> faceflags,
    std::vector<unsigned> vertflags,
    std::vector<unsigned> edgeflags,
    std::vector<unsigned> fvertflags,
                       std::string *sanityMsg = nullptr) {

  for (size_t i = 0; i < faceBlk.size(); i++) {

    if (ItemFlags::is_off(faceflags[i], ItemFlags::DELETED)) {

      FvertHdl fv = faceBlk[i].fv;
      if (fv.isValid() == false) {
        auto formated = fmt::format("invalid fv {}, of face {}\n", fv.id, i);
        assignSanityMsg_(sanityMsg, formated.c_str());
        return 0;
      }

      if (ItemFlags::is_on(fvertflags[fv], ItemFlags::DELETED)) {
        auto formated = fmt::format("deleted fv {}, of face {}\n", fv.id, i);
        assignSanityMsg_(sanityMsg, formated.c_str());
        return 0;
      }

      auto verts = faceverts_stack(faceBlk, fvertBlk, edgeBlk, i);
      for (auto &&cur : face_edges_stack(faceBlk, fvertBlk, i)) {
        if (ItemFlags::is_on(edgeflags[cur], ItemFlags::DELETED)) {
          auto formated = fmt::format("deleted edge {}, of face {}\n", cur.id, i);
          assignSanityMsg_(sanityMsg, formated.c_str());
          return 0;
        }
      }

      for (auto &&cur : facefv_stack(faceBlk, fvertBlk, FaceHdl(i))) {
        if (ItemFlags::is_on(fvertflags[cur], ItemFlags::DELETED)) {
          auto formated = fmt::format("deleted fv {}, of face {}\n", cur.id, i);
          assignSanityMsg_(sanityMsg, formated.c_str());
          return 0;
        }

		FaceHdl curf = fvertBlk[cur].f;

        if (curf.id != i) {
          auto formated = fmt::format("fv {} has wrong face, expected {} got {}\n", cur.id , i, curf.id, i);
          assignSanityMsg_(sanityMsg, formated.c_str());
          return 0;
        }
      }


      if (verts.size() < 3) {
        auto formated = fmt::format("vertex count under 3 for face {}\n", i);
        assignSanityMsg_(sanityMsg, formated.c_str());
        return 0;
      }

      for (auto cur : verts) {
        if (ItemFlags::is_on(vertflags[cur], ItemFlags::DELETED)) {
          auto formated =
              fmt::format("deleted vert {}, of face {}\n", cur.id, i);
          assignSanityMsg_(sanityMsg, formated.c_str());
          return 0;
        }
      }
      std::sort(verts.begin(), verts.end());
      auto uitr = std::unique(verts.begin(), verts.end());
      if (uitr != verts.end()) {
        std::vector<int> tmp_id_buffer(uitr, verts.end());
        auto formated =
            fmt::format("duplicated verts {}, of face {}\n", tmp_id_buffer, i);
        assignSanityMsg_(sanityMsg, formated.c_str());
        return 0;
      }
    }
  }
  return 1;
}

template <class FaceBlk, class FvertBlk , class EdgeBlk >
int sanity_check_radials(
    const FaceBlk &faceBlk, 
    const EdgeBlk & edgeBlk, 
    const FvertBlk &fvertBlk, 
    std::vector<unsigned> faceflags,
    std::vector<unsigned> vertflags,
    std::vector<unsigned> edgeflags,
    std::vector<unsigned> fvertflags,
                         std::string *sanityMsg = nullptr) {

  for (size_t i = 0; i < fvertBlk.size(); i++) {

    if (ItemFlags::is_off(fvertflags[i], ItemFlags::DELETED)) {

      EdgeHdl fv_edge = fvertBlk[i].e;
      auto radials = fv_radial_stack(fvertBlk, i);
      std::vector<int> faces;
      for (FvertHdl cur : radials) {

        if (ItemFlags::is_on(fvertflags[cur], ItemFlags::DELETED)) {
          auto formated = fmt::format("deleted fv {}\n", cur.id);
          assignSanityMsg_(sanityMsg, formated.c_str());
          return 0;
        }

        EdgeHdl cur_e = fvertBlk[cur].e;
        if (cur_e != fv_edge) {
          auto formated = fmt::format(
              "fv {} edge mismatch, expected {} , got {}, starting from {}\n",
              cur.id, fv_edge.id, cur_e.id, i);
          assignSanityMsg_(sanityMsg, formated.c_str());
          return 0;
        }
        faces.push_back((int)fvertBlk[cur].f);
      }

      std::sort(faces.begin(), faces.end());
      auto uitr = std::unique(faces.begin(), faces.end());
      if (uitr != faces.end()) {

        faces.erase(faces.begin(), uitr);
        auto formated =
            fmt::format("fv {} has duplicated faces {}\n",
                        i, faces);
        assignSanityMsg_(sanityMsg, formated.c_str());
        return 0;
      }
    }
  }

  return 1;
}


/////////////////// EDGE QUERIES ///////////////////


template <class VertBlk, class EdgeBlk, class FvertBlk, class FaceBlk>
bool edge_exist_on_face(VertHdl va, VertHdl vb, FaceHdl face_hdl,
                        VertBlk const &vertBlk, EdgeBlk const &edgeBlk,
                        FvertBlk const &fvertBlk, FaceBlk const &faceBlk) {

  auto common_faces = verts_common_faces(va, vb, fvertBlk, edgeBlk, vertBlk);

  if (common_faces.empty()) {
    return false;
  }
  if (face_hdl.isValid() == false) {
    face_hdl = common_faces[0];
  }

  FvertHdl first = faceBlk[face_hdl].fv;
  FvertHdl cur = first;
  FvertHdl curnext;
  do {
    EdgeHdl cur_e = fvertBlk[cur].e;
    curnext = fvertBlk[cur].next;
    VertHdl cur_v = get_v(edgeBlk, fvertBlk, cur);
    VertHdl curnext_v = get_v(edgeBlk, fvertBlk, curnext);

    bool cur_match = cur_v == va || cur_v == vb;
    bool curnext_match = curnext_v == va || curnext_v == vb;
    bool edge_match =
        edge_has_vert(cur_e, va, edgeBlk) && edge_has_vert(cur_e, vb, edgeBlk);
    if (edge_match || cur_match && curnext_match) {
      return true;
    }

  } while ((cur = curnext) != first);

  return false;
}


template <class VertBlk , class EdgeBlk , class FvertBlk>
bool edge_exist(VertHdl va, VertHdl vb, 
     VertBlk const &vertBlk, EdgeBlk const &edgeBlk,
                        FvertBlk const &fvertBlk) {
  for (EdgeHdl ehdl : vert_edge_ring(vertBlk, fvertBlk, edgeBlk, va)) {
    if (edge_has_vert(ehdl, vb, edgeBlk))
      return true;
  }
  return false;
}

// this ensures proper retrieval in non manifold case
template <class FvertBlk, class EdgeBlk>
VertHdlPair get_fvert_vpair(FvertHdl fvh, FvertBlk const &fvertBlk, EdgeBlk  const & edgeBlk) {
   
    VertHdl firstV = get_v(edgeBlk, fvertBlk, fvh); 
    auto radials = fv_radial_stack(fvertBlk, fvh) ;
    VertHdl curV; 
    for (auto cur : radials)
      if ((curV = get_v(edgeBlk, fvertBlk, cur)) != firstV)
        return VertHdlPair{firstV, curV};

    return VertHdlPair{};
}

template <class EdgeBlk>
VertHdlPair get_edge_vpair(EdgeHdl edgehdl, EdgeBlk const &edgeBlk) {
  return VertHdlPair{edgeBlk[edgehdl.id].v0, edgeBlk[edgehdl.id].v1};
}



template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
VertHdlPair edge_ccw_vert_pair(EdgeHdl ehdl,
                               FaceHdl on_face /* the face on which we look for
                                                  the oriented vert pair */
                               ,
                               const FaceBlk &faceBlk, const FvertBlk &fvertBlk,
                               const EdgeBlk &edgeBlk, const VertBlk &vertBlk) {

  FvertHdl edge_face_vtx = get_edge_fvert(ehdl, edgeBlk);

  auto facevtx_fan = fv_radial_stack(fvertBlk, edge_face_vtx);

  for (auto fvtx : facevtx_fan) {

    if (fvertBlk[fvtx.id].f == on_face) {

      VertHdl v0 = get_v(edgeBlk, fvertBlk, fvtx);
      VertHdl v1 = get_v(edgeBlk, fvertBlk, fvertBlk[fvtx.id].next);
      return VertHdlPair{v0, v1};
    }
  }

  return VertHdlPair{VertHdl(), VertHdl()};
  //
}

/*
 * \brief same as is_on_triangle but for corrupt ones
 */
template <class FvertBlk>
int is_on_two_verts_triangle(FvertHdl fvhdl, const FvertBlk &fvertBlk) {

  auto fv0 = fvhdl;
  auto fv1 = fvertBlk[fv0.id].next;
  auto fv2 = fvertBlk[fv1.id].next;
  return static_cast<int>(fv2 == fv0);
}
/// @brief chec if Fvert bounds a triangle
/// @tparam FvertBlk
/// @param fvhdl
/// @param fvertBlk
/// @return
template <class FvertBlk>
int is_on_triangle(FvertHdl fvhdl, const FvertBlk &fvertBlk) {

  auto fv0 = fvhdl;
  auto fv1 = fvertBlk[fv0.id].next;
  auto fv2 = fvertBlk[fv1.id].next;
  auto fv3 = fvertBlk[fv2.id].next;
  return static_cast<int>(fv3 == fv0);
}

template <class FvertBlk>
int is_boundary_fvert(FvertHdl fvh, const FvertBlk &fvertBlk) {
  return fvertBlk[fvh].radial == fvh;
}

/// @brief check if Edge is boundary
/// @tparam VertBlk
/// @tparam FaceBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param ehdl
/// @param faceBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk
/// @return
template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
int is_boundary_edge(EdgeHdl ehdl, const FaceBlk &faceBlk,
                     const FvertBlk &fvertBlk, const EdgeBlk &edgeBlk,
                     const VertBlk &vertBlk) {

  FvertHdl edge_fv = get_edge_fvert(ehdl, edgeBlk);

  return fvertBlk[edge_fv.id].radial == edge_fv;
}

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
int is_x_manifold_vert(VertHdl vhdl, const FaceBlk &faceBlk,
                       const FvertBlk &fvertBlk, const EdgeBlk &edgeBlk,
                       const VertBlk &vertBlk) {

  auto v_edge_ring = vert_edge_ring(vertBlk, fvertBlk, edgeBlk, vhdl);
  auto v_edge_ring_unordered =
      vert_edge_ring_unordered(vertBlk, fvertBlk, edgeBlk, vhdl);

  return v_edge_ring.size() != v_edge_ring_unordered.size();
}


template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
int is_nonmanifold_edge(EdgeHdl ehdl, const FaceBlk &faceBlk,
                        const FvertBlk &fvertBlk, const EdgeBlk &edgeBlk,
                        const VertBlk &vertBlk) {

  // FvertHdl edge_fv = get_edge_fvert(ehdl, faceBlk, fvertBlk, edgeBlk,
  // vertBlk);

  // return static_cast<int>(fvertBlk[fvertBlk[edge_fv.id].radial.id].radial !=
  //                         edge_fv);

  return fv_radial_stack(fvertBlk, edgeBlk[ehdl.id].fv).size() > 2;
}

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
int is_nonmanifold_edge2(EdgeHdl ehdl, const FaceBlk &faceBlk,
                        const FvertBlk &fvertBlk, const EdgeBlk &edgeBlk,
                        const VertBlk &vertBlk) {

  //FvertHdl edge_fv = get_edge_fvert(ehdl, faceBlk, fvertBlk, edgeBlk, vertBlk);


  //return static_cast<int>(fvertBlk[fvertBlk[edge_fv.id].radial.id].radial !=
  //                        edge_fv);

    return is_x_manifold_vert(edgeBlk[ehdl.id].v0, faceBlk, fvertBlk, edgeBlk,
                            vertBlk) ||
         is_x_manifold_vert(edgeBlk[ehdl.id].v1, faceBlk, fvertBlk, edgeBlk,
                            vertBlk);
}


/// @brief get this Edge corresponding Fvert
template <class EdgeBlk>
FvertHdl get_edge_fvert(EdgeHdl ehdl, const EdgeBlk &edgeBlk
                       ) {
  return edgeBlk[ehdl.id].fv;
}

struct EdgeAdjacencyResult {

  FvertHdl edge_fv;
  llvm_vecsmall::SmallVector<FaceHdl, 4> adj_faces;
};

template <class EdgeBlk>
VertHdlPair edge_common_verts(const EdgeBlk &edgeBlk, EdgeHdl a, EdgeHdl b) {
    // if the handle is negative it is not shared 
  VertHdl v0, v1;

  if (edgeBlk[a.id].v0 == edgeBlk[b.id].v0 ||
      edgeBlk[a.id].v0 == edgeBlk[b.id].v1

  ) {

    v0 = edgeBlk[a.id].v0;
  }

  if (edgeBlk[a.id].v1 == edgeBlk[b.id].v0 ||
      edgeBlk[a.id].v1 == edgeBlk[b.id].v1

  ) {

    v1 = edgeBlk[a.id].v1;
  }

  return VertHdlPair{v0, v1};
}

template < class FvertBlk, class EdgeBlk>
EdgeAdjacencyResult edge_adjacent_faces(EdgeHdl ehdl, 
                                        const FvertBlk &fvertBlk,
                                        const EdgeBlk &edgeBlk
                                        ) {

  EdgeAdjacencyResult result;
  FvertHdl edge_face_vertex = get_edge_fvert(ehdl, edgeBlk);

  result.edge_fv = edge_face_vertex;

  auto face_vertex_fan = fv_radial_stack(fvertBlk, edge_face_vertex);

  for (FvertHdl face_vertex :  face_vertex_fan) {
    result.adj_faces.push_back(fvertBlk[face_vertex.id].f);
  }

  return result;
}

/////////////////// VERT QUERIES ///////////////////

template <class VertBlk>
std::vector<VertHdl> get_undead_verts(VertBlk const &vertBlk,
                                      const BitArray &deadverts, int invert) {

  std::vector<VertHdl> result;

  for (int i = 0; i < vertBlk.size(); i++) {
    int bit_valid = invert ^ (int)deadverts.isBitOn(i);
    if (bit_valid) {
      result.emplace_back(i);
    }
  }

  return result;
}
/// @brief check if Vert is a border Vert
/// @tparam VertBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param vHdl
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk
/// @return
template <class VertBlk, class FvertBlk, class EdgeBlk>
int is_boundary_vert(VertHdl vHdl, FvertBlk const &fvertBlk,
                     EdgeBlk const &edgeBlk, VertBlk const &vertBlk) {

  // simplest / dumb way to do it , will make it more efficient later
  auto outgoing_fv = vert_outgoing_fverts(vertBlk, fvertBlk, edgeBlk, vHdl);
  for (auto curfv : outgoing_fv) {
    if (fvertBlk[curfv.id].radial == curfv) {
      return 1;
    }
  }

  return 0;
}

// TODO make it by just checking radials of the two (incoming and outgoing)
// fvert to the vert
template <class VertBlk, class FvertBlk, class EdgeBlk>
int vert_has_just_one_face(VertHdl vHdl, FvertBlk const &fvertBlk,
                           EdgeBlk const &edgeBlk, VertBlk const &vertBlk,
                           EdgeHdl ehdl) {

  return !vert_has_more_than_two_edges(edgeBlk, vHdl, ehdl);

  // auto outgoing_fv = vert_outgoing_fverts(vertBlk, fvertBlk, edgeBlk, vHdl);
  // return (int)(outgoing_fv.size() == 1);
}


/// @brief get all border Vert 's
/// @tparam VertBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk
/// @param invert
/// @return VertHdl *
template <class VertBlk, class FvertBlk, class EdgeBlk>
std::vector<VertHdl>
get_all_border_verts(FvertBlk const &fvertBlk, EdgeBlk const &edgeBlk,
                     VertBlk const &vertBlk, unsigned invert = 0) {

  assert(invert < 2 && "invert is either 0 or 1");
  std::vector<VertHdl> result;
  // need to collect data for an approx
  result.reserve(vertBlk.size() * 0.25);
  for (size_t i = 0; i < vertBlk.size(); i++) {
    VertHdl current_vert{(int)i};
    if (invert ^ is_boundary_vert(current_vert, fvertBlk, edgeBlk, vertBlk)) {
      result.push_back(current_vert);
    }
  }

  return result;
}

/// @brief given two Edge 's , finds a common connected Face
/// @tparam VertBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param e1hdl
/// @param e2hdl
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk
/// @return
template <class VertBlk, class FvertBlk, class EdgeBlk>
auto edge_common_face(EdgeHdl e1hdl, EdgeHdl e2hdl, FvertBlk const &fvertBlk,
                      EdgeBlk const &edgeBlk, VertBlk const &vertBlk) ->

    llvm_vecsmall::SmallVector<FaceHdl, 1> {

  auto edge_adjacency1 = edge_adjacent_faces(e1hdl, fvertBlk, edgeBlk, vertBlk);
  auto edge_adjacency2 = edge_adjacent_faces(e2hdl, fvertBlk, edgeBlk, vertBlk);

  llvm_vecsmall::SmallVector<FaceHdl, 1> mystack;

  for (size_t i = 0; i < edge_adjacency1.size(); i++) {
    for (size_t j = 0; j < edge_adjacency2.size(); j++) {

      if (edge_adjacency1.adj_faces[i] == edge_adjacency2.adj_faces[j]) {

        mystack.push_back(edge_adjacency1.adj_faces[i]);
      }
    }
  }
  return mystack;
}

// the connected face that two verts have in common
// simplest / dumb way to do it , will make it more efficient later
template <class VertBlk, class FvertBlk, class EdgeBlk>
auto verts_common_faces(VertHdl va_hdl, VertHdl vb_hdl,
                          FvertBlk const &fvertBlk, EdgeBlk const &edgeBlk,
                           VertBlk const &vertBlk) 
 -> 
llvm_vecsmall::SmallVector<FaceHdl, 2> 
{

  llvm_vecsmall::SmallVector<FaceHdl, 2> ret;

  auto vert_a_faces = vert_face_ring(vertBlk, fvertBlk, edgeBlk, va_hdl);
  auto vert_b_faces = vert_face_ring(vertBlk, fvertBlk, edgeBlk, vb_hdl);

  for (int i = 0; i < vert_a_faces.size(); i++) {
    for (int j = 0; j < vert_b_faces.size(); j++) {

      if (vert_b_faces[j] == vert_a_faces[i]) {

        ret.push_back(vert_b_faces[j]);
      }
    }
  }

  return ret;
}

template <class FvertBlk, class VertBlk>
EdgeHdl get_vert_edge(const VertHdl vhdl, const FvertBlk &fvertBlk,
                       const VertBlk &vertBlk) {

  return fvertBlk[vertBlk[vhdl.id].fv.id].e;
}

/// @brief get any Fvert starting at this Vert
/// @tparam VertBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param vhdl
/// @param vertBlk
/// @param fvertBlk
/// @param edgeBlk
/// @return
template <class VertBlk, class FvertBlk, class EdgeBlk>
GNM_INLINE FvertHdl get_vert_fv(const VertHdl vhdl, VertBlk const &vertBlk,
                                FvertBlk const &fvertBlk,
                                EdgeBlk const &edgeBlk) {

  FvertHdl vert_fv = vertBlk[vhdl.id].fv;
  VertHdl vert_fv_v = get_v(edgeBlk, fvertBlk, vert_fv);

  if (vert_fv_v != vhdl) {
    vert_fv = fvertBlk[vert_fv.id].next;
  }
  return vert_fv;
}

/// #TODO can make it better by not calling faceverts_stack
/// #TODO return a vert pair struct instead of passing two int references
template <class FaceBlk, class FvertBlk, class EdgeBlk>
void face_corner(VertHdl verthdl, FaceHdl fhdl, const FaceBlk &faceBlk,
                  const FvertBlk &fvertBlk, const EdgeBlk &edgeBlk,
                  int &previous, int &next) {

  auto face_verts = faceverts_stack(faceBlk, fvertBlk, edgeBlk, fhdl,
                                    GNM_NAMESPACE::szarg<4>());

  for (int i = 0; i < face_verts.size(); ++i) {
    if (face_verts[i] == verthdl) {
      previous = face_verts[((i - 1) + (int)face_verts.size()) %
                            (int)face_verts.size()]
                     .id;
      next = face_verts[(i + 1) % (int)face_verts.size()].id;
      break;
    }
  }
}

/////////////////// FACE QUERIES ///////////////////



template <class FaceBlk, class FvertBlk, class EdgeBlk>
VertHdlPair face_ccw_vert_pair(VertHdl va, VertHdl vb, FaceHdl on_face,
                               const FaceBlk &faceBlk, const FvertBlk &fvertBlk,
                               const EdgeBlk &edgeBlk) {

  FvertHdl first = faceBlk[on_face].fv;
  FvertHdl cur = first;
  FvertHdl curnext;
  do {
    curnext = fvertBlk[cur].next;
    VertHdl cur_v = get_v(edgeBlk, fvertBlk, cur);
    VertHdl curnext_v = get_v(edgeBlk, fvertBlk, curnext);
    bool cur_match = cur_v == va || cur_v == vb;
    bool curnext_match = curnext_v == va || curnext_v == vb;
    if (cur_match && curnext_match) {
      assert(cur_v != curnext_v);
      return VertHdlPair{cur_v, curnext_v};
    }

  } while ((cur = curnext) != first);

  return VertHdlPair{};
}


template <class FaceBlk, class FvertBlk, class EdgeBlk>
bool face_ccw_vert_pair_info(VertHdl va, VertHdl vb, FaceHdl on_face,
                               const FaceBlk &faceBlk, const FvertBlk &fvertBlk,
                               const EdgeBlk &edgeBlk,
    FvertHdl & va_prefv, 
    FvertHdl & vb_prefv, 
    bool & consecutive

) {
  va_prefv.invalidate();
  vb_prefv.invalidate();
  consecutive = false;

  FvertHdl first = faceBlk[on_face].fv;
  FvertHdl cur = first;
  FvertHdl curnext;

  do {
    curnext = fvertBlk[cur].next;
    VertHdl curnext_v = get_v(edgeBlk, fvertBlk, curnext);
    bool curnext_match = curnext_v == va || curnext_v == vb;
    if (curnext_match) {
      if (curnext_v == va) {
        va_prefv = cur;
      } else {
        vb_prefv = cur;
      }
    }

  } while ((cur = curnext) != first && (!va_prefv.isValid() || !vb_prefv.isValid()));

  if (va_prefv.isValid() && vb_prefv.isValid()) {
    if (
        get_v(edgeBlk, fvertBlk, va_prefv) == vb
        ||
        get_v(edgeBlk, fvertBlk, vb_prefv) == va
        ) {
      consecutive = true;
    }

    return true;
  }

  return false;

}

/// @brief
/// @tparam FvertBlk
/// @tparam FaceBlk
/// @param fhdl
/// @param fvertBlk
/// @param faceBlk
/// @return number of verts of given face
template <class FvertBlk, class FaceBlk>
int face_count(const FaceHdl fhdl, FvertBlk const &fvertBlk,
               FaceBlk const &faceBlk) {

  FvertHdl first = faceBlk[fhdl.id].fv;
  FvertHdl cur = first;
  int count = 0;
  do {
    ++count;
  } while ((cur = fvertBlk[cur.id].next) != first);
  return count;
}

/////////////////// FVERT QUERIES ///////////////////


/// @brief check if this Fvert is on a border
/// @tparam FvertBlk
/// @param fvertBlk
/// @param fvhdl
/// @return
template <class FvertBlk>
inline int is_border_fv(const FvertBlk &fvertBlk, const FvertHdl fvhdl) {
  return fvertBlk[fvhdl.id].radial == fvhdl;
}

template <class FvertBlk>
GNM_INLINE FvertHdl getprev_triangle(const FvertBlk &fvertBlk, FvertHdl fvhdl) {

  return fvertBlk[fvertBlk[fvhdl.id].next.id].next;
}

/// @brief get previous Fvert of this Fvert
/// which is like taking a step backward intead of forward (with Fvert.next )
/// @tparam FvertBlk
/// @param fvertBlk
/// @param fvhdl
/// @return FvertHdl
template <class FvertBlk>
inline FvertHdl getprev(const FvertBlk &fvertBlk, FvertHdl fvhdl) {
  FvertHdl first = fvhdl;
  FvertHdl cur = first;
  FvertHdl curnext;
  do {
    curnext = fvertBlk[cur.id].next;
  } while (curnext != fvhdl && (cur = curnext) != first);
  assert(cur != fvhdl);
  assert(fvertBlk[cur.id].next == fvhdl);
  return cur;
}

/// @brief get the Vert at this Fvert
/// @tparam EdgeBlk
/// @tparam FvertBlk
/// @param edgeBlk
/// @param fvertBlk
/// @param fvhdl
/// @return VertHdl
template <class EdgeBlk, class FvertBlk>
inline VertHdl get_v(const EdgeBlk &edgeBlk, const FvertBlk &fvertBlk,
                     const FvertHdl fvhdl) {
  const FvEdgeHdl fve = fvertBlk[fvhdl.id].e;
  return reinterpret_cast<const VertHdl *>(&edgeBlk[fve.id])[fve.v];
}

/////////////////// STACKS ///////////////////

/// @brief get the vertices of a triangle
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param fvhdl
/// @param fvertBlk
/// @param edgeBlk
/// @return VertHdl triplet

template <class FvertBlk, class EdgeBlk>
VertHdlTriple get_tri_v(FvertHdl fvhdl, const FvertBlk &fvertBlk,
                        const EdgeBlk &edgeBlk) {
  VertHdlTriple ret;
  FvertHdlTriple tri_fv = self_plus_two(fvertBlk, fvhdl);
  ret.v0 = get_v(edgeBlk, fvertBlk, tri_fv.v0);
  ret.v1 = get_v(edgeBlk, fvertBlk, tri_fv.v1);
  ret.v2 = get_v(edgeBlk, fvertBlk, tri_fv.v2);
  return ret;
}

/// @brief gets the full circular list of Fvert 's starting at given Fvert
/// @tparam FvertBlk
/// @param fvertBlk
/// @param fvhdl
/// @param
/// @return small vector of FvertHdl

template <class FvertBlk, size_t vcount = 4>
llvm_vecsmall::SmallVector<FvertHdl, vcount>
fv_loop_stack(const FvertBlk &fvertBlk, FvertHdl fvhdl,
              GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> =
                  GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{}) {

  llvm_vecsmall::SmallVector<FvertHdl, vcount> result;

  FvertHdl first = fvhdl;
  FvertHdl cur = first;

  do {
    result.push_back(cur);
  } while ((cur = fvertBlk[cur.id].next) != first);

  return result;
}

/// @brief get full circular list of this Fvert's radial (including it)
/// @tparam FvertBlk
/// @param fvertBlk
/// @param fvhdl
/// @param
/// @return small vector of FvertHdl

template <class FvertBlk, size_t vcount = 3>
llvm_vecsmall::SmallVector<FvertHdl, vcount>
fv_radial_stack(const FvertBlk &fvertBlk, FvertHdl fvhdl,
                GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> =
                    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{}) {

  llvm_vecsmall::SmallVector<FvertHdl, vcount> result;

  FvertHdl first = fvhdl;
  FvertHdl cur = first;

  do {
    result.push_back(cur);
  } while ((cur = fvertBlk[cur.id].radial) != first);

  return result;
}

template <class FvertBlk, size_t vcount = 3>
llvm_vecsmall::SmallVector<int, vcount>
fv_radial_stack_int(const FvertBlk &fvertBlk, FvertHdl fvhdl,
                    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> =
                        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{}) {

  llvm_vecsmall::SmallVector<int, vcount> result;

  FvertHdl first = fvhdl;
  FvertHdl cur = first;

  do {
    result.push_back(cur.id);
  } while ((cur = fvertBlk[cur.id].radial) != first);

  return result;
}

/// @brief get full circular list of this Fvert's radial (excluding it)
/// @tparam FvertBlk
/// @param fvertBlk
/// @param fvhdl
/// @param
/// @return small vector of FvertHdl

template <class FvertBlk, size_t vcount = 1>
llvm_vecsmall::SmallVector<FvertHdl, vcount>
fv_radial_stack_exclude(const FvertBlk &fvertBlk, FvertHdl fvhdl,
                        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> =
                            GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{}) {

  llvm_vecsmall::SmallVector<FvertHdl, vcount> result;

  FvertHdl first = fvhdl;
  FvertHdl cur = first;

  do {
    if (cur != fvhdl) {
      result.push_back(cur);
    }

  } while ((cur = fvertBlk[cur.id].radial) != first);

  return result;
}

/// @brief get this Fvert's radials based on predicate
/// @tparam FvertBlk
/// @tparam F
/// @param fvertBlk
/// @param fvhdl
/// @param filter_fn
/// @param
/// @return a small vector of FvertHdl

template <class FvertBlk, class F, size_t vcount = 1>
llvm_vecsmall::SmallVector<FvertHdl, vcount>
fv_radial_stack_filter(const FvertBlk &fvertBlk, FvertHdl fvhdl,
                       const F &filter_fn,
                       GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> =
                           GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{}) {

  llvm_vecsmall::SmallVector<FvertHdl, vcount> result;

  FvertHdl first = fvhdl;
  FvertHdl cur = first;

  do {
    if (filter_fn(cur)) {
      result.push_back(cur);
    }

  } while ((cur = fvertBlk[cur.id].radial) != first);

  return result;
}

template <class FvertBlk, class F, size_t vcount = 1>
llvm_vecsmall::SmallVector<FvertHdl, vcount> fv_radial_stack_exclude_and_filter(
    const FvertBlk &fvertBlk, FvertHdl fvhdl, const F &filter_fn,
    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> =
        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{}) {

  llvm_vecsmall::SmallVector<FvertHdl, vcount> result;

  FvertHdl first = fvhdl;
  FvertHdl cur = first;

  do {
    if (filter_fn(cur) && cur != fvhdl) {
      result.push_back(cur);
    }

  } while ((cur = fvertBlk[cur.id].radial) != first);

  return result;
}

/// @brief advance a Fvert 2 steps forward
/// @tparam FvertBlk
/// @param fvertBlk
/// @param fvhdl
/// @return FvertHdl triplet

template <class FvertBlk>
FvertHdlTriple self_plus_two(const FvertBlk &fvertBlk, FvertHdl fvhdl) {
  FvertHdlTriple triple;
  triple.buf[0] = fvhdl;
  triple.buf[1] = fvertBlk[fvhdl.id].next;
  triple.buf[2] = fvertBlk[fvertBlk[fvhdl.id].next.id].next;
  return triple;
}

template <class VBlk, class FvBlk, class EdgeBlk, class OutHdl, class F,
          size_t vcount = 8>
GNM_INLINE auto
vert_ringfv_stack_(const VBlk &vertBlock, const FvBlk &fvertBlock,
                   const EdgeBlk &edgeBlock, VertHdl vhdl, const F &convertFn,
                   GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> =
                       GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})

    -> llvm_vecsmall::SmallVector<OutHdl, vcount> {

  llvm_vecsmall::SmallVector<OutHdl, vcount> mystack;

  auto initbuff = vert_outgoing_fverts(vertBlock, fvertBlock, edgeBlock, vhdl,
                                       GNM_NAMESPACE::szarg<vcount * 2>());

  for (FvertHdl facevertex : initbuff)
    mystack.push_back(convertFn(fvertBlock[facevertex_j.id].next));

  return mystack;
}


struct Vertex_Edges_Result {

  llvm_vecsmall::SmallVector<EdgeHdl, 6> edges;
  llvm_vecsmall::SmallVector<FvertHdlPair, 6> edge_fvpair;
};

template <class VertBlk, class EdgeBlk, class FaceBlk, class FvertBlk>
Vertex_Edges_Result
vertex_surrounding_fverts_per_edge(VertBlk const &vertblk, EdgeBlk const &edgeblk,
                                   FaceBlk const &faceblk, FvertBlk const &fvertblk,
                                   VertHdl verthdl) {

  auto face_ring = vert_face_ring(vertblk, fvertblk, edgeblk, verthdl);

  Vertex_Edges_Result result;

  for (auto f : face_ring) {
    FvertHdl first = faceblk[f.id].fv;
    FvertHdl cur = first;

    do {
      VertHdl cur_v = get_v(edgeblk, fvertblk, cur);
      if (cur_v == verthdl) {
        EdgeHdl cur_e = fvertblk[cur.id].e;

        if (fvertblk[cur.id].radial == cur) {
          result.edges.push_back(cur_e);
          result.edge_fvpair.push_back(FvertHdlPair{cur, cur});
        } else {

          result.edges.push_back(cur_e);
          FvertHdl opposite_on_vert = fvertblk[cur.id].radial;
          opposite_on_vert = fvertblk[opposite_on_vert.id].next;
          result.edge_fvpair.push_back(FvertHdlPair{cur, opposite_on_vert});
        }

        break;
      }
      //
      cur = fvertblk[cur.id].next;
    } while (cur != first);
    //
  }

  return result;
}

template <unsigned int S>
int fv_killdouble(llvm_vecsmall::SmallVector<FvertHdl, S> &_stack) {

  std::sort(_stack.begin(), _stack.end());
  auto x = std::unique(_stack.begin(), _stack.end());
  int has_double = (x != _stack.end());
  _stack.erase(x, _stack.end());
  return has_double;
}

/// @brief get the Fvert 's on the edge at the given vert
/// @tparam EdgeBlk
/// @tparam VertBlk
/// @tparam FvertBlk
/// @tparam FaceBlk
/// @param edgeBlk
/// @param vertBlk
/// @param fvertBlk
/// @param faceBlk
/// @param ei
/// @param v
/// @return
template <class EdgeBlk, class VertBlk, class FvertBlk, class FaceBlk>
FvertHdlPair edge_manifold_fv(const EdgeBlk &edgeBlk, const VertBlk &vertBlk,
                              const FvertBlk &fvertBlk, const FaceBlk &faceBlk,
                              EdgeHdl ei, VertHdl v) {

  auto edge_adjacency = edge_adjacent_faces(ei, fvertBlk, edgeBlk);

  llvm_vecsmall::SmallVector<FvertHdl, 2> mystack;

  for (auto adjf : edge_adjacency.adj_faces) {

    auto fvcycle = facefv_stack(faceBlk, fvertBlk, adjf);
    for (auto face_vtx : fvcycle) {
      if (get_v(edgeBlk, fvertBlk, face_vtx) == v) {
        mystack.push_back(face_vtx);
      }
    }
  }

  assert(mystack.size());

  FvertHdlPair ret;
  ret.first = mystack[0];
  ret.second = mystack.size() < 2 ? ret.first : mystack[1];

  return ret;
}

template <class VBlk, class FvBlk, class EdgeBlk, class OutHdl, class F,
          size_t vcount = 8>
GNM_INLINE auto
vert_outgoing_fv_stack_(const VBlk &vertBlock, const FvBlk &fvertBlock,
                        const EdgeBlk &edgeBlock, VertHdl vhdl,
                        const F &convertFn,
                        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> =
                            GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<OutHdl, vcount> {

  llvm_vecsmall::SmallVector<OutHdl, vcount> mystack;

  auto vert_edges =
      vert_edge_ring_unordered(vertBlock, fvertBlock, edgeBlock, vhdl);
  for (size_t i = 0; i < vert_edges.size(); i++) {
    EdgeHdl cur_e = vert_edges[i];
    FvertHdl fv = get_edge_fvert(cur_e, edgeBlock);
    auto fv_radials = fv_radial_stack(fvertBlock, fv);
    for (size_t j = 0; j < fv_radials.size(); j++) {
      FvertHdl curfv = fv_radials[j];
      VertHdl fv_v = get_v(edgeBlock, fvertBlock, curfv);
      if (fv_v == vhdl) {
        mystack.push_back(convertFn(curfv));
      }
    }
  }

  return mystack;
}

#if 1
/// @brief get Vert 's connected Edge 's
/// @tparam VertBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param vertBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vhdl
/// @param vcountarg
/// @return small vector of EdgeHdl
/// @note this query has no winding order
template <class VertBlk, class FvertBlk, class EdgeBlk, size_t vcount = 8>
auto vert_edge_ring(const VertBlk &vertBlk, const FvertBlk &fvertBlk,
                    const EdgeBlk &edgeBlk, const VertHdl vhdl,
                    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
                        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<EdgeHdl, vcount> {

    return vert_edge_ring_unordered(vertBlk, fvertBlk, edgeBlk, vhdl, vcountarg);
}
template <class EdgeBlk, size_t vcount = 8>
auto vert_edge_ring_unordered(
    const EdgeBlk &edgeBlk, const VertHdl vhdl, const EdgeHdl ehdl,
    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<EdgeHdl, vcount> {

  llvm_vecsmall::SmallVector<EdgeHdl, vcount> mystack;

  EdgeHdl first = ehdl;
  EdgeHdl cur = first;

  do {
    mystack.push_back(cur);
  } while ((cur = *get_next_edge(vhdl, cur, edgeBlk)) != first);

  return mystack;
}
/// @brief same as vert_edge_ring but without winding
/// @tparam EdgeBlk
/// @param edgeBlk
/// @param vhdl
/// @param ehdl
/// @param vcountarg
/// @return EdgeHdl *
template <class VertBlk, class FvertBlk, class EdgeBlk, size_t vcount = 8>
auto vert_edge_ring_unordered(
    const VertBlk &vertBlk, const FvertBlk &fvertBlk, const EdgeBlk &edgeBlk,
    const VertHdl vhdl,
    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<EdgeHdl, vcount> {

  llvm_vecsmall::SmallVector<EdgeHdl, vcount> mystack;

  EdgeHdl first = get_vert_edge(vhdl, fvertBlk,  vertBlk);
  EdgeHdl cur = first;

  do {
    mystack.push_back(cur);
  } while ((cur = *get_next_edge(vhdl, cur, edgeBlk)) != first);

  return mystack;
}

template <class VertBlk, class FvertBlk, class EdgeBlk>
auto vert_edge_ring3(const VertBlk &vertBlk, const FvertBlk &fvertBlk,
                     const EdgeBlk &edgeBlk, const VertHdl vhdl)
    -> llvm_vecsmall::SmallVector<EdgeHdl, 3> {

  llvm_vecsmall::SmallVector<EdgeHdl, 3> mystack;

  EdgeHdl first = get_vert_edge(vhdl, fvertBlk, vertBlk);
  EdgeHdl cur = first;

  int counter = 0;
  do {
    mystack.push_back(cur);
  } while (++counter, (cur = *get_next_edge(vhdl, cur, edgeBlk)) != first &&
           counter != 3);

  return mystack;
}


/// @brief get Vert 's connected Vert 's
/// @tparam VertBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param vertBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vhdl
/// @param vcountarg
/// @return VertHdl *
template <class VertBlk, class FvertBlk, class EdgeBlk, size_t vcount = 8>
auto vert_vert_ring(const VertBlk &vertBlk, const FvertBlk &fvertBlk,
                    const EdgeBlk &edgeBlk, const VertHdl vhdl,
                    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
                        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<VertHdl, vcount> {

  llvm_vecsmall::SmallVector<VertHdl, vcount> mystack;

  auto vert_edges = vert_edge_ring_unordered(vertBlk, fvertBlk, edgeBlk, vhdl);

  for (size_t i = 0; i < vert_edges.size(); i++) {
    mystack.push_back(get_other_vert(vert_edges[i], vhdl, edgeBlk));
  }
  return mystack;
}

/// @brief get all Fvert 's starting at this Vert
/// @tparam VertBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param vertBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vhdl
/// @param vcountarg
/// @return FvertHdl *
template <class VertBlk, class FvertBlk, class EdgeBlk, size_t vcount = 8>
auto vert_outgoing_fverts(
    const VertBlk &vertBlk, const FvertBlk &fvertBlk, const EdgeBlk &edgeBlk,
    const VertHdl vhdl,
    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<FvertHdl, vcount> {

  return vert_outgoing_fv_stack_<VertBlk, FvertBlk, EdgeBlk, FvertHdl>(
      vertBlk, fvertBlk, edgeBlk, vhdl,
      [](const FvertHdl inFv) { return inFv; }, vcountarg);
}


/// @brief get Vert 's connected Face 's
/// @tparam VertBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param vertBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vhdl
/// @param vcountarg
/// @return FaceHdl *
template <class VertBlk, class FvertBlk, class EdgeBlk, size_t vcount = 8>
auto vert_face_ring(const VertBlk &vertBlk, const FvertBlk &fvertBlk,
                    const EdgeBlk &edgeBlk, const VertHdl vhdl,
                    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
                        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<FaceHdl, vcount> {

  return vert_outgoing_fv_stack_<VertBlk, FvertBlk, EdgeBlk, FaceHdl>(
      vertBlk, fvertBlk, edgeBlk, vhdl,
      [&fvertBlk](const FvertHdl inFv) { return fvertBlk[inFv.id].f; },
      vcountarg);
}

#endif

template <class FaceBlk, class FvertBlk, class Yield, class F,
          size_t vcount = 4>
GNM_INLINE auto fv_stack_(FaceBlk const &faceBlk, const FvertBlk &fvertBlk,
                          const FaceHdl fhdl, F _convertfun,
                          GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> =
                              GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<Yield, vcount> {

  llvm_vecsmall::SmallVector<Yield, vcount> mystack;
  FvertHdl first = faceBlk[fhdl.id].fv;
  FvertHdl cur = first;

  do {

    mystack.push_back(_convertfun(cur));
    cur = fvertBlk[cur.id].next;

  } while (cur != first);

  return mystack;
}

template <class EdgeBlk>
VertHdl get_other_vert(EdgeHdl e, VertHdl v, const EdgeBlk &edgeBlk) {
  auto ret = (int *)&edgeBlk[e.id];
  return VertHdl{ret[!(ret[1] == v.id)]};
}

template <class EdgeBlk>
int edge_has_vert(EdgeHdl e, VertHdl v, const EdgeBlk &edgeBlk) {
  return (int)(edgeBlk[e.id].v0 == v || edgeBlk[e.id].v1 == v);
}

/// \brief get this Face 's Vert 's
/// \return VertHdl *
template <class FaceBlk, class EdgeBlk, class FvertBlk, size_t vcount = 4>
GNM_INLINE auto
faceverts_stack(FaceBlk const &faceBlk, const FvertBlk &fvertBlk,
                EdgeBlk const &edgeBlk, const FaceHdl fhdl,
                GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
                    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<VertHdl, vcount> {

  return fv_stack_<FaceBlk, FvertBlk, VertHdl>(
      faceBlk, fvertBlk, fhdl,
      [&edgeBlk, &fvertBlk](const FvertHdl fv) {
        return get_v(edgeBlk, fvertBlk, fv);
      },
      vcountarg);
}

/// \brief get this Face 's Fvert 's
/// \return FvertHdl *
template <class FaceBlk, class FvertBlk, size_t vcount = 4>
GNM_INLINE auto
facefv_stack(FaceBlk const &faceBlk, const FvertBlk &fvertBlk,
             const FaceHdl fhdl,
             GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
                 GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<FvertHdl, vcount> {

  return fv_stack_<FaceBlk, FvertBlk, FvertHdl>(
      faceBlk, fvertBlk, fhdl, [](const FvertHdl fv) { return fv; }, vcountarg);
}

template <class FaceBlk, class FvertBlk, size_t vcount = 4>
GNM_INLINE auto
facefv_stack(FaceBlk const &faceBlk, const FvertBlk &fvertBlk,
             const FvertHdl fvhdl,
             GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
                 GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<FvertHdl, vcount> {

  llvm_vecsmall::SmallVector<FvertHdl, vcount> ret;

  FvertHdl first = fvhdl;
  FvertHdl cur = first;
  do {
    ret.push_back(cur);
  } while ((cur = fvertBlk[cur.id].next) != first);
  return ret;
}

template <class FaceBlk, class FvertBlk, size_t vcount = 4>
GNM_INLINE auto facefv_stack_radial_1_int(
    FaceBlk const &faceBlk, const FvertBlk &fvertBlk, const FaceHdl fhdl,
    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<int, vcount> {

  return fv_stack_<FaceBlk, FvertBlk, int>(
      faceBlk, fvertBlk, fhdl,
      [&fvertBlk](const FvertHdl fv) { return fvertBlk[fv.id].radial.id; },
      vcountarg);
}
template <class FaceBlk, class FvertBlk, size_t vcount = 4>
GNM_INLINE auto facefv_stack_radial_2_int(
    FaceBlk const &faceBlk, const FvertBlk &fvertBlk, const FaceHdl fhdl,
    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<int, vcount> {

  return fv_stack_<FaceBlk, FvertBlk, int>(
      faceBlk, fvertBlk, fhdl,
      [&fvertBlk](const FvertHdl fv) {
        return fvertBlk[fvertBlk[fv.id].radial.id].radial.id;
      },
      vcountarg);
}

template <class FaceBlk, class FvertBlk, size_t vcount = 4>
GNM_INLINE auto facefv_stack_radial_3_int(
    FaceBlk const &faceBlk, const FvertBlk &fvertBlk, const FaceHdl fhdl,
    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<int, vcount> {

  return fv_stack_<FaceBlk, FvertBlk, int>(
      faceBlk, fvertBlk, fhdl,
      [&fvertBlk](const FvertHdl fv) {
        return fvertBlk[fvertBlk[fvertBlk[fv.id].radial.id].radial.id]
            .radial.id;
      },
      vcountarg);
}

struct LaminaPrep_Data {

  static constexpr int expected_maxsz = 8;

  using sepvec_type = llvm_vecsmall::SmallVector<int, expected_maxsz>;
  using datavec_type = llvm_vecsmall::SmallVector<FvertHdl, expected_maxsz>;

  llvm_vecsmall::SmallVector<FvertHdl, expected_maxsz> data;
};


template <class FaceBlk, class FvertBlk, size_t vcount = 4>
GNM_INLINE auto
facefv_stack_int(FaceBlk const &faceBlk, const FvertBlk &fvertBlk,
                 const FaceHdl fhdl,
                 GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
                     GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<int, vcount> {

  return fv_stack_<FaceBlk, FvertBlk, int>(
      faceBlk, fvertBlk, fhdl, [](const FvertHdl fv) { return fv.id; },
      vcountarg);
}


template <class FaceBlk, class EdgeBlk, class FvertBlk,  class VertBlk , size_t vcount = 4>
GNM_INLINE auto
faceface_rich_stack(
    FaceBlk const &faceBlk, const FvertBlk &fvertBlk, EdgeBlk const &edgeBlk,  const VertBlk  & vertBlk  , 
    const FaceHdl fhdl,
    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{}) 
    -> llvm_vecsmall::SmallVector<FaceHdl, 16>

{

    llvm_vecsmall::SmallVector<FaceHdl, 16> ret; 

    auto vertices = faceverts_stack(faceBlk, fvertBlk, edgeBlk, fhdl, vcountarg);

    for (auto v : vertices) {
      auto adjacent_faces = vert_face_ring(vertBlk, fvertBlk, edgeBlk, v, vcountarg);
      for (auto f : adjacent_faces) {
        ret.push_back(f);
      }
    }


    std::sort(ret.begin(), ret.end());
    auto u = std::unique(ret.begin(), ret.end());
    ret.erase(u, ret.end());

    auto r = std::remove(ret.begin(), ret.end(), fhdl);
    ret.erase(r, ret.end());

    return ret;

}

/// \brief get this Face 's  adjacent Face 's (by edges)
/// \return FaceHdl *
template <class FaceBlk, class EdgeBlk, class FvertBlk, size_t vcount = 4>
GNM_INLINE auto
faceface_stack(FaceBlk const &faceBlk, const FvertBlk &fvertBlk,
               EdgeBlk const &edgeBlk, const FaceHdl fhdl,
               GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
                   GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<FaceHdl, vcount> {

  return fv_stack_<FaceBlk, FvertBlk, FaceHdl>(
      faceBlk, fvertBlk, fhdl,
      [&fvertBlk](const FvertHdl fv) -> FaceHdl {
        FvertHdl radial = fvertBlk[fv.id].radial;
        if (radial == fv) {
          return FaceHdl();
        } else {
          return fvertBlk[radial.id].f;
        }
        return FaceHdl();
      },
      vcountarg);
}

template <class FaceBlk, class EdgeBlk, class FvertBlk, size_t vcount = 4>
GNM_INLINE auto
faceverts_stack_int(FaceBlk const &faceBlk, const FvertBlk &fvertBlk,
                    EdgeBlk const &edgeBlk, const FaceHdl fhdl,
                    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
                        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<int, vcount> {

  return fv_stack_<FaceBlk, FvertBlk, int>(
      faceBlk, fvertBlk, fhdl,
      [&edgeBlk, &fvertBlk](const FvertHdl fv) {
        return get_v(edgeBlk, fvertBlk, fv).id;
      },
      vcountarg);
}



template <class FaceBlk, class EdgeBlk, class FvertBlk, size_t vcount = 4>
GNM_INLINE
    std::tuple<gnm_index_buffer_t, gnm_index_buffer_t, gnm_index_buffer_t>
    get_mesh_indices(FaceBlk const &faceBlk, const FvertBlk &fvertBlk,
                     EdgeBlk const &edgeBlk,
                     GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
                         GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{}) {

  std::tuple<gnm_index_buffer_t, gnm_index_buffer_t, gnm_index_buffer_t> ret;

  int tot_v = 0;
  for (size_t i = 0; i < faceBlk.size(); i++) {
    tot_v += face_count(i, fvertBlk, faceBlk);
  }

  gnm_index_buffer_t ids;
  gnm_index_buffer_t cnt;
  gnm_index_buffer_t offset;
  ids.resize(tot_v);
  cnt.resize(faceBlk.size());
  offset.resize(faceBlk.size());

  int pvi = 0;
  for (size_t i = 0; i < faceBlk.size(); i++) {
    auto face_vertices =
        faceverts_stack_int(faceBlk, fvertBlk, edgeBlk, i, vcountarg);

    memcpy(ids.data() + pvi, face_vertices.data(),
           sizeof(int) * face_vertices.size());

    offset[i] = pvi;
    cnt[i] = face_vertices.size();

    pvi += face_vertices.size();
  }

  std::get<0>(ret) = std::move(cnt);
  std::get<1>(ret) = std::move(ids);
  std::get<2>(ret) = std::move(offset);

  return ret;
}


/// \brief get this Face 's Edge 's
/// \return EdgeHdl *
template <class FaceBlk, class FvertBlk, size_t vcount = 4>
GNM_INLINE auto
face_edges_stack(FaceBlk const &faceBlk, const FvertBlk &fvertBlk,
                 const FaceHdl fhdl,
                 GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg =
                     GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{})
    -> llvm_vecsmall::SmallVector<EdgeHdl, vcount> {

  return fv_stack_<FaceBlk, FvertBlk, EdgeHdl>(
      faceBlk, fvertBlk, fhdl,
      [&fvertBlk](const FvertHdl fv) { return fvertBlk[fv.id].e; }, vcountarg);
}

/////////////////// ITERATION ///////////////////



////  DEPRECATED
//template <class Cont>
//GNM_INLINE FvertHdl vv_next(const Cont &cont, const FvertHdl fvhdl) {
//  FvertHdl accross = cont[fvhdl.id].radial;
//  return accross != fvhdl ? cont[accross.id].next : fvhdl;
//}
//
//// DEPRECATED
//template <class Cont>
//GNM_INLINE FvertHdlPair vv_next_byprev(const Cont &cont, const FvertHdl fvhdl) {
//  FvertHdl prev = getprev(cont, fvhdl);
//  FvertHdl across = cont[prev.id].radial;
//  return FvertHdlPair{across, prev};
//}



/// @brief check if the mesh is not all-tris
/// @tparam FaceBlk
/// @tparam FvertBlk
/// @param faceBlk
/// @param fvertBlk
/// @return
template <class FaceBlk, class FvertBlk>
int is_not_trimesh(const FaceBlk &faceBlk, const FvertBlk &fvertBlk) {

  for (int i = 0; i < faceBlk.size(); ++i) {
    if (!is_on_triangle(faceBlk[i].fv, fvertBlk)) {
      return 1;
    }
  }

  return 0;
}

/// @brief get the opposite vert to the one this Fvert has
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param edgeBlk
/// @param fvertBlk
/// @param fvh
/// @return
template <class FvertBlk, class EdgeBlk>
VertHdl get_other_vert(const EdgeBlk &edgeBlk, const FvertBlk &fvertBlk,
                       FvertHdl fvh) {

  const Edge &edge = edgeBlk[fvertBlk[fvh.id].e.id];
  VertHdl fv_v = get_v(edgeBlk, fvertBlk, fvh);
  assert(edge.v0 == fv_v || edge.v1 == fv_v);
  VertHdl ret;
  ret.id = ((int *)&edge)[!(fv_v == edge.v1)];
  return ret;
}


/// @brief check if Vert has more than two connected Edge 's
/// @tparam EdgeBlk
/// @param edgeBlk
/// @param vertHdl
/// @param ehdl
/// @return
template <class EdgeBlk>
int vert_has_more_than_two_edges(const EdgeBlk &edgeBlk, VertHdl vertHdl,
                                 EdgeHdl ehdl) {
  return vert_edge_ring_unordered(edgeBlk, vertHdl, ehdl).size() > 2;
}

template <class EdgeBlk, class Streamo>
void edge_info(EdgeBlk const &edgeBlk, Streamo &s) {
  for (int i = 0; i < edgeBlk.size(); ++i) {
    s << "edge " << i << " : " << edgeBlk[i].v0.id << " <--> "
      << edgeBlk[i].v1.id << " : " << edgeBlk[i].next0.id << ", "
      << edgeBlk[i].next1.id << " : "
      << edgeBlk[i].fv.id <<
        '\n';
  }
  s << '\n';
}



template <class FvertBlk, class Streamo>
void fvert_info(FvertBlk const &fvertBlk, FvertHdl fv, int radcycle,
                int nextcycle, Streamo &sout) {

  sout << ".face " << fvertBlk[fv.id].e.id << "\n";
  sout << ".edge " << fvertBlk[fv.id].f.id << "\n";
  sout << ".edge_v " << fvertBlk[fv.id].e.v << "\n";
  sout << ".radial " << fvertBlk[fv.id].radial.id << "\n";
  sout << ".next " << fvertBlk[fv.id].next.id << "\n";
  sout << "\n";

  if (radcycle) {
    FvertHdl first = fv;
    FvertHdl cur = first;
    std::cout << ".rad cycle\n";
    do {
      std::cout << cur.id << '\n';
    } while ((cur = fvertBlk[cur.id].radial) != first);
  }

  if (nextcycle) {
    FvertHdl first = fv;
    FvertHdl cur = first;
    std::cout << ".next cycle\n";
    do {
      std::cout << cur.id << '\n';
    } while ((cur = fvertBlk[cur.id].next) != first);
  }
  if (nextcycle && radcycle) {
    FvertHdl accross = fvertBlk[fv.id].radial;
    FvertHdl resu = (accross != fv) ? fvertBlk[accross.id].next : fv;
    std::cout << ".radial->next " << resu.id << '\n';
  }
}

template <class EdgeBlk>
EdgeHdl const *get_next_edge(VertHdl v, EdgeHdl e, const EdgeBlk &edgeBlk) {

  return (const EdgeHdl *)((const int *)(&edgeBlk[e.id]) +
                           ((int)(v == edgeBlk[e.id].v1) + 2));
}

template <class EdgeBlk>
EdgeHdl *get_next_edge(VertHdl v, EdgeHdl e, EdgeBlk &edgeBlk) {

  return (EdgeHdl *)((int *)(&edgeBlk[e.id]) +
                     ((int)(v == edgeBlk[e.id].v1) + 2));
}

template <class EdgeBlk>
EdgeHdl const *get_next_edge_check(VertHdl v, EdgeHdl e,
                                   const EdgeBlk &edgeBlk) {

  if (v == edgeBlk[e.id].v0) {
    return (const EdgeHdl *)((const int *)(&edgeBlk[e.id]) + 2);
  } else if (v == edgeBlk[e.id].v1) {
    return (const EdgeHdl *)((const int *)(&edgeBlk[e.id]) + 3);
  } else {
    return nullptr;
  }
  return nullptr;
}

template <class EdgeBlk>
EdgeHdl *get_next_edge_check(VertHdl v, EdgeHdl e, EdgeBlk &edgeBlk) {

  if (v == edgeBlk[e.id].v0) {
    return (EdgeHdl *)((int *)(&edgeBlk[e.id]) + 2);
  } else if (v == edgeBlk[e.id].v1) {
    return (EdgeHdl *)((int *)(&edgeBlk[e.id]) + 3);
  } else {
    return nullptr;
  }
  return nullptr;
}
