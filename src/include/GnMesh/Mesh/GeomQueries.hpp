#pragma once

//#include <GnMesh/Storage/BlockArray.hpp>
#include <GnMeshCommon/TParam.hpp>

#include <GnMeshCommon/SmallVector.h> //
//#include <GnMeshCommon/bitUtils.hpp>

#include <GnMeshCommon/IntTypes.hpp>

#include <GnMesh/Mesh/Items.hpp>
#include <GnMeshCommon/Specifiers.hpp>
#include <GnMeshCommon/BitArray.hpp>

#include <GnMesh/Geom/GnmEigen.PCH.hpp>
#include <vector>

namespace {

using Vector_t = Eigen::Vector_t;
using Vector4_t = Eigen::Vector4_t;
using Vector2_t = Eigen::Vector2_t;
using Mat33_t = Eigen::Mat33_t;
} // namespace
//////// NORMAL

/////////////////////////

bool is_concave_part(Vector_t normal, Vector_t ij, Vector_t jk) {
     return ij.cross(jk).dot(normal) < 0.; 
}


bool is_point_in_tris(Vector_t ij , Vector_t jk, Vector_t Z) {
  Mat33_t basis;
  auto crossed = ij.cross(jk);
  ij = -ij;
  basis.col(0) = ij;
  basis.col(1) = jk;
  basis.col(2) = crossed;
  Mat33_t  basis_inv = basis.inverse();
  
  Vector_t scales = basis_inv * Z.transpose();

  float s = scales[0]; 
  float t = scales[1]; 
  float w = scales[2]; 
  float scale_sum = s + t; 

  bool in_plane = abs(w) < 1e-5f; 
  bool s_inbound = s >= 0.f && s <= 1.f ; 
  bool t_inbound = t >= 0.f && t <= 1.f ; 
  bool sum_inbound = scale_sum <= 1.f;

  if (in_plane && s_inbound && t_inbound && sum_inbound) {
    return true;
  } 

  return false;

}


template <class FaceBlk, class EdgeBlk, class FvertBlk, class VertBlk>
bool gather_ear_clip_triangles(FaceHdl fhdl, FaceBlk &faceBlk, EdgeBlk &edgeBlk,
                               FvertBlk &fvertBlk, VertBlk &vertBlk,
                               const std::vector<Vector_t> &points,
                               std::vector<unsigned> &triangle_list) {

  struct LocalIndex {
    int glob;
    int loc;
  };

  auto face_vertices_ = faceverts_stack_int(faceBlk, fvertBlk, edgeBlk, fhdl);
  llvm_vecsmall::SmallVector<LocalIndex, 5> face_vertices;
  face_vertices.reserve(face_vertices_.size());
  for (int i = 0; i < face_vertices_.size(); ++i) {
    face_vertices.push_back({face_vertices_[i], i});
  }

  // std::map<int, int> mapo;
  // for (size_t i = 0; i < face_vertices.size(); i++) {
  //   mapo[face_vertices[i]] = i;
  // }

  if (face_vertices.size() < 4)
    return false;

  struct AngleId {
    float ang;
    int id;
  };

  std::vector<AngleId> angles;

  auto l_doangles = [&points](llvm_vecsmall::SmallVector<LocalIndex, 5> &fbuf,
                              std::vector<AngleId> &angles_) {
    angles_.clear();
    for (int j = 0; j < fbuf.size(); j++) {

      int i = j ? j - 1 : fbuf.size() - 1;

      int k = (j + 1) % fbuf.size();

      int vi = fbuf[i].glob;
      int vj = fbuf[j].glob;
      int vk = fbuf[k].glob;

      auto &anchor = points[vj];
      Vector_t a = points[vi] - anchor;
      Vector_t b = points[vk] - anchor;

      float dot_ = a.dot(b);
      auto cross = a.cross(b);
      float phi = std::atan2(cross.norm(), dot_);
      angles_.push_back({phi, j});
    }
  };

  l_doangles(face_vertices, angles);

  std::sort(angles.begin(), angles.end(),
            [](auto a, auto b) { return a.ang < b.ang; });

  auto normal =
      get_face_normal(fhdl, faceBlk, fvertBlk, edgeBlk, vertBlk, points.data());

  int polyNumTriangle_expected = face_vertices.size() - 2;
  int num_tri = 0;

  int angoffset = 0;

  for (; face_vertices.size();) {
    if (num_tri >= polyNumTriangle_expected) {
      break;
    }
    if (angoffset >= face_vertices.size()) {
      return false;
    }

    int indexof = angles[angoffset].id;

    const int j = indexof;
    const int i = j ? j - 1 : face_vertices.size() - 1;
    const int k = (j + 1) % face_vertices.size();

    const int vi = face_vertices[i].glob;
    const int vj = face_vertices[j].glob;
    const int vk = face_vertices[k].glob;

    const int vi_local = face_vertices[i].loc;
    const int vj_local = face_vertices[j].loc;
    const int vk_local = face_vertices[k].loc;

    auto &anchor = points[vj];
    Vector_t ij = points[vj] - points[vi];
    Vector_t jk = points[vk] - points[vj];
    bool is_convex = !is_concave_part(normal, ij, jk);
    if (is_convex) {
      int num_inside = 0;
      for (size_t x = 0; x < face_vertices.size(); x++) {
        int vx = face_vertices[x].glob;
        if (vx != vi && vx != vj && vx != vk) {
          auto &coordinate = points[vx];
          auto Z = coordinate - anchor;

          num_inside += (int)is_point_in_tris(ij, jk, Z);
        }
      }

      if (num_inside == 0) {
        if (vi != vk) {
          triangle_list.push_back(vi_local);
          triangle_list.push_back(vj_local);
          triangle_list.push_back(vk_local);
          ++num_tri;
        }

        auto findv = face_vertices.end();

        for (auto it = face_vertices.begin(); it != face_vertices.end(); ++it)
          if (it->glob == vj) {
            findv = it;
            break;
          }

        if (findv != face_vertices.end()) {
          face_vertices.erase(findv);
          angles.erase(angles.begin() + indexof);
        }

        l_doangles(face_vertices, angles);
        std::sort(angles.begin(), angles.end(),
                  [](auto a, auto b) { return a.ang < b.ang; });

        angoffset = 0;

      } else {
        ++angoffset;
      }
    } else {
      ++angoffset;
    }
  }

  return true;
}



//
// method adapted from https://help.autodesk.com/view/MAYAUL/2017/ENU/?guid=__files_GUID_052EC6C3_BFF1_42E3_B4A5_1725700E3428_htm

void quad_split_solver(int v0, int v1, int v2, int v3, 
    std::vector<Eigen::Vector_t> &pointsRef, int * two_triangles) {


  float fudgeFactor = 0.999f;
  bool canSplit = true;
  bool split02 = false;

  auto vec02 = pointsRef[v2] - pointsRef[v0];
  auto vec01 = pointsRef[v1] - pointsRef[v0];

  auto vec13 = pointsRef[v3] - pointsRef[v1];
  auto vec03 = pointsRef[v3] - pointsRef[v0];

  // triangle 1, 2, 0;
  // triangle 2, 3, 0;

  float a1 = abs(vec02.cross(vec01).norm() / 2.);
  float a2 = abs(vec02.cross(vec03).norm() / 2.);

  float dist02 = vec02.norm();
  float dist13 = vec13.norm();

  if (dist02 < fudgeFactor * dist13) {

    if (a1 < 7 * a2 && a2 < 7 * a1) {
      split02 = true;
    }
  }


  if (split02) {

    two_triangles[0] = 1;
    two_triangles[1] = 2;
    two_triangles[2] = 0;

    two_triangles[3] = 0;
    two_triangles[4] = 2;
    two_triangles[5] = 3;

  } else {

    two_triangles[0] = 0;
    two_triangles[1] = 1;
    two_triangles[2] = 3;

    two_triangles[3] = 3;
    two_triangles[4] = 1;
    two_triangles[5] = 2;
  }


}



//
template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
void fan_triangulation(FaceBlk &faceBlk, FvertBlk &fvertBlk, EdgeBlk &edgeBlk,
                       VertBlk &vertBlk, FaceHdl fhdl, std::vector<unsigned> & triangles, bool dry_run, std::vector<unsigned>  & face_map ) {

  auto face_vertices = faceverts_stack(faceBlk, fvertBlk, edgeBlk, fhdl);
  int vcount = face_vertices.size();
  int numtri = 0;

  const int v0 = face_vertices[0];
  for (int i = 2; i < face_vertices.size() - 1; ++i) {


    if (!dry_run) {
    (void)vert_vert_connect(v0, face_vertices[i], FaceHdl(), faceBlk, fvertBlk,
                            edgeBlk, vertBlk);
    }


    triangles.push_back(0);
    triangles.push_back(i-1);
    triangles.push_back(i);
    face_map.push_back(fhdl.id); 
    face_map.push_back(fhdl.id); 
    face_map.push_back(fhdl.id); 
    ++numtri;

  }


  // last triangle
  if (numtri != (vcount - 2)) {
    int vi = 0;
    int vk = face_vertices.size() - 1;
    int vj = vk - 1;

    triangles.push_back(vi);
    triangles.push_back(vj);
    triangles.push_back(vk);
    face_map.push_back(fhdl.id);
    face_map.push_back(fhdl.id);
    face_map.push_back(fhdl.id);

    ++numtri; // hehe
  }



}


//
// 
template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
std::vector<unsigned>
general_triangulation(FaceBlk &faceBlk, FvertBlk &fvertBlk, EdgeBlk &edgeBlk,
                      VertBlk &vertBlk, std::vector<Eigen::Vector_t> &pointsRef,
                      bool dry_run, std::vector<unsigned> &face_map

) {

  int numFace = faceBlk.size();

  std::vector<unsigned> ngon_connections;
  std::vector<unsigned> ngon_tmp;

  // we could precount but this is alot of looping, would rather do this
  ngon_connections.reserve(numFace * 3 *
                           3); // let's act like every face is a pentagon , that
                               // makes it 5-2 triangles

  for (int fi = 0; fi < numFace; fi++) {

    auto face_vertices = faceverts_stack(faceBlk, fvertBlk, edgeBlk, fi,
                                         GNM_NAMESPACE::szarg<5>());
    int count = face_vertices.size();

    if (count == 4) {

      int two_triangles[6];

      quad_split_solver(face_vertices[0], face_vertices[1], face_vertices[2],
                        face_vertices[3], pointsRef, two_triangles);

      ngon_connections.push_back(two_triangles[0]);
      ngon_connections.push_back(two_triangles[1]);
      ngon_connections.push_back(two_triangles[2]);
      ngon_connections.push_back(two_triangles[3]);
      ngon_connections.push_back(two_triangles[4]);
      ngon_connections.push_back(two_triangles[5]);

      face_map.push_back(fi);
      face_map.push_back(fi);
      face_map.push_back(fi);
      face_map.push_back(fi);
      face_map.push_back(fi);
      face_map.push_back(fi);

      int va = face_vertices[two_triangles[1]];
      int vb = face_vertices[two_triangles[2]];

      if (!dry_run && !edge_exist(va, vb, vertBlk, edgeBlk, fvertBlk)) {
        // fmt::print("va = {} , vb = {}\n", va, vb);
        (void)vert_vert_connect(va, vb, fi, faceBlk, fvertBlk, edgeBlk,
                                vertBlk);
      }

    } else if (count > 4) {

      ngon_tmp.clear();
      if (gather_ear_clip_triangles(FaceHdl{fi}, faceBlk, edgeBlk, fvertBlk,
                                    vertBlk, pointsRef, ngon_tmp)) {

        #if 1
        std::copy(ngon_tmp.begin(), ngon_tmp.end(),
                  std::back_inserter(ngon_connections));

        for (size_t g = 0; g < ngon_tmp.size(); g++)
          face_map.push_back(fi);
        #endif

        if (!dry_run) {
          for (size_t i = 0; i < ngon_tmp.size(); i += 3) {
            int va = face_vertices[ngon_tmp[i + 0]];
            int vb = face_vertices[ngon_tmp[i + 2]];
            // fmt::print("va = {} , vb = {}\n", va, vb);

            if (!edge_exist(va, vb, vertBlk, edgeBlk, fvertBlk)) {


              (void)vert_vert_connect(va, vb, FaceHdl(), faceBlk, fvertBlk,
                                      edgeBlk, vertBlk);
            }
          }
        }

      } else {
        // fallback to fan triangulation

        fan_triangulation(faceBlk, fvertBlk, edgeBlk, vertBlk, fi,
                          ngon_connections, dry_run, face_map);
      }
    } else {

      ngon_connections.push_back(0);
      ngon_connections.push_back(1);
      ngon_connections.push_back(2);

      face_map.push_back(fi);
      face_map.push_back(fi);
      face_map.push_back(fi);
    }
  }

#ifndef NDEBUG

  if (!dry_run) {

    for (int fi = 0; fi < faceBlk.size(); fi++) {
      auto face_vertices = faceverts_stack(faceBlk, fvertBlk, edgeBlk, fi,
                                           GNM_NAMESPACE::szarg<3>());
      assert(face_vertices.size() == 3);
    }
  }

#endif

  return ngon_connections;
}



/// @brief filters Edge 's based on angle
/// @tparam VertBlk
/// @tparam EdgeBlk
/// @tparam FaceBlk
/// @tparam FvertBlk
/// @param _angle
/// @param faceBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk
/// @param face_normals
/// @return BitArray
template <class VertBlk, class EdgeBlk, class FaceBlk, class FvertBlk>
BitArray dihedral_angle_filter(/*BitArray &tags,*/ float _angle,
                               FaceBlk const &faceBlk, FvertBlk const &fvertBlk,
                               EdgeBlk const &edgeBlk, VertBlk const &vertBlk,
                               const Vector_t *face_normals) {
  auto tags = BitArray::zeros(edgeBlk.size());
  auto cosang = std::cos(_angle);
  for (int i = 0; i < edgeBlk.size(); i++) {
    auto z = get_edge_fvert(EdgeHdl{i}, edgeBlk);
    auto f0 = fvertBlk[z.id].f;
    auto f1 = fvertBlk[fvertBlk[z.id].radial.id].f;
    // assert(f0.id < faceBlk.size() && f0.id >= 0);
    // assert(f1.id < faceBlk.size() && f1.id >= 0);
    auto &f0_n = face_normals[f0.id];
    auto &f1_n = face_normals[f1.id];
    auto dota = f0_n.dot(f1_n);
    auto normprod = f0_n.norm() * f1_n.norm();
    normprod += 1e-6;
    auto dihedr = dota / normprod;
    tags.setBit(i, dihedr > cosang);
  }
  return tags;
}
//
//
//// result is given in terms of normal per fvert
// template <class VertBlk, class EdgeBlk, class FaceBlk, class FvertBlk>
// std::vector<Vector_t>
// smooth_by_edge_tags2(BitArray const &tags, FaceBlk const &faceBlk,
//                    FvertBlk const &fvertBlk, EdgeBlk const &edgeBlk,
//                    VertBlk const &vertBlk, const Vector_t *in_points,
//                    const Vector_t *face_normals) {
//
//  std::vector<Vector_t> ret;
//
//  for (size_t i = 0; i < fvertBlk.size(); i++) {
//    ret.push_back(face_normals[fvertBlk[i].f.id]);
//  }
//
//  for (int i = 0; i < edgeBlk.size(); i++) {
//    EdgeHdl ei{i};
//    if (tags.isBitOn (ei.id)) {
//      Vector_t cum;
//      cum.setZero();
//      auto fv = get_edge_fvert(ei, faceBlk, fvertBlk, edgeBlk, vertBlk);
//      auto f0 = fvertBlk[fv.id].f;
//      auto f1 = fvertBlk[fvertBlk[fv.id].radial.id].f;
//      cum = face_normals[f0.id] + face_normals[f1.id];
//    }
//  }
//
//
//  for (auto &n : ret)
//    n.normalize();
//  return ret;
//}
template <class FvertBlk, unsigned N>
void cumul_fvnormals(const FvertBlk &fvertBlk,
                     const llvm_vecsmall::SmallVector<FvertHdl, N> &_stack,
                     Vector_t &cum, const Vector_t *face_normals) {
  cum.setZero();
  for (auto cur : _stack)
    cum += face_normals[fvertBlk[cur.id].f.id];
}

template <class VertBlk, class EdgeBlk, class FaceBlk, class FvertBlk>
void smooth_by_edge_tags_(BitArray const &tags, FaceBlk const &faceBlk,
                          FvertBlk const &fvertBlk, EdgeBlk const &edgeBlk,
                          VertBlk const &vertBlk, const Vector_t *face_normals,
                          char *v_visited, Vector_t *ret, EdgeHdl ei,
                          VertHdl vj) {

  if (!v_visited[vj.id]) {
    auto e0 = vert_edge_ring_unordered(edgeBlk, vj, ei);

    llvm_vecsmall::SmallVector<FvertHdl, 16> manifolder;

    for (auto ej : e0) {
      if (tags.isBitOn(ej.id)) {
        FvertHdlPair efv =
            edge_manifold_fv(edgeBlk, vertBlk, fvertBlk, faceBlk, ej, vj);
        manifolder.push_back(efv.first);
        manifolder.push_back(efv.second);
      }
    }
    if (fv_killdouble(manifolder)) {
      Vector_t cum;
      cumul_fvnormals(fvertBlk, manifolder, cum, face_normals);
      for (auto cur : manifolder)
        ret[cur.id] = cum;

      v_visited[vj.id] = 1;
    } else {

      FvertHdlPair efv =
          edge_manifold_fv(edgeBlk, vertBlk, fvertBlk, faceBlk, ei, vj);
      auto f0 = fvertBlk[efv.first.id].f;
      auto f1 = fvertBlk[efv.second.id].f;

      Vector_t cum = face_normals[f0.id] + face_normals[f1.id];

      ret[efv.first.id] = cum;
      ret[efv.second.id] = cum;
    }
  }
}

/// @brief smooth a mesh normals based on Edge 's, just like in AdskMaya
/// @tparam VertBlk
/// @tparam EdgeBlk
/// @tparam FaceBlk
/// @tparam FvertBlk
/// @param tags
/// @param faceBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk
/// @param face_normals
/// @return

template <class VertBlk, class EdgeBlk, class FaceBlk, class FvertBlk>
std::vector<Vector_t>
smooth_by_edge_tags(BitArray const &tags, FaceBlk const &faceBlk,
                    FvertBlk const &fvertBlk, EdgeBlk const &edgeBlk,
                    VertBlk const &vertBlk, const Vector_t *face_normals) {

  std::vector<char> v_visited(vertBlk.size(), 0);

  std::vector<Vector_t> ret;

  ret.reserve(fvertBlk.size());
  for (int i = 0; i < fvertBlk.size(); i++) {
    ret.push_back(face_normals[fvertBlk[i].f.id]);
  }

  for (int i = 0; i < edgeBlk.size(); i++) {
    EdgeHdl ei{i};
    if (tags.isBitOn(ei.id)) {
      VertHdl *ev = (VertHdl *)&edgeBlk[ei.id];

      smooth_by_edge_tags_(tags, faceBlk, fvertBlk, edgeBlk, vertBlk,
                           face_normals, v_visited.data(), ret.data(), ei,
                           ev[0]);

      smooth_by_edge_tags_(tags, faceBlk, fvertBlk, edgeBlk, vertBlk,
                           face_normals, v_visited.data(), ret.data(), ei,
                           ev[1]);

      //

      //
    }
  }

  for (auto &cur : ret) {
    cur.normalize();
  }
  return ret;
}

//// result is given in terms of normal per fvert
// template <class VertBlk, class EdgeBlk, class FaceBlk, class FvertBlk>
// std::vector<Vector_t>
// smooth_by_edge_tags(BitArray const &tags, FaceBlk const &faceBlk,
//                    FvertBlk const &fvertBlk, EdgeBlk const &edgeBlk,
//                    VertBlk const &vertBlk, const Vector_t *in_points,
//                    const Vector_t *face_normals) {
//
//  std::vector<Vector_t> ret;
//
//  for (size_t i = 0; i < fvertBlk.size(); i++) {
//    ret.push_back(face_normals[fvertBlk[i].f.id]);
//  }
//
//  for (int i = 0; i < vertBlk.size(); i++) {
//    VertHdl vi{i};
//    auto fv = vert_outgoing_fverts(vertBlk, fvertBlk, edgeBlk, vi);
//
//    Vector_t cum;
//    cum.setZero();
//    for (auto j : fv) {
//      EdgeHdl ej = fvertBlk[j.id].e;
//      if (tags.isBitOn(ej.id)) {
//        auto jk = fvertBlk[fvertBlk[j.id].radial.id].next;
//        cum += face_normals[fvertBlk[jk.id].f.id];
//        cum += face_normals[fvertBlk[j.id].f.id];
//      }
//    }
//
//    for (auto j : fv) {
//      EdgeHdl ej = fvertBlk[j.id].e;
//      if (tags.isBitOn(ej.id)) {
//        auto jk = fvertBlk[fvertBlk[j.id].radial.id].next;
//        ret[jk.id] = cum;
//        ret[j.id] = cum;
//      }
//    }
//  }
//
//  for (auto &n : ret)
//    n.normalize();
//  return ret;
//}

//////////////////////////////////////////// EDGE
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/// @brief get all edge lengths
/// @tparam EdgeBlk
/// @param edgeBlk
/// @param in_points
/// @param out_lengths
template <class EdgeBlk>
void get_all_edge_lengths(EdgeBlk &edgeBlk, const Vector_t *in_points,
                          std::vector<float> &out_lengths) {

  for (int i = 0; i < edgeBlk.size(); ++i) {
    out_lengths.push_back(
        (in_points[edgeBlk[i].v0.id] - in_points[edgeBlk[i].v1.id]).norm());
  }
}
/// @brief get all edge center positions
/// @tparam EdgeBlk
/// @param edgeBlk
/// @param in_points
/// @param out_lengths
template <class EdgeBlk>
void get_all_edge_mids(EdgeBlk &edgeBlk, const Vector_t *in_points,
                       std::vector<float> &out_lengths) {

  for (int i = 0; i < edgeBlk.size(); ++i) {
    out_lengths.push_back(
        (in_points[edgeBlk[i].v0.id] + in_points[edgeBlk[i].v1.id]) * 0.5);
  }
}

template <class EdgeBlk>
Vector_t get_edge_mid(const EdgeBlk &edgeBlk, EdgeHdl ehdl,
                      const Vector_t *in_points) {

  return (in_points[edgeBlk[ehdl.id].v0.id] +
          in_points[edgeBlk[ehdl.id].v1.id]) *
         0.5;
}

//////////////////////////////////////////// VERTEX
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct TrisPositions {

  Vector_t v0, v1, v2;
};

template <class EdgeBlk, class FvertBlk, class FaceBlk>
TrisPositions tris_points(const FaceBlk &faceBlk, const FvertBlk &fvertBlk,
                          const EdgeBlk &edgeBlk, FaceHdl fh,
                          const Vector_t *in_points) {

  FvertHdl fv0 = faceBlk[fh.id].fv;
  FvertHdl fv1 = fvertBlk[fv0.id].next;
  FvertHdl fv2 = fvertBlk[fv1.id].next;

  VertHdl v0 = get_v(edgeBlk, fvertBlk, fv0);
  VertHdl v1 = get_v(edgeBlk, fvertBlk, fv1);
  VertHdl v2 = get_v(edgeBlk, fvertBlk, fv2);

  return {{in_points[v0.id]}, {in_points[v1.id]}, {in_points[v2.id]}};
}

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
auto static_initial_tri_qem(const VertBlk &vertBlk, const FaceBlk &faceBlk,
                            const EdgeBlk &edgeBlk, const FvertBlk &fvertBlk,
                            const Vector_t *inpoints) {

  std::vector<Eigen::MatrixXd> ret;
  ret.resize(vertBlk.size());
  for (auto &mat : ret) {
    mat.resize(4, 4);
    mat.setZero();
  }

  for (int i = 0; i < faceBlk.size(); ++i) {
    FaceHdl curf{i};
    TrisPositions pt = tris_points(faceBlk, fvertBlk, edgeBlk, curf, inpoints);

    Vector_t e1 = (pt.v1 - pt.v0).normalized();
    Vector_t e2 = pt.v2 - pt.v0;
    e2 = e2 - (e1.dot(e2)) * e1;
    e2.normalize();

    const double epsi = 1e-7;

    Eigen::MatrixXd A(3, 3);
    A.setIdentity();
    A = A - e1.transpose() * e1 - e2.transpose() * e2;
    Eigen::VectorXd b = pt.v0.dot(e1) * e1 + pt.v0.dot(e2) * e2 - pt.v0;
    double c = pt.v0.dot(pt.v0) - pt.v0.dot(e1) * pt.v0.dot(e1) -
               pt.v0.dot(e2) * pt.v0.dot(e2);

    Eigen::MatrixXd metric(4, 4);
    metric.block(0, 0, 3, 3) = A;
    metric.block(0, 3, 3, 1) = b;
    metric.block(3, 0, 1, 3) = b.transpose();
    metric(3, 3) = c;

    auto curf_vs = faceverts_stack_int(faceBlk, fvertBlk, edgeBlk, curf);

    for (int j = 0; j < 3; j++) {
      int vi = curf_vs[j];
      ret[vi] += metric;
    }
  }

  return ret;
}

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk,
          size_t vcount = 8>
Vector_t vertexSmoothNormal(
    VertHdl vhdl, FaceBlk const &faceBlk, FvertBlk const &fvertBlk,
    EdgeBlk const &edgeBlk, VertBlk const &vertBlk, const Vector_t *in_points,
    const Vector_t *in_face_normals,
    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> _vert_valence =
        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{}

) {

  Vector_t total = Vector_t ::Zero();
  if (in_face_normals) {

    auto vert_faces =
        vert_face_ring(vertBlk, fvertBlk, edgeBlk, vhdl, gnm::szarg<vcount>());
    for (FaceHdl face : vert_faces) {
      total += in_face_normals[face.id];
    }

  } else {

    auto vert_faces =
        vert_face_ring(vertBlk, fvertBlk, edgeBlk, vhdl, gnm::szarg<vcount>());

    for (FaceHdl face : vert_faces) {
      total +=
          get_face_normal(face, faceBlk, fvertBlk, edgeBlk, vertBlk, in_points);
    }
  }

  return total.normalized();
}

/// @brief computes smooth Vert 's normals
/// @tparam VertBlk
/// @tparam FaceBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param faceBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk
/// @param in_points
/// @param in_face_normals
/// @param _face_vert_count
/// @return

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk,
          size_t vcount = 8>
std::vector<Vector_t> vertexSmoothNormal_all(
    FaceBlk const &faceBlk, FvertBlk const &fvertBlk, EdgeBlk const &edgeBlk,
    VertBlk const &vertBlk, const Vector_t *in_points,
    const Vector_t *in_face_normals,
    GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> _face_vert_count =
        GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{}) {
  std::vector<Vector_t> out;
  out.resize(vertBlk.size());

  using Scalar_t = Vector_t::Scalar;

  // set to zero
  std::fill((Scalar_t *)out.data(), (Scalar_t *)(out.data() + vertBlk.size()),
            (Scalar_t)0.0);

  if (in_face_normals) {

    for (int i = 0; i < faceBlk.size(); ++i) {
      FaceHdl face_handle{i};
      auto face_verts = faceverts_stack(faceBlk, fvertBlk, edgeBlk, face_handle,
                                        GNM_NAMESPACE::szarg<vcount>());
      for (int j = 0; j < face_verts.size(); ++j) {
        auto &normal = out[face_verts[j].id];
        normal += in_face_normals[face_handle.id];
      }
    }
    // finally normalize
    for (auto &vertNormal : out) {
      vertNormal.normalize();
    }

  } else {

    for (int i = 0; i < faceBlk.size(); ++i) {
      FaceHdl face_handle{i};
      auto face_verts = faceverts_stack(faceBlk, fvertBlk, edgeBlk, face_handle,
                                        GNM_NAMESPACE::szarg<vcount>());

      int j = face_verts.size() - 2;
      int k = face_verts.size() - 1;
      // Vector_t normal;
      for (int x = 0; x < face_verts.size(); j = k, k = x++) {

        auto &normal = out[face_verts[k].id];
        normal += (in_points[face_verts[k].id] - in_points[face_verts[j].id])
                      .cross(in_points[face_verts[x].id] -
                             in_points[face_verts[k].id]);
      }
    }
    // finally normalize
    for (auto &vertNormal : out) {
      vertNormal.normalize();
    }
  }

  return out;
}

//////////////////////////////////////////// FACE
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
Vector_t triangle_normal(FaceHdl face_handle, const FaceBlk &faceBlk,
                         FvertBlk const &fvertBlk, EdgeBlk const &edgeBlk,
                         const VertBlk &vertBlk, const Vector_t *in_points) {

  FvertHdl fv0 = faceBlk[face_handle.id].fv;
  FvertHdl fv1 = fvertBlk[fv0.id].next;
  FvertHdl fv2 = fvertBlk[fv1.id].next;

  VertHdl v0 = get_v(edgeBlk, fvertBlk, fv0);
  VertHdl v1 = get_v(edgeBlk, fvertBlk, fv1);
  VertHdl v2 = get_v(edgeBlk, fvertBlk, fv2);

  // v0--->v1 vs v1--->v2
  return (in_points[v1.id] - in_points[v0.id])
      .cross(in_points[v2.id] - in_points[v1.id])
      .normalized();
}

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
Vector_t get_face_normal(FaceHdl face_handle, const FaceBlk &faceBlk,
                         FvertBlk const &fvertBlk, EdgeBlk const &edgeBlk,
                         const VertBlk &vertBlk, const Vector_t *in_points) {

  auto face_verts = faceverts_stack(faceBlk, fvertBlk, edgeBlk, face_handle);

  Vector_t normal = Vector_t::Zero();

  int j = face_verts.size() - 2;
  int k = face_verts.size() - 1;

  for (int i = 0; i < face_verts.size(); j = k, k = i++) {

    auto a_to_b = (in_points[face_verts[k].id] - in_points[face_verts[j].id]);
    auto b_to_c = (in_points[face_verts[i].id] - in_points[face_verts[k].id]);
    normal += (a_to_b.cross(b_to_c)).normalized();
  }
  normal.normalize();

  return normal;
}

template <class FaceBlk, class FvertBlk, class EdgeBlk>
VertHdl face_first_vert(FaceHdl fhdl, const FaceBlk &faceBlk,
                        FvertBlk const &fvertBlk, EdgeBlk const &edgeBlk) {

  FvertHdl fv0 = faceBlk[fhdl.id].fv;
  return get_v(edgeBlk, fvertBlk, fv0);
}

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
Vector4_t face_plane_equation(FaceHdl fhdl, const FaceBlk &faceBlk,
                              FvertBlk const &fvertBlk, EdgeBlk const &edgeBlk,
                              const VertBlk &vertBlk,
                              const Vector_t *in_points) {

  Vector_t f_n =
      triangle_normal(fhdl, faceBlk, fvertBlk, edgeBlk, vertBlk, in_points);

  double d =
      -f_n.dot(in_points[face_first_vert(fhdl, faceBlk, fvertBlk, edgeBlk).id]);

  return Vector4_t(f_n[0], f_n[1], f_n[2], d);
}

/// @brief check if Face is planar
/// @tparam VertBlk
/// @tparam FaceBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @tparam EpsilonT
/// @param face_handle
/// @param faceBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk
/// @param in_points
/// @param in_normal
/// @param epsilon
/// @return
template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk,
          typename EpsilonT = float>
int is_face_planar(FaceHdl face_handle, const FaceBlk &faceBlk,
                   FvertBlk const &fvertBlk, EdgeBlk const &edgeBlk,
                   const VertBlk &vertBlk, const Vector_t *in_points,
                   Vector_t const *in_normal = nullptr,
                   const EpsilonT epsilon = 1e-5f) {

  Vector_t normal;
  if (in_normal) {
    normal = *in_normal;
  } else {
    normal = get_face_normal(face_handle, faceBlk, fvertBlk, edgeBlk, vertBlk,
                             in_points);
  }
  auto face_verts = faceverts_stack(faceBlk, fvertBlk, edgeBlk, face_handle);

  int j = face_verts.size() - 2;
  int k = face_verts.size() - 1;

  for (int i = 0; i < face_verts.size(); j = k, k = i++) {

    auto a_to_b = (in_points[face_verts[k].id] - in_points[face_verts[j].id]);

    if (abs(normal.dot(a_to_b)) >= epsilon) {
      return 0;
    }
  }

  return 1;
}

/// @brief check if Face is convex
/// @tparam VertBlk
/// @tparam FaceBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param face_handle
/// @param faceBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk
/// @param in_points
/// @param in_normal
/// @return
template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
int is_face_convex(FaceHdl face_handle, const FaceBlk &faceBlk,
                   FvertBlk const &fvertBlk, EdgeBlk const &edgeBlk,
                   const VertBlk &vertBlk, const Vector_t *in_points,
                   Vector_t const *in_normal = nullptr) {

  Vector_t normal;
  if (in_normal) {
    normal = *in_normal;
  } else {
    normal = get_face_normal(face_handle, faceBlk, fvertBlk, edgeBlk, vertBlk,
                             in_points);
  }
  auto face_verts = faceverts_stack(faceBlk, fvertBlk, edgeBlk, face_handle);

  int j = face_verts.size() - 2;
  int k = face_verts.size() - 1;
  for (int i = 0; i < face_verts.size(); j = k, k = i++) {

    auto a_to_b = (in_points[face_verts[k].id] - in_points[face_verts[j].id]);
    auto b_to_c = (in_points[face_verts[i].id] - in_points[face_verts[k].id]);
    auto dotprod = normal.dot(a_to_b.cross(b_to_c));
    if (dotprod < 0) {
      return 0;
    }
  }
  return 1;
}

/// @brief get all Face 's normals
/// @tparam VertBlk
/// @tparam FaceBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param faceBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk
/// @param in_points
/// @return Vector_t *

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
std::vector<Vector_t>
get_all_face_normals(FaceBlk const &faceBlk, FvertBlk const &fvertBlk,
                     EdgeBlk const &edgeBlk, VertBlk const &vertBlk,
                     const Vector_t *in_points) {
  std::vector<Vector_t> out;
  out.reserve(faceBlk.size());
  for (int i = 0; i < faceBlk.size(); ++i) {
    FaceHdl face_handle(i);
    out.push_back(get_face_normal(face_handle, faceBlk, fvertBlk, edgeBlk,
                                  vertBlk, in_points));
  }

  return out;
}

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
Vector_t get_face_center(FaceHdl face_handle, FaceBlk &faceBlk,
                         FvertBlk &fvertBlk, EdgeBlk &edgeBlk, VertBlk &vertBlk,
                         const Vector_t *in_points) {

  Vector_t cumul = Vector_t::Zero();

  auto face_verts = faceverts_stack(faceBlk, fvertBlk, edgeBlk, face_handle);

  for (size_t i = 0; i < face_verts.size(); i++) {
    cumul += in_points[face_verts[i].id];
  }

  return cumul / (double)face_verts.size();
}

/// @brief get the area of a Face
/// @tparam VertBlk
/// @tparam FaceBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param face_handle
/// @param faceBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk
/// @param in_points
/// @param normal
/// @param use_normal
/// @param outNormal
/// @return float
template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
float get_face_area(FaceHdl face_handle, FaceBlk &faceBlk, FvertBlk &fvertBlk,
                    EdgeBlk &edgeBlk, VertBlk &vertBlk,
                    const Vector_t *in_points, Vector_t normal = Vector_t(),
                    bool use_normal = false, Vector_t *outNormal = nullptr) {

  auto face_verts = faceverts_stack(faceBlk, fvertBlk, edgeBlk, face_handle,
                                    GNM_NAMESPACE::szarg<4>());

  Vector_t cumul = Vector_t::Zero();
  normal = Vector_t::Zero();
  if (!use_normal) {

    int j = face_verts.size() - 2;
    int k = face_verts.size() - 1;

    for (int i = 0; i < face_verts.size(); j = k, k = i++) {

      auto a_to_b = (in_points[face_verts[k].id] - in_points[face_verts[j].id]);
      auto b_to_c = (in_points[face_verts[i].id] - in_points[face_verts[k].id]);
      normal += a_to_b.cross(b_to_c);
      cumul += in_points[face_verts[j].id].cross(in_points[face_verts[k].id]);
    }
    normal.normalize();

  } else {

    int j = face_verts.size() - 1;

    for (int i = 0; i < face_verts.size(); j = i++) {
      cumul += in_points[face_verts[j].id].cross(in_points[face_verts[i].id]);
    }
  }
  if (!use_normal && outNormal) {

    *outNormal = normal;
  }
  return abs(normal.dot(cumul) * 0.5F);
}

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
std::vector<float> get_face_area_at_indices(
    FaceBlk &faceBlk, FvertBlk &fvertBlk, EdgeBlk &edgeBlk, VertBlk &vertBlk,
    const Vector_t *in_points, std::vector<int> const &in_indices,
    const Vector_t *in_normals = nullptr, bool use_normals = false) {

  std::vector<float> outAreas;
  if (!use_normals) {

    for (int i = 0; i < in_indices.size(); ++i) {
      outAreas.push_back(get_face_area(FaceHdl(in_indices[i]), faceBlk,
                                       fvertBlk, edgeBlk, vertBlk, in_points)

      );
    }

  } else {
    for (int i = 0; i < in_indices.size(); ++i) {
      outAreas.push_back(get_face_area(FaceHdl(in_indices[i]), faceBlk,
                                       fvertBlk, edgeBlk, vertBlk, in_points,
                                       in_normals[in_indices[i]], true));
    }
  }

  return outAreas;
}

