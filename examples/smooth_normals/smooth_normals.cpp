

#include "../obj_brep.h"
#include <GnMesh/Mesh/GeomQueries.hpp>

int main() {

  BlkAllocConfig01::init(MeshStoreConfig{(1u << 11), 64u, (1u << 10), 64u,
                                         (1u << 11), 64u, (1u << 10), 64u});

  BM_BlkArrayGeo<BlkAllocConfig01> blkmesh;

  obj_to_BM(blkmesh, "c:/users/baidhir/desktop/hard_normals_cylinder.obj");

  auto &faceBlk = blkmesh.mFaceBlock;
  auto &fvertBlk = blkmesh.mFvertBlock;
  auto &edgeBlk = blkmesh.mEdgeBlock;
  auto &vertBlk = blkmesh.mVertBlock;
  auto &point_ref = blkmesh.mPoints;

  auto deadverts = BitArray::ones(vertBlk.size());
  auto deadfaces = BitArray::ones(faceBlk.size());

  auto face_normals = get_all_face_normals(faceBlk, fvertBlk, edgeBlk, vertBlk,
                                           point_ref.data());

  double degrees = 89.0;
  double radians = degrees * (M_PI / 180.0);
  BitArray edge_tags_ = dihedral_angle_filter(
      radians, faceBlk, fvertBlk, edgeBlk, vertBlk, face_normals.data());

  fmt::print("{}\n", edge_tags_.num_ones());
  auto vnormals = smooth_by_edge_tags(edge_tags_, faceBlk, fvertBlk, edgeBlk,
                                      vertBlk, face_normals.data());

}