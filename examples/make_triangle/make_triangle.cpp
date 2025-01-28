

#include "../obj_brep.h"
#include <GnMesh/Mesh/GeomQueries.hpp>

int main() {

  BlkAllocConfig01::init(MeshStoreConfig{(1u << 11), 64u, (1u << 10), 64u,
                                         (1u << 11), 64u, (1u << 10), 64u});

  BM_BlkArrayGeo<BlkAllocConfig01> blkmesh;

  auto &faceBlk = blkmesh.mFaceBlock;
  auto &fvertBlk = blkmesh.mFvertBlock;
  auto &edgeBlk = blkmesh.mEdgeBlock;
  auto &vertBlk = blkmesh.mVertBlock;
  auto &point_ref = blkmesh.mPoints;

  std::vector<int> indices = {0, 1, 2};
  std::vector<int> counts = {3};

  point_ref.emplace_back(0.0, 0.0, 0.0);
  point_ref.emplace_back(1.0, 0.0, 0.0);
  point_ref.emplace_back(0.0, 0.0, -1.0);

  structure_from_indices(counts.data(), indices.data(), counts.size(),
                         indices.size(), faceBlk, fvertBlk, edgeBlk, vertBlk,
                         point_ref.size());
}