
#include <GnMesh/Mesh/BlockArrayKernel.hpp>
#include <GnMesh/Mesh/Queries.hpp>
#include <GnMesh/Mesh/Construct.hpp>
#include <cmath>

#include <iostream>

int main() {


  BlkAllocConfig01::init(MeshStoreConfig{(1u << 11), 64u, (1u << 10), 64u,
                                         (1u << 11), 64u, (1u << 10), 64u});

  printf("block array size %zu\n", sizeof(BlkAllocConfig01::VertBlockArray_t));
  BM_BlkArrayGeo<BlkAllocConfig01> blkmesh;

  printf("\n-----------------\n");

  // structure_from_indices()

  int counts[]{4, 4};
  int indices[]{0, 1, 2, 3, 3, 2, 4, 5};

  auto &faceBlk = blkmesh.mFaceBlock;
  auto &fvertBlk = blkmesh.mFvertBlock;
  auto &edgeBlk = blkmesh.mEdgeBlock;
  auto &vertBlk = blkmesh.mVertBlock;

  structure_from_indices(counts, indices, 2, 8, faceBlk, fvertBlk, edgeBlk,
                         vertBlk, 6, 7);

  edge_insert_vert(EdgeHdl{3}, faceBlk, fvertBlk, edgeBlk, vertBlk);

  std::cout << "inserting done\n";

  std::cout << "\n";
  std::cout << "faces v\n";

  for (int k = 0; k < faceBlk.size(); ++k) {
    std::cout << k << "- ";
    std::cout << faceBlk[k].fv.id << "\n";
  }

  std::cout << "\n";

  std::cout << "\nverts v\n";

  for (int k = 0; k < vertBlk.size(); ++k) {
    std::cout << k << "- ";
    std::cout << vertBlk[k].fv.id << "\n";
  }

  std::cout << "\n";
  std::cout << "\nedges v\n";
  for (int k = 0; k < edgeBlk.size(); ++k) {
    std::cout << k << "- ";
    std::cout << edgeBlk[k].v0.id << ", ";
    std::cout << edgeBlk[k].v1.id << ", ";
    std::cout << edgeBlk[k].next0.id << ", ";
    std::cout << edgeBlk[k].next1.id << '\n';
  }

  std::cout << "\n";
  std::cout << "face-vertex v\n";
  for (int k = 0; k < fvertBlk.size(); ++k) {
    std::cout << k << "- ";
    std::cout << fvertBlk[k].e.id << ", ";
    std::cout << get_v(edgeBlk, fvertBlk, FvertHdl(k)).id << ", ";
    std::cout << fvertBlk[k].f.id << ", ";
    std::cout << fvertBlk[k].next.id << ", ";
    std::cout << fvertBlk[k].radial.id << '\n';
  }
  std::cout << "\n";

  //
}