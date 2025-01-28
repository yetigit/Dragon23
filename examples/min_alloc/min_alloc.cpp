#include <GnMesh/Mesh/BlockArrayKernel.hpp>

int main() {

  std::vector<int> counts = {3};
  std::vector<int> inds = {0, 1, 2};

  unsigned int orig_numf = 1;
  unsigned int orig_numv = 3;
  unsigned int orig_nume = (3 / 2) + 3;
  unsigned int orig_numfv = 3;

  /*

  unsigned numVertPerBlock, numVertBlock;

  unsigned numEdgePerBlock, numEdgeBlock;

  unsigned numFacePerBlock, numFaceBlock;

  unsigned numFvertPerBlock, numFvertBlock;
};


  */

  BlkAllocConfig01::init(MeshStoreConfig{orig_numv + 1, 1, orig_nume + 1, 2,
                                         orig_numf + 1, 1, orig_numfv + 1, 1});

  BM_BlkArrayGeo<BlkAllocConfig01> bm;

  auto &vertBlk = bm.mVertBlock;
  auto &fvertBlk = bm.mFvertBlock;
  auto &faceBlk = bm.mFaceBlock;
  auto &edgeBlk = bm.mEdgeBlock;

  structure_from_indices(counts.data(), inds.data(), counts.size(), inds.size(),
                         faceBlk, fvertBlk, edgeBlk, vertBlk, 3);
}
