

#include <GnMesh/Mesh/BlockArrayKernel.hpp>
#include <GnMesh/Mesh/Construct.hpp>
#include <gtest/gtest.h>
#include <unordered_set>
#include <random>

/*

proofing block array erase works

*/
struct EdgeHasher {
  std::size_t operator()(const Edge &e) const {

    auto a = (int *)&e;

    std::size_t h = 0;

    for (size_t i = 0; i < 5; i++) {

      int id = a[i];
      h ^= std::hash<int>{}(id) + 0x9e3779b9 + (h << 6) + (h >> 2);
    }
    return h;
  }
};

TEST(BlockArrayErasure, GoodValues) {

  const int N = 7;
  const int M = 3;

  BlkAllocConfig01::init(MeshStoreConfig{N, M, N, M, N, M, N, M});

  BM_BlkArrayGeo<BlkAllocConfig01> blkmesh;

  auto &edgeBlk = blkmesh.mEdgeBlock;
  std::vector<int> erase_ids;

  std::unordered_set<Edge, EdgeHasher> edgeset;
  std::unordered_set<Edge, EdgeHasher> noDeleto;

  std::mt19937 prng(std::random_device{}());
  std::uniform_int_distribution<int> dist(0, (N * M) - 1);

  for (int i = 0; i < N * M; i++) {

    int g = dist(prng);
    erase_ids.push_back(g);
    edgeBlk.pushBack(Edge{i, g, i, 0, i});
  }

  {
    std::unordered_set<int> killdupl(erase_ids.begin(), erase_ids.end());
    erase_ids = std::vector<int>(killdupl.begin(), killdupl.end());
#if 0
    printf("random erase ids\n");
    for (auto cur : erase_ids) {
      printf("%d ", cur);
    }
    puts("");
#endif
  }

  for (auto cur : erase_ids)
    edgeset.insert(edgeBlk[cur]);

  for (size_t i = 0; i < edgeBlk.size(); i++) {
    auto &cur = edgeBlk[i];
    if (edgeset.count(cur) == 0)
      noDeleto.insert(cur);
  }

  std::unordered_set<Edge, EdgeHasher> maptesto;
  std::set<int> toeraseSet(erase_ids.begin(), erase_ids.end());
  auto idxmap = edgeBlk.erase(erase_ids.data(), erase_ids.size());

  for (size_t i = 0; i < idxmap.size(); i++) {
    if (toeraseSet.count(i) == 0) {
      int goodid = idxmap[i];
      auto &cur = edgeBlk[goodid];
      EXPECT_EQ(noDeleto.count(cur), 1);
    }
  }

  for (size_t i = 0; i < edgeBlk.size(); i++) {
    auto &cur = edgeBlk[i];
    EXPECT_EQ(edgeset.count(cur), 0);
  }

  for (size_t i = 0; i < edgeBlk.size(); i++) {
    auto &cur = edgeBlk[i];
    EXPECT_EQ(noDeleto.count(cur), 1);
  }

  BlkAllocConfig01::destroy();
}

TEST(BlockArrayErasure, GoodSizeAndBlocks) {

  const int N = 4;
  const int M = 2;

  BlkAllocConfig01::init(MeshStoreConfig{N, M, N, M, N, M, N, M});

  BM_BlkArrayGeo<BlkAllocConfig01> blkmesh;

  auto &edgeBlk = blkmesh.mEdgeBlock;

  for (int i = 0; i < N; i++) {

    edgeBlk.pushBack(Edge{i, 3, i, 0, i});
  }

  Edge lastItemCopy = edgeBlk.back();

  int toErase[]{0};
  int toErase_sz = sizeof(toErase) / sizeof(int);
  edgeBlk.erase(toErase, toErase_sz);

  EXPECT_EQ(edgeBlk.size(), N - 1);
  EXPECT_EQ(edgeBlk.numBlocks(), 1);

  EXPECT_EQ(lastItemCopy, edgeBlk[N - 2]);

  BlkAllocConfig01::destroy();
}

int main(int argc, char **argv) {

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
