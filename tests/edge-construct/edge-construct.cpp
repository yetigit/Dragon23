

#include "../obj_brep.h"
#include <gtest/gtest.h>
#include <set>

/*

this test aim at proofing 1 : 1 correspondence with edge indices and verts
indices between data derived from a dcc and dragon

*/

struct BufferedEdgeData {

  std::vector<float> points;
  std::vector<int> edges;
  std::vector<int> counts;
  std::vector<int> index;
};

BufferedEdgeData read_edgedata(
    std::string filepath = R"(C:\repos\Dragon\data\plane_nonmani_data.txt)") {

  std::vector<float> points;
  std::vector<int> edges;
  std::vector<int> counts;
  std::vector<int> index;

  std::string path_to_file = filepath;
  std::ifstream file(path_to_file);
  if (file.is_open()) {

    for (size_t i = 0; i < 4; i++) {

      std::string line;
      std::getline(file, line);
      std::istringstream iss(line);

      if (i == 0) {

        float cur;
        while (iss >> cur) {
          points.push_back(cur);
        }
      } else if (i == 1) {

        int cur;
        while (iss >> cur) {
          edges.push_back(cur);
        }

      } else if (i == 2) {

        int cur;
        while (iss >> cur) {
          counts.push_back(cur);
        }

      } else if (i == 3) {

        int cur;
        while (iss >> cur) {
          index.push_back(cur);
        }
      }
    }

    file.close();
  } else {
    EXPECT_TRUE(false) << "cant open file";
  }

  return BufferedEdgeData{points, edges, counts, index};
}

TEST(EdgeConstruct, LaminaFacet) {
  BlkAllocConfig01::init(MeshStoreConfig{(1u << 11), 64u, (1u << 10), 64u,
                                         (1u << 11), 64u, (1u << 10), 64u});

  BM_BlkArrayGeo<BlkAllocConfig01> blkmesh;

  std::string loadp = R"(C:\repos\Dragon\data\lamina_face.txt)";

  auto edgedata = read_edgedata(loadp);

  int numPoints = edgedata.points.size() / 3;

  auto &faceBlk = blkmesh.mFaceBlock;
  auto &fvertBlk = blkmesh.mFvertBlock;
  auto &edgeBlk = blkmesh.mEdgeBlock;
  auto &vertBlk = blkmesh.mVertBlock;
  auto &point_ref = blkmesh.mPoints;

  structure_from_edges(edgedata.counts.data(), edgedata.index.data(),
                       edgedata.counts.size(), edgedata.index.size(),
                       edgedata.edges.data(), faceBlk, fvertBlk, edgeBlk,
                       vertBlk, numPoints);

  int numEdges = 0;
  int numFaces = 0;
  int numVerts = 0;
  int numFv = 0;

  numEdges = edgeBlk.size();
  numFaces = faceBlk.size();
  numVerts = vertBlk.size();
  numFv = fvertBlk.size();

  EXPECT_EQ(numVerts, 3);
  EXPECT_EQ(numEdges, 3);
  EXPECT_EQ(numFaces, 2);
  EXPECT_EQ(numFv, 6);

  auto f0Verts = faceverts_stack_int(faceBlk, fvertBlk, edgeBlk, 0);
  auto f1Verts = faceverts_stack_int(faceBlk, fvertBlk, edgeBlk, 1);
  EXPECT_EQ(f0Verts.size(), 3);
  EXPECT_EQ(f1Verts.size(), f0Verts.size());
  f0Verts.insert(f0Verts.end(), f1Verts.begin(), f1Verts.end());

  std::set<int> indexSet(f0Verts.begin(), f0Verts.end());

  EXPECT_EQ(indexSet.size(), 3);
  // .////////////

  auto f0Edges = face_edges_stack(faceBlk, fvertBlk, 0);
  auto f1Edges = face_edges_stack(faceBlk, fvertBlk, 1);
  EXPECT_EQ(f0Edges.size(), 3);
  EXPECT_EQ(f1Edges.size(), f0Edges.size());
  f0Edges.insert(f0Edges.end(), f1Edges.begin(), f1Edges.end());

  auto indexSet2 = std::set<EdgeHdl>(f0Edges.begin(), f0Edges.end());

  EXPECT_EQ(indexSet.size(), 3);
}

TEST(EdgeConstruct, GoodIndices) {
  BlkAllocConfig01::init(MeshStoreConfig{(1u << 11), 64u, (1u << 10), 64u,
                                         (1u << 11), 64u, (1u << 10), 64u});

  BM_BlkArrayGeo<BlkAllocConfig01> blkmesh;

  auto edgedata = read_edgedata();

  int numPoints = edgedata.points.size() / 3;

  auto &faceBlk = blkmesh.mFaceBlock;
  auto &fvertBlk = blkmesh.mFvertBlock;
  auto &edgeBlk = blkmesh.mEdgeBlock;
  auto &vertBlk = blkmesh.mVertBlock;
  auto &point_ref = blkmesh.mPoints;

  structure_from_edges(edgedata.counts.data(), edgedata.index.data(),
                       edgedata.counts.size(), edgedata.index.size(),
                       edgedata.edges.data(), faceBlk, fvertBlk, edgeBlk,
                       vertBlk, numPoints);

  int fvi = 0;
  for (int i = 0; i < faceBlk.size(); ++i) {

    FvertHdl first = faceBlk[i].fv;
    FvertHdl cur = first;
    int curcount = edgedata.counts[i];

    int x = 0;
    do {

      EdgeHdl ei = fvertBlk[cur].e;

      int v0 = edgeBlk[ei].v0;
      int v1 = edgeBlk[ei].v1;

      int fv_v = get_v(edgeBlk, fvertBlk, cur);
      int fv_vnext = get_v(edgeBlk, fvertBlk, fvertBlk[cur].next);

      // check that edge index is matching per face vertex
      EXPECT_EQ((int)ei, edgedata.edges[fvi + x]);

      int x1 = x == (curcount - 1) ? 0 : x + 1;

      if (v1 < v0) {
        std::swap(v0, v1);
      }
      int comp_v0 = edgedata.index[fvi + x];
      int comp_v1 = edgedata.index[fvi + x1];

      // check that the index order is the same
      EXPECT_EQ(fv_v, comp_v0);
      EXPECT_EQ(fv_vnext, comp_v1);

      if (comp_v1 < comp_v0) {
        std::swap(comp_v0, comp_v1);
      }

      // check that the vertices of the edge independent of the order are the
      // same
      EXPECT_EQ(v0, comp_v0);
      EXPECT_EQ(v1, comp_v1);

      ++x;

    } while ((cur = fvertBlk[cur].next) != first);

    fvi += curcount;
  }

  BlkAllocConfig01::destroy();
}

int main(int argc, char **argv) {

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
