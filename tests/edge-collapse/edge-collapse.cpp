

#include <GnMesh/Mesh/BlockArrayKernel.hpp>
#include <GnMesh/Mesh/Queries.hpp>
#include <GnMesh/Mesh/Construct.hpp>
#include <iostream>
#include <fstream>

#include <vector>
#include <GnMesh/Mesh/Flags.hpp>

#include <gtest/gtest.h>
#include <set>

/*

this test aim at proofing edge collapse

*/
struct BufferedEdgeData {

  std::vector<float> points;
  std::vector<int> edges;
  std::vector<int> counts;
  std::vector<int> index;
};

std::vector<int> load_tri_sphere_index_data() {

  std::ifstream file("../../../data/tri_sphere_indices.txt");
  assert(file.is_open());

  std::string line;
  std::getline(file, line);
  std::istringstream iss(line);

  std::vector<int> ret;

  int cur;
  while (iss >> cur) {
    ret.push_back(cur);
  }

  return ret;
}

BufferedEdgeData read_edgedata(std::string filepath) {

  std::vector<float> points;
  std::vector<int> edges;
  std::vector<int> counts;
  std::vector<int> index;

  std::ifstream file(filepath);
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
    printf("couldnt open file\n");
    std::exit(-1);
  }

  return BufferedEdgeData{points, edges, counts, index};
}

template <class FaceBlk, class FvertBlk, class EdgeBlk>
static bool checkMesh(FaceBlk const &faceBlk, FvertBlk const &fvertBlk,
                      EdgeBlk const &edgeBlk,
                      std::vector<unsigned> const &faceflags,
                      std::vector<unsigned> const &vertflags) {

  for (size_t i = 0; i < faceBlk.size(); i++) {
    if (ItemFlags::is_off(faceflags[i], ItemFlags::DELETED)) {
      auto faceverts = faceverts_stack_int(faceBlk, fvertBlk, edgeBlk, i);
      for (auto vi : faceverts) {
        if (ItemFlags::is_on(vertflags[vi], ItemFlags::DELETED)) {
          return false;
        }
      }
    }
  }

  return true;
}
template <class FaceBlk, class FvertBlk, class EdgeBlk, class VertBlk>
static void checkCycles(FaceBlk const &faceBlk, FvertBlk const &fvertBlk,
                        EdgeBlk const &edgeBlk, VertBlk const &vertBlk,
                        std::vector<unsigned> const &faceflags,
                        std::vector<unsigned> const &vertflags,
                        std::vector<unsigned> const &edgeflags,
                        std::vector<unsigned> const &fvflags) {

  for (size_t i = 0; i < vertBlk.size(); i++) {
    if (ItemFlags::is_off(vertflags[i], ItemFlags::DELETED)) {
      FvertHdl vfv = vertBlk[i].fv;

      EXPECT_TRUE(vfv.isValid());
      EXPECT_TRUE(ItemFlags::is_off(fvflags[vfv], ItemFlags::DELETED));

      auto vring = vert_edge_ring(vertBlk, fvertBlk, edgeBlk, i);
      for (EdgeHdl ei : vring) {
        ASSERT_TRUE(ItemFlags::is_off(edgeflags[ei], ItemFlags::DELETED));
      }
    }
  }
}

TEST(EdgeCollapse, Cube) {

  BlkAllocConfig01::init(MeshStoreConfig{(1u << 11), 64u, (1u << 10), 64u,
                                         (1u << 11), 64u, (1u << 10), 64u});

  BM_BlkArrayGeo<BlkAllocConfig01> blkmesh;

  auto edgedata = read_edgedata("../../../data/cube_data.txt");

  auto &faceBlk = blkmesh.mFaceBlock;
  auto &fvertBlk = blkmesh.mFvertBlock;
  auto &edgeBlk = blkmesh.mEdgeBlock;
  auto &vertBlk = blkmesh.mVertBlock;
  auto &point_ref = blkmesh.mPoints;

  for (size_t i = 0; i < edgedata.points.size(); i += 3)
    point_ref.emplace_back(edgedata.points[i + 0], edgedata.points[i + 1],
                           edgedata.points[i + 2]);

  structure_from_edges(edgedata.counts.data(), edgedata.index.data(),
                       edgedata.counts.size(), edgedata.index.size(),
                       edgedata.edges.data(), faceBlk, fvertBlk, edgeBlk,
                       vertBlk, point_ref.size());

  // edge_info(edgeBlk, std::cout);

  auto flagsfaces = std::vector<unsigned>(faceBlk.size(), 0);
  auto flagsverts = std::vector<unsigned>(vertBlk.size(), 0);
  auto flagsedges = std::vector<unsigned>(edgeBlk.size(), 0);
  auto flagsfvert = std::vector<unsigned>(fvertBlk.size(), 0);

  // 2 , 3, 1 , 0
  std::vector<int> e_inds = {2, 3, 1, 0};
  // std::vector<int> e_inds = {2, 3};

  int ith = 0;
  int reject = 0;

  int numRepair = 0;
  for (auto eidx : e_inds) {
    ith += 1;

    auto ei = EdgeHdl{eidx};

    auto vpair = get_edge_vpair(ei, edgeBlk);

    // make it a rule for this example to kill the highest index
    if (vpair.first > vpair.second) {
      std::swap(vpair.second, vpair.first);
    }

    bool ok = false;
    ok = edge_collapse_ok(faceBlk, fvertBlk, edgeBlk, vertBlk, ei, flagsfaces,
                          flagsfvert, flagsedges, flagsverts, vpair);
    // ok = true;

    if (ok) {

      bool succ = false;
      VertHdl kept;
      succ =
          edge_collapse(faceBlk, fvertBlk, edgeBlk, vertBlk, ei, flagsfaces,
                        flagsfvert, flagsedges, flagsverts, vpair, point_ref, kept);

      bool repairSucc = edge_collapse_repair(
          faceBlk, fvertBlk, edgeBlk, vertBlk, flagsfaces, flagsfvert,
          flagsedges, flagsverts, vpair.first, numRepair);

      EXPECT_TRUE(succ);
      EXPECT_TRUE(repairSucc);

#if 1
      int numfaces = 0;
      int numedges = 0;
      int numverts = 0;

      for (size_t i = 0; i < flagsverts.size(); i++) {
        auto &flags = flagsverts[i];
        if (ItemFlags::is_off(flags, ItemFlags::DELETED))
          numverts += 1;
      }

      for (size_t i = 0; i < flagsedges.size(); i++) {
        auto &flags = flagsedges[i];
        if (ItemFlags::is_off(flags, ItemFlags::DELETED))
          numedges += 1;
      }

      for (size_t i = 0; i < flagsfaces.size(); i++) {
        auto &flags = flagsfaces[i];
        if (ItemFlags::is_off(flags, ItemFlags::DELETED))
          numfaces += 1;
      }

      if (ith == 1) {
        EXPECT_EQ(numfaces, faceBlk.size());
        EXPECT_EQ(numedges, edgeBlk.size() - 1);
        EXPECT_EQ(numverts, vertBlk.size() - 1);
      } else if (ith == 2) {
        EXPECT_EQ(numfaces, faceBlk.size() - 1);
        EXPECT_EQ(numedges, edgeBlk.size() - 3);
        EXPECT_EQ(numverts, vertBlk.size() - 2);
      } else if (ith == 3) {
        EXPECT_EQ(numfaces, 4);
        EXPECT_EQ(numedges, 7);
        EXPECT_EQ(numverts, 5);
      } else if (ith == 4) {
        EXPECT_EQ(numfaces, 2);
        EXPECT_EQ(numedges, 4);
        EXPECT_EQ(numverts, 4);
      }

      EXPECT_TRUE(
          checkMesh(faceBlk, fvertBlk, edgeBlk, flagsfaces, flagsverts));

      checkCycles(faceBlk, fvertBlk, edgeBlk, vertBlk, flagsfaces, flagsverts,
                  flagsedges, flagsfvert);
      // if (ith == 2)
      //   break;

#endif

    } else {
      reject += 1;
    }
  }

  EXPECT_EQ(reject, 0);
  EXPECT_EQ(numRepair, 0);

  BlkAllocConfig01::destroy();
}

TEST(EdgeCollapse, NonManifold_Parkour_B) {

  BlkAllocConfig01::init(MeshStoreConfig{(1u << 11), 64u, (1u << 10), 64u,
                                         (1u << 11), 64u, (1u << 10), 64u});

  BM_BlkArrayGeo<BlkAllocConfig01> blkmesh;

  auto edgedata = read_edgedata("../../../data/parkour_nonmani_b.txt");

  auto &faceBlk = blkmesh.mFaceBlock;
  auto &fvertBlk = blkmesh.mFvertBlock;
  auto &edgeBlk = blkmesh.mEdgeBlock;
  auto &vertBlk = blkmesh.mVertBlock;
  auto &point_ref = blkmesh.mPoints;

  for (size_t i = 0; i < edgedata.points.size(); i += 3)
    point_ref.emplace_back(edgedata.points[i + 0], edgedata.points[i + 1],
                           edgedata.points[i + 2]);

  structure_from_edges(edgedata.counts.data(), edgedata.index.data(),
                       edgedata.counts.size(), edgedata.index.size(),
                       edgedata.edges.data(), faceBlk, fvertBlk, edgeBlk,
                       vertBlk, point_ref.size());

  auto flagsfaces = std::vector<unsigned>(faceBlk.size(), 0);
  auto flagsverts = std::vector<unsigned>(vertBlk.size(), 0);
  auto flagsedges = std::vector<unsigned>(edgeBlk.size(), 0);
  auto flagsfvert = std::vector<unsigned>(fvertBlk.size(), 0);

  std::vector<int> e_inds = {34, 5, 42, 16, 25, 28};

  int ith = 0;
  int reject = 0;
  int numRepair = 0;

  for (auto eidx : e_inds) {
    ith += 1;

    auto ei = EdgeHdl{eidx};

    auto vpair = get_edge_vpair(ei, edgeBlk);

    if (vpair.first > vpair.second) {
      std::swap(vpair.second, vpair.first);
    }

    bool ok = false;
    ok = edge_collapse_ok(faceBlk, fvertBlk, edgeBlk, vertBlk, ei, flagsfaces,
                          flagsfvert, flagsedges, flagsverts, vpair);

    if (ok) {

      bool succ = false;
      VertHdl kept;
      succ =
          edge_collapse(faceBlk, fvertBlk, edgeBlk, vertBlk, ei, flagsfaces,
                        flagsfvert, flagsedges, flagsverts, vpair, point_ref, kept);

      bool repairSucc = edge_collapse_repair(
          faceBlk, fvertBlk, edgeBlk, vertBlk, flagsfaces, flagsfvert,
          flagsedges, flagsverts, vpair.first, numRepair);

      EXPECT_TRUE(succ);
      EXPECT_TRUE(repairSucc);

#if 1
      int numfaces = 0;
      int numedges = 0;
      int numverts = 0;

      for (size_t i = 0; i < flagsverts.size(); i++) {
        auto &flags = flagsverts[i];
        if (ItemFlags::is_off(flags, ItemFlags::DELETED))
          numverts += 1;
      }

      for (size_t i = 0; i < flagsedges.size(); i++) {
        auto &flags = flagsedges[i];
        if (ItemFlags::is_off(flags, ItemFlags::DELETED))
          numedges += 1;
      }

      for (size_t i = 0; i < flagsfaces.size(); i++) {
        auto &flags = flagsfaces[i];
        if (ItemFlags::is_off(flags, ItemFlags::DELETED))
          numfaces += 1;
      }

      if (ith == 1) {
        EXPECT_EQ(numfaces, 18);
        EXPECT_EQ(numedges, 42);
        EXPECT_EQ(numverts, 25);
      } else if (ith == 2) {
        EXPECT_EQ(numfaces, 16);
        EXPECT_EQ(numedges, 39);
        EXPECT_EQ(numverts, 24);
      } else if (ith == 3) {
        EXPECT_EQ(numfaces, 14);
        EXPECT_EQ(numedges, 35);
        EXPECT_EQ(numverts, 22);
      } else if (ith == 4) {
        EXPECT_EQ(numfaces, 14);
        EXPECT_EQ(numedges, 34);
        EXPECT_EQ(numverts, 21);
      } else if (ith == 5) {
        EXPECT_EQ(numfaces, 14);
        EXPECT_EQ(numedges, 33);
        EXPECT_EQ(numverts, 20);
      } else if (ith == 6) {
        EXPECT_EQ(numfaces, 13);
        EXPECT_EQ(numedges, 31);
        EXPECT_EQ(numverts, 19);
      }

      EXPECT_TRUE(
          checkMesh(faceBlk, fvertBlk, edgeBlk, flagsfaces, flagsverts));

      checkCycles(faceBlk, fvertBlk, edgeBlk, vertBlk, flagsfaces, flagsverts,
                  flagsedges, flagsfvert);
      // if (ith == 2)
      //   break;

#endif

    } else {
      reject += 1;
    }
  }

  EXPECT_EQ(reject, 0);
  EXPECT_EQ(numRepair, 0);

  BlkAllocConfig01::destroy();
}

TEST(EdgeCollapse, TriSphereRandom) {

  BlkAllocConfig01::init(MeshStoreConfig{(1u << 11), 64u, (1u << 10), 64u,
                                         (1u << 11), 64u, (1u << 10), 64u});

  BM_BlkArrayGeo<BlkAllocConfig01> blkmesh;

  auto edgedata = read_edgedata("../../../data/tri_sphere.txt");

  auto &faceBlk = blkmesh.mFaceBlock;
  auto &fvertBlk = blkmesh.mFvertBlock;
  auto &edgeBlk = blkmesh.mEdgeBlock;
  auto &vertBlk = blkmesh.mVertBlock;
  auto &point_ref = blkmesh.mPoints;

  for (size_t i = 0; i < edgedata.points.size(); i += 3)
    point_ref.emplace_back(edgedata.points[i + 0], edgedata.points[i + 1],
                           edgedata.points[i + 2]);

  structure_from_edges(edgedata.counts.data(), edgedata.index.data(),
                       edgedata.counts.size(), edgedata.index.size(),
                       edgedata.edges.data(), faceBlk, fvertBlk, edgeBlk,
                       vertBlk, point_ref.size());

  auto flagsfaces = std::vector<unsigned>(faceBlk.size(), 0);
  auto flagsverts = std::vector<unsigned>(vertBlk.size(), 0);
  auto flagsedges = std::vector<unsigned>(edgeBlk.size(), 0);
  auto flagsfvert = std::vector<unsigned>(fvertBlk.size(), 0);

  std::vector<int> e_inds = load_tri_sphere_index_data();

  int ith = 0;
  int reject = 0;
  int numRepair = 0;

  for (auto eidx : e_inds) {
    ith += 1;

    auto ei = EdgeHdl{eidx};

    auto vpair = get_edge_vpair(ei, edgeBlk);

    if (vpair.first > vpair.second) {
      std::swap(vpair.second, vpair.first);
    }

    bool ok = false;
    ok = edge_collapse_ok(faceBlk, fvertBlk, edgeBlk, vertBlk, ei, flagsfaces,
                          flagsfvert, flagsedges, flagsverts, vpair);

    if (ok) {

      bool succ = false;
      VertHdl kept;
      succ =
          edge_collapse(faceBlk, fvertBlk, edgeBlk, vertBlk, ei, flagsfaces,
                        flagsfvert, flagsedges, flagsverts, vpair, point_ref, kept);

      bool repairSucc = edge_collapse_repair(
          faceBlk, fvertBlk, edgeBlk, vertBlk, flagsfaces, flagsfvert,
          flagsedges, flagsverts, vpair.first, numRepair);

      EXPECT_TRUE(succ);
      EXPECT_TRUE(repairSucc);

#if 1
      int numfaces = 0;
      int numedges = 0;
      int numverts = 0;

      for (size_t i = 0; i < flagsverts.size(); i++) {
        auto &flags = flagsverts[i];
        if (ItemFlags::is_off(flags, ItemFlags::DELETED))
          numverts += 1;
      }

      for (size_t i = 0; i < flagsedges.size(); i++) {
        auto &flags = flagsedges[i];
        if (ItemFlags::is_off(flags, ItemFlags::DELETED))
          numedges += 1;
      }

      for (size_t i = 0; i < flagsfaces.size(); i++) {
        auto &flags = flagsfaces[i];
        if (ItemFlags::is_off(flags, ItemFlags::DELETED))
          numfaces += 1;
      }

      EXPECT_TRUE(
          checkMesh(faceBlk, fvertBlk, edgeBlk, flagsfaces, flagsverts));

      checkCycles(faceBlk, fvertBlk, edgeBlk, vertBlk, flagsfaces, flagsverts,
                  flagsedges, flagsfvert);

#endif

    } else {
      reject += 1;
    }
  }

  float rejecperc = reject / (float)e_inds.size();
  rejecperc *= 100.f;
  EXPECT_NEAR(34.15f, rejecperc, 1e-2f);

  BlkAllocConfig01::destroy();
}


TEST(EdgeCollapse, RemoveVkeep) {

  BlkAllocConfig01::init(MeshStoreConfig{(1u << 11), 64u, (1u << 10), 64u,
                                         (1u << 11), 64u, (1u << 10), 64u});

  BM_BlkArrayGeo<BlkAllocConfig01> blkmesh;

  auto edgedata = read_edgedata("../../../data/vkeep_removed.txt");

  auto &faceBlk = blkmesh.mFaceBlock;
  auto &fvertBlk = blkmesh.mFvertBlock;
  auto &edgeBlk = blkmesh.mEdgeBlock;
  auto &vertBlk = blkmesh.mVertBlock;
  auto &point_ref = blkmesh.mPoints;

  for (size_t i = 0; i < edgedata.points.size(); i += 3)
    point_ref.emplace_back(edgedata.points[i + 0], edgedata.points[i + 1],
                           edgedata.points[i + 2]);

  structure_from_edges(edgedata.counts.data(), edgedata.index.data(),
                       edgedata.counts.size(), edgedata.index.size(),
                       edgedata.edges.data(), faceBlk, fvertBlk, edgeBlk,
                       vertBlk, point_ref.size());

  auto flagsfaces = std::vector<unsigned>(faceBlk.size(), 0);
  auto flagsverts = std::vector<unsigned>(vertBlk.size(), 0);
  auto flagsedges = std::vector<unsigned>(edgeBlk.size(), 0);
  auto flagsfvert = std::vector<unsigned>(fvertBlk.size(), 0);

  std::vector<int> e_inds = {221};

  int ith = 0;
  int reject = 0;
  int numRepair = 0;

  for (auto eidx : e_inds) {
    ith += 1;

    auto ei = EdgeHdl{eidx};

    auto vpair = get_edge_vpair(ei, edgeBlk);

    if (vpair.first > vpair.second) {
      std::swap(vpair.second, vpair.first);
    }

    bool ok = false;
    ok = edge_collapse_ok(faceBlk, fvertBlk, edgeBlk, vertBlk, ei, flagsfaces,
                          flagsfvert, flagsedges, flagsverts, vpair);

    if (ok) {

      bool succ = false;
      VertHdl kept;
      succ = edge_collapse(faceBlk, fvertBlk, edgeBlk, vertBlk, ei, flagsfaces,
                           flagsfvert, flagsedges, flagsverts, vpair, point_ref,
                           kept);

      EXPECT_FALSE(kept.isValid());

      if (kept.isValid()) {
        bool repairSucc = edge_collapse_repair(
            faceBlk, fvertBlk, edgeBlk, vertBlk, flagsfaces, flagsfvert,
            flagsedges, flagsverts, vpair.first, numRepair);
      }

      EXPECT_TRUE(succ);

#if 1
      int numfaces = 0;
      int numedges = 0;
      int numverts = 0;

      for (size_t i = 0; i < flagsverts.size(); i++) {
        auto &flags = flagsverts[i];
        if (ItemFlags::is_off(flags, ItemFlags::DELETED))
          numverts += 1;
      }

      for (size_t i = 0; i < flagsedges.size(); i++) {
        auto &flags = flagsedges[i];
        if (ItemFlags::is_off(flags, ItemFlags::DELETED))
          numedges += 1;
      }

      for (size_t i = 0; i < flagsfaces.size(); i++) {
        auto &flags = flagsfaces[i];
        if (ItemFlags::is_off(flags, ItemFlags::DELETED))
          numfaces += 1;
      }

      EXPECT_TRUE(
          checkMesh(faceBlk, fvertBlk, edgeBlk, flagsfaces, flagsverts));

      checkCycles(faceBlk, fvertBlk, edgeBlk, vertBlk, flagsfaces, flagsverts,
                  flagsedges, flagsfvert);

      EXPECT_EQ(numfaces, 100);
      EXPECT_EQ(numedges, 219);
      EXPECT_EQ(numverts, 120);

#endif

    } else {
      reject += 1;
    }
  }

  EXPECT_EQ(reject, 0);

  BlkAllocConfig01::destroy();
}



int main(int argc, char **argv) {

  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
