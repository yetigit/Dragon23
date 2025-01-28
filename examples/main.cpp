
#include "obj_brep.h"
#include <GnMesh/Mesh/GeomQueries.hpp>
#include <random>
#include <set>

#include <fmt/color.h>

struct BufferedEdgeData {

  std::vector<float> points;
  std::vector<int> edges;
  std::vector<int> counts;
  std::vector<int> index;
};

BufferedEdgeData read_edgedata() {

  std::vector<float> points;
  std::vector<int> edges;
  std::vector<int> counts;
  std::vector<int> index;

  std::ifstream file("../data/lucy-bool1.txt");
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
    std::cout << "could not open file" << std::endl;
    std::exit(-1);
  }

  return BufferedEdgeData{points, edges, counts, index};
}

#define DN_FMT_SANITY_PRINT(x, y)                                              \
  if (x)                                                                       \
    fmt::print(fg(fmt::color::spring_green), "sanity: {}; msg: {}\n", x, y);   \
  else                                                                         \
    fmt::print(fg(fmt::color::crimson), "sanity: {}\nmsg: {}\n", x, y)

int main() {

  auto edgedata = read_edgedata();
  unsigned numface =edgedata.counts.size();
  unsigned numvert = edgedata.points.size() / 3;
  unsigned numedge = numface + numvert - 2;
  unsigned numfv = edgedata.index.size();
  MeshStoreConfig storeconf;
  storeconf.numVertBlock = 1u;
  storeconf.numFaceBlock = 1u;
  storeconf.numEdgeBlock = 2u;
  storeconf.numFvertBlock = 1u;
  storeconf.numVertPerBlock = std::max(3u, numvert);
  storeconf.numFacePerBlock = std::max(1u, numface);
  storeconf.numEdgePerBlock = std::max(3u, numedge);
  storeconf.numFvertPerBlock = std::max(3u, numfv);
  BlkAllocConfig01::init(storeconf);

  BM_BlkArrayGeo<BlkAllocConfig01> blkmesh;

  auto &faceBlk = blkmesh.mFaceBlock;
  auto &fvertBlk = blkmesh.mFvertBlock;
  auto &edgeBlk = blkmesh.mEdgeBlock;
  auto &vertBlk = blkmesh.mVertBlock;
  auto &point_ref = blkmesh.mPoints;

  for (size_t i = 0; i < edgedata.points.size(); i += 3) {
    point_ref.emplace_back(edgedata.points[i + 0], edgedata.points[i + 1],
                           edgedata.points[i + 2]);
  }

  structure_from_edges(edgedata.counts.data(), edgedata.index.data(),
                       edgedata.counts.size(), edgedata.index.size(),
                       edgedata.edges.data(), faceBlk, fvertBlk, edgeBlk,
                       vertBlk, point_ref.size());


  std::vector<unsigned> f;
  auto all_triangles = general_triangulation(faceBlk, fvertBlk, edgeBlk, vertBlk, point_ref, false, f);  


  #if 0
  for (size_t i = 0; i < all_triangles.size(); i+=3) {
    int v0 = all_triangles[i + 0]; 
    int v1 = all_triangles[i + 1]; 
    int v2 = all_triangles[i + 2]; 
    int ti = i / 3;  
    fmt::print("t {} = [{}, {}, {}]\n", ti, v0, v1, v2); 
  }
  #endif

  #if 0
  std::vector<unsigned> connections;
  gather_ear_clip_triangles(FaceHdl(0), faceBlk, edgeBlk, fvertBlk, vertBlk,
                           point_ref, connections);

  int num_connec = 0;
  for (size_t i = 0; i < connections.size(); i+=2) {
    int va = connections[i + 0];
    int vb = connections[i + 1];
    fmt::print("va = {} , vb = {}\n", va, vb);
    
    if (!edge_exist(va, vb, vertBlk, edgeBlk, fvertBlk)) {
		vert_vert_connect(va, vb, FaceHdl(), faceBlk, fvertBlk, edgeBlk, vertBlk);
      num_connec += 1;
    }
  }
  fmt::print("num connec {}\n", num_connec);
  #endif 

 
  auto flagsfaces = std::vector<unsigned>(faceBlk.size(), 0);
  auto flagsverts = std::vector<unsigned>(vertBlk.size(), 0);
  auto flagsedges = std::vector<unsigned>(edgeBlk.size(), 0);
  auto flagsfvert = std::vector<unsigned>(fvertBlk.size(), 0);


  #if 1
  blockMesh_to_obj("c:/users/baidh/desktop/cool.obj", blkmesh, flagsfaces,
                   flagsfvert, flagsedges, flagsverts);
  #endif

  BlkAllocConfig01::destroy();
}
