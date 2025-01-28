

#include "tiny_obj_loader.h"

#include <GnMesh/Mesh/BlockArrayKernel.hpp>
#include <GnMesh/Mesh/Queries.hpp>
#include <GnMesh/Mesh/Construct.hpp>
#include <map>
#include <cmath>

#include <iostream>

#include <vector>

template <class BM_t> void obj_to_BM(BM_t &blkmesh, std::string filepath) {

  auto &faceBlk = blkmesh.mFaceBlock;
  auto &fvertBlk = blkmesh.mFvertBlock;
  auto &edgeBlk = blkmesh.mEdgeBlock;
  auto &vertBlk = blkmesh.mVertBlock;
  auto &point_ref = blkmesh.mPoints;

  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;

  std::string warn;
  std::string err;
  bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                              filepath.c_str(), (const char *)0, false);

  std::vector<int> indices;
  std::vector<int> counts;

  auto &mesh = shapes[0].mesh;
  auto &points = attrib.vertices;

  counts.reserve(mesh.num_face_vertices.size());
  indices.reserve(mesh.indices.size());

  for (auto i : mesh.num_face_vertices) {
    counts.push_back(i);
  }
  for (auto &i : mesh.indices) {
    indices.push_back(i.vertex_index);
  }

  point_ref.reserve(points.size() / 3);
  for (int i = 0; i < points.size(); i += 3) {
    point_ref.emplace_back(points[i + 0], points[i + 1], points[i + 2]);
  }

  // create from data
  structure_from_indices(counts.data(), indices.data(), counts.size(),
                         indices.size(), faceBlk, fvertBlk, edgeBlk, vertBlk,
                         point_ref.size());
}

template <class BM_t>
void blockMesh_to_obj(std::string const &filename, const BM_t &blkmesh,
                      BitArray &deadfaces, BitArray &deadverts,
                      const std::vector<Eigen::Vector_t> *normals_ = nullptr) {

  auto &faceBlk = blkmesh.mFaceBlock;
  auto &fvertBlk = blkmesh.mFvertBlock;
  auto &edgeBlk = blkmesh.mEdgeBlock;
  auto &vertBlk = blkmesh.mVertBlock;
  auto &point_ref = blkmesh.mPoints;

  std::ofstream ofs;
  ofs.open(filename);

  if (!ofs.is_open()) {
    assert(0 && "could not write file");
  }

  ofs << "o simple_mesh" << std::endl;

  int lastvtx = 0;

  robin_hood::unordered_map<int, int> v_idmap;
  v_idmap.reserve(point_ref.size());
  for (int i = 0; i < point_ref.size(); i++) {

    if (deadverts.isBitOn(i)) {
      v_idmap[i] = lastvtx++;
    }
  }

  for (int i = 0; i < point_ref.size(); i++) {

    if (deadverts.isBitOn(i)) {
      ofs << "v " << point_ref[i] << "\n";
    }
  }

  if (normals_) {
    auto &nref = *normals_;
    for (int i = 0; i < nref.size(); i++) {
      ofs << "vn " << nref[i] << "\n";
    }
  }

  for (int i = 0; i < faceBlk.size(); i++) {
    if (deadfaces.isBitOn(i)) {

      auto fv_stack =
          faceverts_stack_int(faceBlk, fvertBlk, edgeBlk, FaceHdl{i});

      auto ffv_stack = facefv_stack_int(faceBlk, fvertBlk, FaceHdl{i});

      int ii = 0;
      ofs << "f ";
      for (auto vi : fv_stack) {
        assert(v_idmap.count(vi));
        if (normals_) {
          //ofs << vi + 1 << "//" << vi + 1 << " ";
          ofs << vi + 1 << "//" << (ffv_stack[ii] + 1) << " ";
        } else {
          ofs << (v_idmap.at(vi)) + 1 << " ";
        }
        ++ii;
      }
      ofs << "\n";
    }
  }

  ofs.close();
}