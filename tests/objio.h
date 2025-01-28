#pragma once
#include <GnMesh/Mesh/IndexMesh.hpp>
#include <iostream>

#include <iomanip>
#include "tiny_obj_loader.h"
#include <iterator>

struct TinyResult {
  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
};

IndexMesh<float> loadobj(std::string const &filepath) {

  tinyobj::attrib_t attrib;
  std::vector<tinyobj::shape_t> shapes;
  std::vector<tinyobj::material_t> materials;

  IndexMesh<float> result;

  std::string warn;
  std::string err;
  bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err,
                              filepath.c_str(), (const char *)0, false);

  auto &mesh = shapes[0].mesh;
  auto &points = attrib.vertices;

  for (auto i : mesh.num_face_vertices) {
    result.counts.push_back(i);
  }

  for (auto &i : mesh.indices) {
    result.indices.push_back(i.vertex_index);
  }

  result.points.resize(points.size() / 3);

  memcpy(result.points.data(), points.data(), points.size() * sizeof(float));

  return result;
}

void exportobj(const IndexMesh<float> &in_mesh, std::string const &filename) {

  std::ofstream ofs;
  ofs.open(filename);

  if (!ofs.is_open()) {
    assert(0 && "could nor write file");
  }

  ofs << "o simple_mesh" << std::endl;
  for (auto &v : in_mesh.points) {
    ofs << "v " << v << '\n';
  }
  int pvi = 0;
  for (auto &fcount : in_mesh.counts) {
    ofs << "f ";
    for (size_t i = 0; i < fcount; i++) {
      int v = in_mesh.indices[pvi + i];
      ofs << (v+1) << ' ';
    }
    pvi += fcount;
    ofs << "\n";
  }

  ofs.close();
}