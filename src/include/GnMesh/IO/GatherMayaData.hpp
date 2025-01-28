
#pragma once

#include <GnMeshCommon/RAII.hpp>
#include <GnMesh/Geom/GnmEigen.hpp>
#include <vector>

#include <maya/MDagPath.h>

#include <maya/MFnMesh.h>

#include <maya/MIntArray.h>
#include <maya/MFloatArray.h>
#include <maya/MString.h>
#include <maya/MMatrix.h>
#include <maya/MColor.h>
#include <maya/MColorArray.h>

namespace maya {

struct Convert2GnmOptions {
  enum E {

    normals = 1,
    uvs = 2,
    colors = 4

  };
  int bits = 0;
  std::string uv_set_name;
  std::string color_set_name;
};

struct Mesh_Indices {
  std::unique_ptr<MIntArray> counts;
  std::unique_ptr<MIntArray> indices;
};

// #TODO complete with ALL properties

struct MayaUvData {

  std::vector<int> counts, indices;
  std::vector<Eigen::Vector2_t> coords;
};

struct MayaColorData {

  std::vector<int> indices;
  std::vector<Eigen::Vector_t> colors;
};

struct MayaMeshData_basic {
  Mesh_Indices indices;
  Eigen::Mat44_t matrix;
  // float const *points;
  std::vector<Eigen::Vector_t> points;
  OpaqueRAII normals;
  OpaqueRAII uvs;
  OpaqueRAII colors;
  int numVerts, numEdges;
};

void gather_existing_mesh_topology(MDagPath mesh_path,
                                   Mesh_Indices &out_indexData) {

  MFnMesh mesh_fn(mesh_path);

  auto counts_uptr = std::make_unique<MIntArray>();
  auto indices_uptr = std::make_unique<MIntArray>();

  (void)mesh_fn.getVertices(*counts_uptr, *indices_uptr);

  out_indexData.counts = std::move(counts_uptr);
  out_indexData.indices = std::move(indices_uptr);

  //
}

void gather_existing_mesh_points(MDagPath mesh_path,
                                 std::vector<Eigen::Vector_t> &out_pointBuffer,
                                 Eigen::Mat44_t &out_matrix,
                                 int points_to_world) {

  MStatus status;

  MMatrix matrix = mesh_path.inclusiveMatrix(&status);
  CHECK_MSTATUS(status);

  MFnMesh mesh_fn(mesh_path);
  auto *pointBuffer = mesh_fn.getRawPoints(&status);
  // out_pointBuffer = mesh_fn.getRawPoints(&status);

  CHECK_MSTATUS(status);

  float matrixBuf[4][4];
  double matrixBuf_d[16];
  matrix.get(matrixBuf);

  for (size_t i = 0; i < 16; i++) {
    matrixBuf_d[i] = (double)(((float *)matrixBuf)[i]);
  }
  out_matrix = Eigen::Map<Eigen::Mat44_t>((double *)matrixBuf_d, 4, 4);

  // transpose for maya right hand rotation
  out_matrix.transposeInPlace();

  assert(out_pointBuffer.empty());
  if (points_to_world) {
    for (size_t i = 0; i < mesh_fn.numVertices() * 3; i += 3) {
      auto transformed = out_matrix.block<3, 3>(0, 0) *
                             Eigen::Vector3d((double)pointBuffer[i],
                                             (double)pointBuffer[i + 1],
                                             (double)pointBuffer[i + 2]) +
                         out_matrix.block<3, 1>(0, 3);

      out_pointBuffer.push_back(transformed);
    }
  } else {
    for (size_t i = 0; i < mesh_fn.numVertices() * 3; i += 3) {
      out_pointBuffer.emplace_back((double)pointBuffer[i],
                                   (double)pointBuffer[i + 1],
                                   (double)pointBuffer[i + 2]);
    }
  }

  //
}

void gather_existing_mesh_normals(MDagPath mesh_path,
                                  Eigen::Mat44_t const &matrix,
                                  std::vector<Eigen::Vector_t> &out_normals,
                                  int world_normals) {
  MFnMesh mesh_fn(mesh_path);

  auto normalMatrix = matrix.block<3, 3>(0, 0).inverse().transpose();

  MStatus status;

  const float *normalBuf = mesh_fn.getRawNormals(&status);

  CHECK_MSTATUS(status);
  assert(out_normals.empty());

  if (world_normals) {
    for (size_t i = 0; i < mesh_fn.numNormals() * 3; i += 3) {

      auto transformed =
          (normalMatrix * Eigen::Vector3d((double)normalBuf[i],
                                          (double)normalBuf[i + 1],
                                          (double)normalBuf[i + 2]))
              .normalized();

      out_normals.push_back(transformed);
    }
  } else {
    for (size_t i = 0; i < mesh_fn.numNormals() * 3; i += 3) {

      out_normals.push_back(Eigen::Vector_t((double)normalBuf[i],
                                            (double)normalBuf[i + 1],
                                            (double)normalBuf[i + 2]));
    }
  }

  /*
    VectorNf n1, n2;
    MatrixNf normalMatrix = t.linear().inverse().transpose();
    n2 = (normalMatrix * n1).normalized();
  */
}

void gather_existing_mesh_uvs(MDagPath mesh_path,
                              std::string const &uv_set_name,
                              MayaUvData &out_uvs) {

  auto &out_counts = out_uvs.counts;
  auto &out_indices = out_uvs.indices;
  auto &out_coords = out_uvs.coords;

  MStatus status;

  MFnMesh mesh_fn(mesh_path);

  MIntArray counts, indices;

  MString uvset_name(uv_set_name.c_str());

  MString *uvset_name_arg = uvset_name == "" ? (MString *)nullptr : &uvset_name;
  status = mesh_fn.getAssignedUVs(counts, indices, uvset_name_arg);

  CHECK_MSTATUS(status);

  MFloatArray u, v;

  mesh_fn.getUVs(u, v, uvset_name_arg);

  out_counts.resize(counts.length());
  out_indices.resize(indices.length());
  memcpy((int *)&counts[0], out_counts.data(), sizeof(int) * counts.length());
  memcpy((int *)&indices[0], out_indices.data(),
         sizeof(int) * indices.length());

  assert(out_coords.empty());

  // can memcpy mfloatarray ?
  for (size_t i = 0; i < u.length(); i++) {
    out_coords.emplace_back((double)u[i], (double)v[i]);
  }
}

// we will use the default unset color as a marker for whic fv has or does not
// have a color forget about indices , maya is too stupid to give it in bulk
void gather_existing_mesh_colors(MDagPath mesh_path,
                                 std::string const &color_set_name,
                                 MayaColorData &out_clr) {

  auto &out_indices = out_clr.indices;
  auto &out_rgb = out_clr.colors;

  MColor unsetColor(0.0f, 0.0f, 0.0f,
                    0.0f); // the marker will be the alpha being 0

  MString colorset_name(color_set_name.c_str());

  MString *colorset_name_arg =
      colorset_name == "" ? (MString *)nullptr : &colorset_name;

  MStatus status;

  MFnMesh mesh_fn(mesh_path);

  if (mesh_fn.numColors() == 0) {
    return;
  }

  MColorArray colors;
  status = mesh_fn.getFaceVertexColors(colors, colorset_name_arg, &unsetColor);

  CHECK_MSTATUS(status);

  out_rgb.reserve(colors.length());
  out_indices.reserve(colors.length());

  auto almost_zero = [](const float a) { return abs(a) < 1e-5f; };
  for (size_t i = 0; i < colors.length(); i++) {
    if (almost_zero(colors[i].a)) {
      out_indices.push_back(-1);
    } else {
      out_indices.push_back(i);
      out_rgb.emplace_back((double)colors[i].r, (double)colors[i].g,
                           (double)colors[i].b);
    }
  }
}

void gather_existing_mesh_basic(MDagPath mesh_path,
                                MayaMeshData_basic &out_basicMeshData,
                                int points_to_world,
                                Convert2GnmOptions options) {

  maya::gather_existing_mesh_topology(mesh_path, out_basicMeshData.indices);
  maya::gather_existing_mesh_points(mesh_path, out_basicMeshData.points,
                                    out_basicMeshData.matrix, points_to_world);

  if (options.bits & Convert2GnmOptions::normals) {
    std::vector<Eigen::Vector_t> *ref;
    out_basicMeshData.normals =
        OpaqueRAII::construct<std::vector<Eigen::Vector_t>>(ref);
    gather_existing_mesh_normals(mesh_path, out_basicMeshData.matrix, *ref,
                                 points_to_world);
  }

  if (options.bits & Convert2GnmOptions::uvs) {

    MayaUvData *ref;
    out_basicMeshData.uvs = OpaqueRAII::construct<MayaUvData>(ref);
    gather_existing_mesh_uvs(mesh_path, options.uv_set_name, *ref);
  }

  if (options.bits & Convert2GnmOptions::colors) {

    MayaColorData *ref;
    out_basicMeshData.colors = OpaqueRAII::construct<MayaColorData>(ref);
    gather_existing_mesh_colors(mesh_path, options.color_set_name, *ref);
  }

  MFnMesh mesh_fn(mesh_path);
  out_basicMeshData.numVerts = mesh_fn.numVertices();
  out_basicMeshData.numEdges = mesh_fn.numEdges();
}

} // namespace maya