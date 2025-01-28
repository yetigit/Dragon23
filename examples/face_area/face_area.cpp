

#include <GnMesh/Mesh/BlockArrayKernel.hpp>
#include <GnMesh/Mesh/Queries.hpp>
#include <GnMesh/Mesh/GeomQueries.hpp>
#include <GnMesh/Mesh/Construct.hpp>
#include <cmath>

#include <iostream>

#include <vector>

using Vector_t = Eigen::Vector_t;
using Vector2_t = Eigen::Vector_t;

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
float example_get_face_area(FaceHdl face_handle, FaceBlk &faceBlk,
                            FvertBlk &fvertBlk, EdgeBlk &edgeBlk,
                            VertBlk &vertBlk, const Vector_t *in_points,
                            Vector_t normal = Vector_t(),
                            bool use_normal = false,
                            Vector_t *outNormal = nullptr) {

  auto face_verts = faceverts_stack(faceBlk, fvertBlk, edgeBlk, face_handle,
                                    GNM_NAMESPACE::szarg<4>());

  Vector_t cumul = Vector_t::Zero();
  normal = Vector_t::Zero();
  if (!use_normal) {

    int j = face_verts.size() - 2;
    int k = face_verts.size() - 1;

    for (int i = 0; i < face_verts.size(); j = k, k = i++) {

      auto a_to_b = (in_points[face_verts[k].id] - in_points[face_verts[j].id]);
      auto b_to_c = (in_points[face_verts[i].id] - in_points[face_verts[k].id]);
      normal += a_to_b.cross(b_to_c);
      cumul += in_points[face_verts[j].id].cross(in_points[face_verts[k].id]);
    }
    normal.normalize();

  } else {

    int j = face_verts.size() - 1;

    for (int i = 0; i < face_verts.size(); j = i++) {
      cumul += in_points[face_verts[j].id].cross(in_points[face_verts[i].id]);
    }
  }
  if (!use_normal && outNormal) {

    *outNormal = normal;
  }
  return abs(normal.dot(cumul) * 0.5);
}
int main() {

  BlkAllocConfig01::init(MeshStoreConfig{(1u << 11), 64u, (1u << 10), 64u,
                                         (1u << 11), 64u, (1u << 10), 64u});

  printf("block array size %zu\n", sizeof(BlkAllocConfig01::VertBlockArray_t));
  BM_BlkArrayGeo<BlkAllocConfig01> blkmesh;


  printf("\n-----------------\n");

  constexpr unsigned num_verts = 26;
  constexpr unsigned num_edges = 26;
  // the mesh is a cube with 1lvl sub-d
  std::vector<int> counts{4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4,
                          4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4

  };
  std::vector<int> indices{
      0,  14, 20, 17, 14, 1,  15, 20, 20, 15, 3,  16, 17, 20, 16, 2,  2,
      16, 21, 8,  16, 3,  18, 21, 21, 18, 5,  19, 8,  21, 19, 4,  4,  19,
      22, 11, 19, 5,  9,  22, 22, 9,  7,  10, 11, 22, 10, 6,  6,  10, 23,
      13, 10, 7,  12, 23, 23, 12, 1,  14, 13, 23, 14, 0,  1,  12, 24, 15,
      12, 7,  9,  24, 24, 9,  5,  18, 15, 24, 18, 3,  6,  13, 25, 11, 13,
      0,  17, 25, 25, 17, 2,  8,  11, 25, 8,  4

  };

  float points_data[]{

      -0.277778, -0.277778, 0.277778,  0.277778,  -0.277778, 0.277778,
      -0.277778, 0.277778,  0.277778,  0.277778,  0.277778,  0.277778,
      -0.277778, 0.277778,  -0.277778, 0.277778,  0.277778,  -0.277778,
      -0.277778, -0.277778, -0.277778, 0.277778,  -0.277778, -0.277778,
      -0.375000, 0.375000,  0.000000,  0.375000,  0.000000,  -0.375000,
      0.000000,  -0.375000, -0.375000, -0.375000, 0.000000,  -0.375000,
      0.375000,  -0.375000, 0.000000,  -0.375000, -0.375000, 0.000000,
      0.000000,  -0.375000, 0.375000,  0.375000,  0.000000,  0.375000,
      0.000000,  0.375000,  0.375000,  -0.375000, 0.000000,  0.375000,
      0.375000,  0.375000,  0.000000,  0.000000,  0.375000,  -0.375000,
      0.000000,  0.000000,  0.500000,  0.000000,  0.500000,  0.000000,
      0.000000,  0.000000,  -0.500000, 0.000000,  -0.500000, 0.000000,
      0.500000,  0.000000,  0.000000,  -0.500000, 0.000000,  0.000000};

  std::vector<Eigen::Vector_t> points;
  points.resize(num_verts);

  memcpy(points.data(), points_data, sizeof(float) * 3 * num_verts);

  auto &faceBlk = blkmesh.mFaceBlock;
  auto &fvertBlk = blkmesh.mFvertBlock;
  auto &edgeBlk = blkmesh.mEdgeBlock;
  auto &vertBlk = blkmesh.mVertBlock;


  
  // create from data
  structure_from_indices(counts.data(), indices.data(), counts.size(),
                         indices.size(), faceBlk, fvertBlk, edgeBlk, vertBlk,
                         num_verts, num_edges);

  /// turns out you can compute the normals and areas at the same time in the
  /// same loop
  std::vector<Eigen::Vector_t> face_normals;
  face_normals.resize(faceBlk.size());
  auto all_areas = [&faceBlk, &fvertBlk, &edgeBlk, &vertBlk, &points,
                    &face_normals]()

      -> std::vector<float> {
    std::vector<float> areas;
    for (int i = 0; i < faceBlk.size(); i++) {

      areas.push_back(

          example_get_face_area(FaceHdl(i), faceBlk, fvertBlk, edgeBlk, vertBlk,
                                points.data(), {}, false, &face_normals[i])

      );
    }
    return areas;
  };

  auto areas_result = all_areas();

  int i = 0;
  for (auto area : areas_result) {
    printf("face %d area > %f\n", i++, area);
  }

  i = 0;
  for (auto normal : face_normals) {
    std::cout << "face " << (i++) << " > " << normal << '\n';
  }
}
