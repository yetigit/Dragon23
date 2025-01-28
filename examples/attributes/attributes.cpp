

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

  int weights{};
  float data[]{1.f, 0.5f, 0.2f, 0.23f};
  blkmesh.createAttribute(weights, GeoAttrib::kFloat, data, sizeof(data),
                          sizeof(data) / sizeof(float));

  unsigned buffer_sz{};
  auto buffer = blkmesh.getAttribute<float>(weights, buffer_sz);
  printf("%d ", buffer_sz);

  for (int i = 0; i < buffer_sz; i++) {
    printf("%.2f ", buffer[i]);
  }

  printf("\n-----------------\n");
}
