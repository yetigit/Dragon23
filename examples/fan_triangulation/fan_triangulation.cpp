
#include <GnMesh/Mesh/BlockArrayKernel.hpp>
#include <GnMesh/Mesh/Queries.hpp>
#include <GnMesh/Mesh/Construct.hpp>

#include <cmath>

#include <iostream>

// you can visualize after i finish the IO first day of the week ok

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk,
          size_t FaceSizeHint = 4>
void example_fan_triangulate(
    FaceHdl face_handle, FaceBlk &faceBlk, FvertBlk &fvertBlk, EdgeBlk &edgeBlk,
    VertBlk &vertBlk,
    GNM_NAMESPACE::meta::WrapTValue<size_t, FaceSizeHint> faceSizeHintArg =
        GNM_NAMESPACE::meta::WrapTValue<size_t, FaceSizeHint>{}) {

  auto face_verts_now = faceverts_stack(faceBlk, fvertBlk, edgeBlk, face_handle,
                                        GNM_NAMESPACE::szarg<FaceSizeHint>());
  if (face_verts_now.size() < 4) {
    // assert(0 && "face is already a triangle");
    return;
  }

#ifndef NDEBUG
  printf("ok we can triangulate\n");
#endif //

  int num_triangles = face_verts_now.size() - 2; // always the case
  int num_face_to_add = num_triangles - 1;
  int num_edge_to_add = num_triangles - 1;

  int face_new_indices_start = faceBlk.size();
  const int face_start_const = face_new_indices_start;
  // int face_new_indices_end = face_new_indices_start + num_face_to_add;

  int edge_new_indices_start = edgeBlk.size();
  // int edge_new_indices_end = edge_new_indices_start + num_edge_to_add;

  int fvert_new_indices_start = fvertBlk.size();

  FvertHdl face_firstfv = faceBlk[face_handle.id].fv;
  FvertHdl cur = face_firstfv;
  FvertHdl triangle_lastfv = cur;
  FvertHdl triangle_second_fv = cur;
  FvertHdl triangle_firstfv = cur;

  int counter = num_edge_to_add;

  VertHdl alpha_vhdl = get_v(edgeBlk, fvertBlk, cur);
  int i = 0;

  llvm_vecsmall::SmallVector<FvertHdl, FaceSizeHint> followers, leaders;

  triangle_lastfv = fvertBlk[fvertBlk[cur.id].next.id].next;
  do {
    --counter;

    auto target_v = face_verts_now[i + 2];
    // triangle_lastfv = fvertBlk[fvertBlk[triangle_lastfv.id].next.id].next;
    triangle_second_fv = fvertBlk[triangle_second_fv.id].next;
    followers.push_back(triangle_second_fv);

    int new_Edge = edge_new_indices_start++;
    int new_Face = face_new_indices_start++;

    int new_Fvert = fvert_new_indices_start++;
    edgeBlk.pushBack(Edge{alpha_vhdl, target_v, {}, {}, FvertHdl(new_Fvert)});


    construct_add_to_edge_cycle(alpha_vhdl, EdgeHdl{new_Edge}, faceBlk,
                                fvertBlk, edgeBlk, vertBlk);
    construct_add_to_edge_cycle(target_v, EdgeHdl{new_Edge}, faceBlk, fvertBlk,
                                edgeBlk, vertBlk);

    fvertBlk.pushBack(Fvert{FVEHDL(new_Edge, !, ), FaceHdl{new_Face},
                            triangle_lastfv, FvertHdl{new_Fvert + 1}});

    leaders.push_back(FvertHdl{new_Fvert + 1});

    fvertBlk.pushBack(Fvert{FVEHDL(new_Edge, , ),
                            FaceHdl{fvertBlk[triangle_firstfv.id].f},
                            triangle_firstfv, FvertHdl{new_Fvert}});

    triangle_firstfv.id = new_Fvert;
    faceBlk.pushBack(Face{triangle_firstfv});
    ++fvert_new_indices_start; // for the second pushback

    i += 1;

    triangle_lastfv = fvertBlk[triangle_lastfv.id].next;

  } while ((cur = fvertBlk[cur.id].next), counter);

  fvertBlk[triangle_lastfv.id].next = triangle_firstfv;
  fvertBlk[triangle_firstfv.id].next = fvertBlk[cur.id].next;

  for (i = 0; i < followers.size(); i++) {
    fvertBlk[followers[i].id].next = leaders[i];
  }

  for (i = face_start_const; i < faceBlk.size(); i++) {

    face_firstfv = faceBlk[i].fv;
    cur = face_firstfv;
    do {
      fvertBlk[cur.id].f.id = i;

    } while ((cur = fvertBlk[cur.id].next) != face_firstfv);
  }
}

int main() {

  BlkAllocConfig01::init(MeshStoreConfig{(1u << 11), 64u, (1u << 10), 64u,
                                         (1u << 11), 64u, (1u << 10), 64u});


  printf("block array size %zu\n", sizeof(BlkAllocConfig01::VertBlockArray_t)); 
  BM_BlkArrayGeo<BlkAllocConfig01> blkmesh;

  printf("\n-----------------\n");

  // create a disc

  constexpr unsigned approx = 16;

  std::vector<int> counts(1, approx);
  std::vector<int> indices;
  std::vector<Eigen::Vector_t> points;

  points.reserve(approx);
  indices.reserve(approx);

  float increment = 2 * M_PI / approx;
  for (size_t i = 0; i < approx; i++) {
    indices.push_back(i);
    points.emplace_back(cos(i * increment), sin(i * increment), 0.0F);
  }

  auto &faceBlk = blkmesh.mFaceBlock;
  auto &fvertBlk = blkmesh.mFvertBlock;
  auto &edgeBlk = blkmesh.mEdgeBlock;
  auto &vertBlk = blkmesh.mVertBlock;

  // create from data
  structure_from_indices(counts.data(), indices.data(), counts.size(),
                         indices.size(), faceBlk, fvertBlk, edgeBlk, vertBlk,
                         points.size(), points.size());

  // triangulate face 0

  example_fan_triangulate(FaceHdl(0), faceBlk, fvertBlk, edgeBlk, vertBlk,
                          GNM_NAMESPACE::szarg<approx>());

  printf("triangles are done\n\n");

  // let's get an idea of what happened

  for (int i = 0; i < faceBlk.size(); i++) {
    printf("face %d > ", i);

#if 0
  
          auto face_verts = faceverts_stack(faceBlk, fvertBlk, edgeBlk,
          FaceHdl{ i }, GNM_NAMESPACE::szarg<3>());
#else
    // OR
    auto face_verts = faceverts_stack_int(
        faceBlk, fvertBlk, edgeBlk, FaceHdl{i}, GNM_NAMESPACE::szarg<3>());

#endif

    for (auto index : face_verts) {
      printf("%d, ", index);
    }
    printf("\n");
  }

  edge_info(edgeBlk, std::cout); 

  BlkAllocConfig01::destroy(); 
}