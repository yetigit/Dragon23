

#include <GnMesh/Mesh/BlockArrayKernel.hpp>

int main() {


    
  BlkAllocConfig01::init(MeshStoreConfig{(1u << 11), 64u, (1u << 10), 64u,
                                         (1u << 11), 64u, (1u << 10), 64u});

  BM_BlkArrayGeo<BlkAllocConfig01> mesh;

 

  auto &faceBlock = mesh.mFaceBlock;
  auto &edgeBlock = mesh.mEdgeBlock;
  auto &vertBlock = mesh.mVertBlock;
  auto &fvertBlock = mesh.mFvertBlock;

  int counts[]{6};

  int indices[]{0, 1, 2, 3, 4, 5};

  structure_from_indices(counts, indices, sizeof(counts) / sizeof(int),
                         sizeof(indices) / sizeof(int), faceBlock, fvertBlock,
                         edgeBlock, vertBlock, counts[0], counts[0]);

  FvertHdl prebeg;
  FvertHdl pre_end;

  // connecting through 1---newvert---4
  /*


  0-------------------5
  \                   \
  \                   \
  \                   \
  1         v         4
  \                   \
  \                   \
  \                   \
  2--------------------3
  */

  {
    FvertHdl first = faceBlock[0].fv;
    FvertHdl cur = first;
    FvertHdl curnext = cur;
    VertHdl curnext_v;

    do {
      cur = curnext;
      curnext = fvertBlock[cur.id].next;
      curnext_v = get_v(edgeBlock, fvertBlock, curnext);
      /*     std::cout << cur.id << '>';
           std::cout << curnext.id << '\n';*/
    } while (curnext_v != VertHdl(1));
    prebeg = cur;

    do {
      cur = curnext;
      curnext = fvertBlock[cur.id].next;
      curnext_v = get_v(edgeBlock, fvertBlock, curnext);
    } while (curnext_v != VertHdl(4));

    pre_end = cur;
  }

  //
  //
  // std::cout << get_v(edgeBlock, fvertBlock, pre_end).id << "\n";

  {
    FvertHdl first = faceBlock[0].fv;
    FvertHdl cur = first;
    FvertHdl curnext;

    do {
      std::cout << get_v(edgeBlock, fvertBlock, cur).id << ">";
      std::cout << cur.id << ", ";
      curnext = fvertBlock[cur.id].next;
    } while ((cur = curnext) != first);
  }
  std::cout << '\n';

  // auto faceverts = facefv_stack(faceBlock, fvertBlock, FaceHdl{0});
  // for (auto vi : faceverts) {
  //  std::cout << vi.id << ", ";
  //}
  // std::cout << '\n';

  make_face_on_top(1, prebeg, pre_end, faceBlock, fvertBlock, edgeBlock,
                   vertBlock);

#if 0

  auto otherthing =
      facefv_stack(faceBlock, fvertBlock,  FaceHdl{1});

#else
  auto otherthing =
      faceverts_stack(faceBlock, fvertBlock, edgeBlock, FaceHdl{1});

#endif

  for (auto vi : otherthing) {
    std::cout << vi.id << ", ";
  }

  std::cout << '\n';
  //
}