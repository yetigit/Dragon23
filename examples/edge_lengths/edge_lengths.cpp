

#include <GnMesh/Mesh/BlockArrayKernel.hpp>
#include <GnMesh/Mesh/Queries.hpp>
#include <GnMesh/Mesh/Construct.hpp>
#include <cmath>

#include <iostream>


#include <vector>


using Vector_t = Eigen::Vector_t;
using Vector2_t = Eigen::Vector_t;




template <class EdgeBlk>
void example_get_all_edge_lengths(EdgeBlk& edgeBlk, const Vector_t* in_points,
    std::vector<float>& out_lengths) {

    for (int i = 0; i < edgeBlk.size(); ++i) {
        out_lengths.push_back(
            (in_points[edgeBlk[i].v0.id] - in_points[edgeBlk[i].v1.id]).norm());
    }
}




int main() { 

    
  BlkAllocConfig01::init(MeshStoreConfig{(1u << 11), 64u, (1u << 10), 64u,
                                         (1u << 11), 64u, (1u << 10), 64u});

     
  BM_BlkArrayGeo<BlkAllocConfig01> blkmesh;

    printf("\n-----------------\n");


    constexpr unsigned num_verts = 26;
    constexpr unsigned num_edges = 26;



    // the mesh is a cube with 1lvl sub-d
    std::vector<int> counts{ 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4, 4

    };
    std::vector<int> indices{
      0, 14, 20, 17, 14, 1, 15, 20, 20, 15, 3, 16, 17, 20, 16, 2, 2, 16, 21, 8, 16, 3, 18, 21, 21, 18, 5, 19, 8, 21, 19, 4, 4, 19, 22, 11, 19, 5, 9, 22, 22, 9, 7, 10, 11, 22, 10, 6, 6, 10, 23, 13, 10, 7, 12, 23, 23, 12, 1, 14, 13, 23, 14, 0, 1, 12, 24, 15, 12, 7, 9, 24, 24, 9, 5, 18, 15, 24, 18, 3, 6, 13, 25, 11, 13, 0, 17, 25, 25, 17, 2, 8, 11, 25, 8, 4

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
        0.500000,  0.000000,  0.000000,  -0.500000, 0.000000,  0.000000 };


    std::vector<Eigen::Vector_t> points;
    points.resize(num_verts);

    memcpy(points.data(), points_data, sizeof(float) * 3 * num_verts);



    auto& faceBlk = blkmesh.mFaceBlock;
    auto& fvertBlk = blkmesh.mFvertBlock;
    auto& edgeBlk = blkmesh.mEdgeBlock;
    auto& vertBlk = blkmesh.mVertBlock;

    // create from data
    structure_from_indices(counts.data(), indices.data(), counts.size(),
        indices.size(), faceBlk, fvertBlk, edgeBlk, vertBlk,
        num_verts, num_edges);



    std::vector<float> edge_lengths;
    edge_lengths.reserve(edgeBlk.size());
     
    example_get_all_edge_lengths(edgeBlk, points.data(), edge_lengths); 

    int i = 0; 
    for (auto len : edge_lengths)
    {
        printf("edge %d length > %f\n" ,  i++,  len);
    }
 
} 
