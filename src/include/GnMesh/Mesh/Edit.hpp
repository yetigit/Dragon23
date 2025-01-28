
#pragma once
//#include <GnMesh/Storage/BlockArray.hpp>
#include <GnMeshCommon/TParam.hpp>

#include <GnMeshCommon/SmallVector.h> //
//#include <GnMeshCommon/bitUtils.hpp>

#include <GnMeshCommon/IntTypes.hpp>

#include <GnMesh/Mesh/Items.hpp>
#include <GnMeshCommon/Specifiers.hpp>
#include <GnMesh/Mesh/Queries.hpp>
#include <GnMesh/Mesh/Flags.hpp>
#include <set>
 

/////////////////// EDGE /////////////////// ///////////////////
////////////////////// /////////////////// ///////////////////
////////////////////// /////////////////// ///////////////////
////////////////////// /////////////////// ///////////////////
//////////////////////



template <class EdgeBlk>
void assign_edge_vert(EdgeHdl edgehdl, EdgeBlk &edgeBlk, VertHdl to_replace,
                      VertHdl new_verthdl) {

  ((int *)&edgeBlk[edgehdl])[to_replace == edgeBlk[edgehdl].v1] =
      new_verthdl.id;
}

template <class EdgeBlk>
void assign_edge_vert_check(EdgeHdl edgehdl, EdgeBlk &edgeBlk,
                            VertHdl to_replace, VertHdl new_verthdl) {

  if (to_replace == edgeBlk[edgehdl.id].v1) {
    edgeBlk[edgehdl.id].v1 = new_verthdl;
  } else if (to_replace == edgeBlk[edgehdl.id].v0) {
    edgeBlk[edgehdl.id].v0 = new_verthdl;
  }
}

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
bool edge_collapse_repair(FaceBlk &faceBlk, FvertBlk &fvertBlk,
                          EdgeBlk &edgeBlk, VertBlk &vertBlk,
                          std::vector<unsigned> &flagsface,
                          std::vector<unsigned> &flagsfver,
                          std::vector<unsigned> &flagsedge,
                          std::vector<unsigned> &flagsvert, VertHdl delta,
                          int &numRepair) {

  assert(ItemFlags::is_off(flagsvert[delta], ItemFlags::DELETED));

  std::map<std::pair<VertHdl, VertHdl>, std::vector<EdgeHdl>> mapi;

  auto deltaRing = vert_edge_ring(vertBlk, fvertBlk, edgeBlk, delta);
  for (auto cur : deltaRing) {
    auto curpair = get_edge_vpair(cur, edgeBlk);
    if (curpair.first > curpair.second)
      std::swap(curpair.second, curpair.first);

    auto stlpair = std::make_pair(curpair.first, curpair.second);
    mapi[stlpair].push_back(cur);
  }

  struct FvertHdlFlip {
    FvertHdl fv;
    int flip;
  };

  for (auto &keyval : mapi) {
    auto &vpair = keyval.first;
    auto &edgeBuf = keyval.second;
    VertHdl va = vpair.first;
    VertHdl vb = vpair.second;

    if (edgeBuf.size() > 1) {
      // printf("entering big mac\n");
      std::vector<FvertHdlFlip> gather;

      EdgeHdl goodEdge = edgeBuf.back();
      // printf("good edge : %d\n", goodEdge.id);
      for (size_t i = 0; i < edgeBuf.size() - 1; i++) {
        EdgeHdl ei = edgeBuf[i];
        // printf("faulty double e : %d , va : %d, vb : %d\n", ei.id, va.id,
        // vb.id);

        assert(ItemFlags::is_off(flagsvert[va], ItemFlags::DELETED));
        assert(ItemFlags::is_off(flagsvert[vb], ItemFlags::DELETED));

        EdgeHdl nextEdge_a =
            remove_edge_fromcycle(edgeBlk, fvertBlk, vertBlk, va, ei, ei);
        EdgeHdl nextEdge_b =
            remove_edge_fromcycle(edgeBlk, fvertBlk, vertBlk, vb, ei, ei);
        ItemFlags::enable(flagsedge[ei], ItemFlags::DELETED);
      }

      for (size_t i = 0; i < edgeBuf.size(); i++) {
        EdgeHdl ei = edgeBuf[i];

        FvertHdl ei_fv = get_edge_fvert(ei, edgeBlk);
        auto ei_fv_cycle = fv_radial_stack(fvertBlk, ei_fv);

        // printf("num adj for e : %d = %d\n", ei.id, (int)ei_fv_cycle.size());

        int canon_v = (int)(va == edgeBlk[goodEdge].v1);
        int orig_v = (int)(va == edgeBlk[ei].v1);
        int flipValue = canon_v != orig_v;

        for (FvertHdl fvi : ei_fv_cycle)
          gather.push_back({fvi, flipValue});
      }
      // printf("num adj sum = %d\n", (int)gather.size());
#if 0
      size_t gather_previousSize = gather.size(); 

      std::sort(gather.begin(), gather.end());
      auto uitr = std::unique(gather.begin(), gather.end());
      gather.erase(uitr, gather.end());

      assert(gather_previousSize == gather.size());
#endif

      for (size_t i = 0; i < gather.size(); i++) {
        FvertHdl fvi = gather[i].fv;
        FvertHdl fvj = gather[(i + 1) % gather.size()].fv;
        fvertBlk[fvi].radial = fvj;
        fvertBlk[fvi].e.id = goodEdge.id;
        if (gather[i].flip) {
          unsigned e_local_idx = fvertBlk[fvi].e.v;
          fvertBlk[fvi].e.v = !e_local_idx;
        }
      }
      numRepair += 1;
    }
  }

  return true;
}


template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
bool edge_collapse_ok(

    FaceBlk &faceBlk, FvertBlk &fvertBlk, EdgeBlk &edgeBlk, VertBlk &vertBlk,
    EdgeHdl edge_hdl,

    std::vector<unsigned> &flagsface, std::vector<unsigned> &flagsfver,
    std::vector<unsigned> &flagsedge, std::vector<unsigned> &flagsvert,
    VertHdlPair keep_and_kill) {

  if (ItemFlags::is_on(flagsedge[edge_hdl], ItemFlags::DELETED))
    return false;

  auto vpair = get_edge_vpair(edge_hdl, edgeBlk);
   if (vpair.first > vpair.second) 
      std::swap(vpair.second, vpair.first);

  if (vpair.first == vpair.second) {
    return false;
  }
  if (ItemFlags::is_on(flagsvert[vpair.first], ItemFlags::DELETED) ||
      ItemFlags::is_on(flagsvert[vpair.second], ItemFlags::DELETED)) {
    return false;
  }


  FvertHdl edge_fv = get_edge_fvert(edge_hdl, edgeBlk);

  if (ItemFlags::is_on(flagsfver[edge_fv], ItemFlags::DELETED))
    return false;

     
  #if 1 
  {

    VertHdl delta = vpair.first;

    auto deltaRing = vert_edge_ring(vertBlk, fvertBlk, edgeBlk, delta);
    int eq_counter = 0;
    for (auto cur : deltaRing) {
      auto curpair = get_edge_vpair(cur, edgeBlk);
      if (curpair.first > curpair.second)
        std::swap(curpair.second, curpair.first);

      if (vpair.first == curpair.first && curpair.second == vpair.second) 
        eq_counter += 1;
    }

    if (eq_counter > 1) {
      //printf("DOUBLE EDGE\n");
      return false;
    }
  } 
  #endif 


  #if 0
  /// note: this clause happens to protect against non manifold edges
  if (

      is_boundary_vert(vpair.first, fvertBlk, edgeBlk, vertBlk) &&
      is_boundary_vert(vpair.second, fvertBlk, edgeBlk, vertBlk)

      && !is_boundary_edge(edge_hdl, faceBlk, fvertBlk, edgeBlk, vertBlk)) {

    return false;
  }
  #endif

  return true;
}

template <class FvertBlk>
FvertHdl remove_fv_from_radial_cycle_(FvertBlk &fvertBlk, FvertHdl fvh) {

  FvertHdl cur = fvh;
  FvertHdl first = cur;
  FvertHdl curnext = fvertBlk[cur].radial;
  if (!curnext.isValid() || curnext == cur) {
    return FvertHdl{};
  }

  do {
    curnext = fvertBlk[cur].radial;
    if (curnext == first) {

      FvertHdl nextnext = fvertBlk[curnext].radial;
      fvertBlk[cur].radial = nextnext;
      return nextnext;
    }

  } while ((cur = curnext) != first);

  return FvertHdl{};
}

template <class EdgeBlk, class FvertBlk, class VertBlk>
EdgeHdl remove_edge_fromcycle(EdgeBlk &edgeBlk, FvertBlk &fvertBlk,
                              VertBlk &vertBlk, VertHdl va, EdgeHdl to_remove,
                              EdgeHdl firstEdge = EdgeHdl{}) {

  EdgeHdl first;
  if (firstEdge.isValid()) {
       first = firstEdge; 
  } else {
	   first = get_vert_edge(va, fvertBlk, vertBlk);
  }

  EdgeHdl cur = first;
  EdgeHdl *curnext;
  do {

    curnext = get_next_edge(va, cur, edgeBlk);
    if (*curnext == to_remove) {
      *curnext = *get_next_edge(va, *curnext, edgeBlk);
      return *curnext;
    }

  } while ((cur = *curnext) != first);
  return EdgeHdl{};
}


// will merge cycle a into cycle b
template <class EdgeBlk, class FvertBlk, class VertBlk, class StackT>
void merge_edge_cycle(EdgeBlk &edgeBlk, FvertBlk &fvertBlk, VertBlk &vertBlk,
                      VertHdl va, VertHdl vb,

                      StackT const &cycle_a, StackT const &cycle_b) {

  assert(cycle_a.size());
  assert(cycle_b.size());

  std::vector<EdgeHdl> edgeset;

  for (const auto &cur_a : cycle_a)
    edgeset.push_back(cur_a);

  for (const auto &cur_b : cycle_b)
    edgeset.push_back(cur_b);

  std::sort(edgeset.begin(), edgeset.end());

  auto edgeset_end = std::unique(edgeset.begin(), edgeset.end());

  size_t edgeset_sz = std::distance(edgeset.begin(), edgeset_end);

#if 1
  for (EdgeHdl cur : cycle_b) {
    if (!edge_has_vert(cur, va, edgeBlk)) {
      assign_edge_vert(cur, edgeBlk, vb, va);
    }
  }
#endif

  for (auto i = 0; i < edgeset_sz; ++i) {

    EdgeHdl ei = edgeset[i];
    EdgeHdl ej = edgeset[(i + 1) % edgeset_sz];

    EdgeHdl *ei_next = get_next_edge(va, ei, edgeBlk);
    *ei_next = ej;
  }
}

template <class FvertBlk, class EdgeBlk>
void collapse_assign_fusingEdge_to_radials(FvertBlk &fvertBlk, EdgeBlk &edgeBlk,
                                           FvertHdl fvh, EdgeHdl to_assign,
                                           VertHdl vkeep) {

  FvertHdl first = fvh;
  FvertHdl cur = first;

  EdgeHdl oppositeEdge = fvertBlk[fvh].e;
  assert(oppositeEdge != to_assign);

  assert(edge_has_vert(to_assign, vkeep, edgeBlk));
  assert(edge_has_vert(oppositeEdge, vkeep, edgeBlk));

  int canon_v = (int)(vkeep == edgeBlk[to_assign].v1);
  int orig_v = (int)(vkeep == edgeBlk[oppositeEdge].v1);
  int flip = canon_v != orig_v;

  do {
    fvertBlk[cur].e.id = (unsigned)to_assign.id;
    unsigned v01 = fvertBlk[cur].e.v;
    if (flip)
      fvertBlk[cur].e.v = !v01;

  } while ((cur = fvertBlk[cur].radial) != first);
}

template <class FvertBlk, class Func>
FvertHdl find_suitable_radial(const FvertBlk &fvertBlk, FvertHdl fvh,
                              const Func &filterFunc) {

  FvertHdl first = fvh;
  FvertHdl cur = first;

  do {
    if (filterFunc(cur)) {
      return cur;
    }

  } while ((cur = fvertBlk[cur].radial) != first);

  return FvertHdl{};
}


template <class FvertBlk,  class EdgeBlk , class Func>
EdgeHdl find_suitable_edge(
    VertHdl vhdl,
    EdgeHdl ehdl,
    const FvertBlk & fvertBlk, 
    const EdgeBlk & edgeBlk, 
    const Func &  filterFunc) { 
 
 
    EdgeHdl first = ehdl; 
    EdgeHdl cur = first; 
    do {
      if ( 
          filterFunc(cur)
           
          ) {
        return cur;
      }

	} while (( cur = *get_next_edge(vhdl, cur, edgeBlk))!= first );    

    return EdgeHdl{}; 
     
}


template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
VertHdl edge_collapse(FaceBlk &faceBlk, FvertBlk &fvertBlk, EdgeBlk &edgeBlk,
                   VertBlk &vertBlk, EdgeHdl edge_hdl,

                   std::vector<unsigned> &flagsface,
                   std::vector<unsigned> &flagsfver,
                   std::vector<unsigned> &flagsedge,
                   std::vector<unsigned> &flagsvert, VertHdlPair keep_and_kill,
                   std::vector<Eigen::Vector_t> &pointsRef, VertHdl & collapsedOn) {

  VertHdl vertToKeep = keep_and_kill.first;
  VertHdl vertToKill = keep_and_kill.second;

  pointsRef[vertToKeep] = pointsRef[vertToKeep] + pointsRef[vertToKill];
  pointsRef[vertToKeep] *= 0.5f;

  assert(edge_has_vert(edge_hdl, vertToKeep, edgeBlk));
  assert(edge_has_vert(edge_hdl, vertToKill, edgeBlk));

  ItemFlags::enable(flagsedge[edge_hdl], ItemFlags::DELETED);

  // get edge adjacent faces
  EdgeAdjacencyResult edge_faces =
      edge_adjacent_faces(edge_hdl, fvertBlk, edgeBlk);

  //printf("num adj : %zu\n", edge_faces.adj_faces.size());

  llvm_vecsmall::SmallVector<EdgeHdl, 2> taggedEdges;

  // warning /!\ could have duplicates
  llvm_vecsmall::SmallVector<VertHdl, 2> x_taggedVerts;
  llvm_vecsmall::SmallVector<EdgeHdl, 4> x_taggedEdges;

  {
    /// tag all edge we may fuse ,
    /// meaning all the edges that can be found along the adjacent faces of the
    /// collapse edge and have vertTokill none of these may be edge_hdl.
    for (FaceHdl cur : edge_faces.adj_faces) {

      auto fvStack = facefv_stack(faceBlk, fvertBlk, cur);

      for (FvertHdl curfv : fvStack) {
        EdgeHdl cur_e = (EdgeHdl)fvertBlk[curfv].e;
        VertHdl cur_v = get_v(edgeBlk, fvertBlk, curfv);

        int onTriangle = is_on_triangle(curfv, fvertBlk);
        if (onTriangle) {


            auto cur_v_edgeRing3 =
                vert_edge_ring3(vertBlk, fvertBlk, edgeBlk, cur_v);

            if (cur_v_edgeRing3.size() < 3) {

              if (cur_v != vertToKeep && cur_v != vertToKill) {
                ItemFlags::enable(flagsvert[cur_v], ItemFlags::TAGGED_X);
                x_taggedVerts.push_back(cur_v);
              }

              for (auto xedge : cur_v_edgeRing3) {
                if (xedge != edge_hdl && !edge_has_vert(xedge, vertToKill, edgeBlk)) {
                  ItemFlags::enable(flagsedge[xedge], ItemFlags::TAGGED_X);
                  x_taggedEdges.push_back(xedge);
                }
              }
            }


          if (cur_e != edge_hdl && edge_has_vert(cur_e, vertToKill, edgeBlk)) {
            taggedEdges.push_back(cur_e);
            ItemFlags::enable(flagsedge[cur_e], ItemFlags::TAGGED);
          }
        }
      }
    }
  }

  auto cycle_a = vert_edge_ring(vertBlk, fvertBlk, edgeBlk, vertToKeep);
  auto cycle_b = vert_edge_ring(vertBlk, fvertBlk, edgeBlk, vertToKill);


  // bring edges from vkill to vkeep
  merge_edge_cycle(edgeBlk, fvertBlk, vertBlk, vertToKeep, vertToKill, cycle_a,
                   cycle_b);

  // remove edge_hdl which we dont want in there
  EdgeHdl legalEdge =
      remove_edge_fromcycle(edgeBlk, fvertBlk, vertBlk, vertToKeep, edge_hdl);
  assert(legalEdge.isValid());

  // --------------------------------------------

  // rerout fvert to skip deleted
  for (FaceHdl curf : edge_faces.adj_faces) {

    assert(curf.isValid());
    FvertHdl first = faceBlk[curf].fv;
    FvertHdl cur = first;
    FvertHdl curnext;
    do {

      curnext = fvertBlk[cur].next;
      if (fvertBlk[curnext].e == edge_hdl) {

        VertHdl curnext_v = get_v(edgeBlk, fvertBlk, curnext);
        // thing to reroute to
        FvertHdl curnext_next = fvertBlk[curnext].next;

        // give vert suitable fvert candidate
        if (curnext_v == vertToKeep) {
          vertBlk[vertToKeep].fv = curnext_next;
        }

#if 1
        // not really necessary
        (void)remove_fv_from_radial_cycle_(fvertBlk, curnext);
#endif

        // delete this fvert
        ItemFlags::enable(flagsfver[curnext], ItemFlags::DELETED);
        // bypass deleted
        fvertBlk[cur].next = curnext_next;
        // give suitable fvert for this face
        faceBlk[curf].fv = curnext_next;
        break;
      }

    } while ((cur = curnext) != first);
  }

// isolate vert to kill
#if 1
  vertBlk[vertToKill].fv.invalidate();
#endif

  ItemFlags::enable(flagsvert[vertToKill], ItemFlags::DELETED);

  // -------------------------------------------------

  llvm_vecsmall::SmallVector<EdgeHdl, 2> fusedEdges;
  // remove illegals
  for (FaceHdl curf : edge_faces.adj_faces) {

    // it could be invalid
    assert(curf.isValid());

    FvertHdl curf_fv = faceBlk[curf].fv;

    // face count is two , the adjacent face was thus a triangle awaiting kill
    bool isIllegal = fvertBlk[fvertBlk[curf_fv].next].next == curf_fv;
    if (isIllegal) {

      ItemFlags::enable(flagsface[curf], ItemFlags::DELETED);

      auto curf_fvs = facefv_stack(faceBlk, fvertBlk, curf_fv);

      for (FvertHdl inbound : curf_fvs) {

        auto cur_e = (EdgeHdl)fvertBlk[inbound].e;
        ItemFlags::enable(flagsfver[inbound], ItemFlags::DELETED);

        // check if the fv of this edge is deleted and replace it
        if (edgeBlk[cur_e].fv == inbound) {

          // find suitable replacement fv
          FvertHdl suitable_fv = find_suitable_radial(
              fvertBlk, inbound, [&flagsfver](FvertHdl curarg) {
                return ItemFlags::is_off(flagsfver[curarg.id],
                                         ItemFlags::DELETED);
              });

          edgeBlk[cur_e].fv = suitable_fv;
          #if 1
          VertHdl cur_ev0 = edgeBlk[cur_e].v0;
          VertHdl cur_ev1 = edgeBlk[cur_e].v1;
          bool v0isLambda = cur_ev0 != vertToKeep && cur_ev0 != vertToKill;
          bool v1isLambda = cur_ev1 != vertToKeep && cur_ev1 != vertToKill;

          if (suitable_fv.isValid()) {

              #if 1
			  if (v0isLambda)
				vertBlk[cur_ev0].fv = suitable_fv;

			  if (v1isLambda)
				vertBlk[cur_ev1].fv = suitable_fv;
              #endif 

          }
          #endif


        }

        bool fuseIt = false;

        // if a tagged is within a problematic face we need to fuse it
        if (ItemFlags::is_on(flagsedge[cur_e], ItemFlags::TAGGED)) {

          // because this edge is about to get fused
          ItemFlags::enable(flagsedge[cur_e], ItemFlags::DELETED);
          fuseIt = true;
        }

        // merge the radials like you would merge two circular linked list
        if (fuseIt) {
          fusedEdges.push_back(cur_e);
          ItemFlags::disable(flagsedge[cur_e], ItemFlags::TAGGED);

          FvertHdl fuseWith = fvertBlk[inbound].next;

          EdgeHdl fuseWith_edge = fvertBlk[fuseWith].e;



#if 1

          collapse_assign_fusingEdge_to_radials(fvertBlk, edgeBlk, inbound,
                                                fuseWith_edge, vertToKeep);
#endif

          assert(fuseWith != inbound);

          FvertHdl inbound_next = fvertBlk[inbound].radial;
          fvertBlk[inbound].radial = fuseWith;

          auto chain1 = fv_radial_stack(fvertBlk, fuseWith);
          fvertBlk[chain1.back()].radial = inbound_next;

          assert(edge_has_vert(cur_e, vertToKeep, edgeBlk));
          VertHdl v_lambda = get_other_vert(cur_e, vertToKeep, edgeBlk);
          assert(v_lambda != vertToKeep);
          EdgeHdl e_lambda= remove_edge_fromcycle(edgeBlk, fvertBlk, vertBlk, v_lambda,
                                      cur_e, cur_e);

          #if 1

          EdgeHdl suitableEdge = find_suitable_edge(
              v_lambda, e_lambda, fvertBlk, edgeBlk,
              [&flagsedge, &flagsfver, &edgeBlk](EdgeHdl edgeCandidate) {
                FvertHdl candidateFv = edgeBlk[edgeCandidate].fv;
                if (candidateFv.isValid())
                  return ItemFlags::is_off(flagsfver[candidateFv],
                                           ItemFlags::DELETED) &&
                         ItemFlags::is_off(flagsedge[edgeCandidate],
                                           ItemFlags::DELETED) &&
                         ItemFlags::is_off(flagsedge[edgeCandidate],
                                           ItemFlags::TAGGED_X);

                return false;
              });

          if (suitableEdge.isValid()) {
            vertBlk[v_lambda].fv = edgeBlk[suitableEdge].fv;
          }

#if 0
			  assert(e_lambda != cur_e);
			  if ( 
				  e_lambda.isValid() 
				   && 
				  ItemFlags::is_off(flagsedge[e_lambda], ItemFlags::DELETED) ) {
				//printf("%d\n", v_lambda.id);   
				vertBlk[v_lambda].fv = edgeBlk[e_lambda].fv;
			  }
#endif

#endif


        }
      }
    }
  }

#if 1
  for (auto cur : x_taggedEdges) {
    if (ItemFlags::is_off(flagsedge[cur], ItemFlags::DELETED)) {

      VertHdl v_charlie = edgeBlk[cur].v0;
      VertHdl v_bravo = edgeBlk[cur].v1;
      assert(v_charlie != v_bravo);

      if (v_charlie != vertToKeep && v_charlie != vertToKill) {
        remove_edge_fromcycle(edgeBlk, fvertBlk, vertBlk, v_charlie, cur, cur);
      }
      if (v_bravo != vertToKeep && v_bravo != vertToKill) {
        remove_edge_fromcycle(edgeBlk, fvertBlk, vertBlk, v_bravo, cur, cur);
      }

      ItemFlags::enable(flagsedge[cur], ItemFlags::DELETED);
    }
  }

  for (auto cur : x_taggedVerts)
    ItemFlags::enable(flagsvert[cur], ItemFlags::DELETED);
#endif

    // -------------------------------------------------

#if 1

  for (FaceHdl curf : edge_faces.adj_faces) {
    FvertHdl curf_fv = faceBlk[curf].fv;

    bool isIllegal = fvertBlk[fvertBlk[curf_fv].next].next == curf_fv;
    if (isIllegal) {

      auto curf_fvs = facefv_stack(faceBlk, fvertBlk, curf_fv);
      for (FvertHdl inbound : curf_fvs) {
        (void)remove_fv_from_radial_cycle_(fvertBlk, inbound);
      }
    }
  }

#endif

  // remove fused edges from the edge cycle
  // -------------------------------------------------

  // auto vring1 = vert_edge_ring(vertBlk, fvertBlk, edgeBlk, vertToKeep);
  bool vKeepDeleted = false; 
#if 1
  {

    std::vector<EdgeHdl> keepEdges;

    // this is safe because vkeep fv got assigned to a fv from an edge of vkill
    for (auto &cur : cycle_a)
      if (ItemFlags::is_off(flagsedge[cur], ItemFlags::DELETED)) {
        FvertHdl cur_efv = get_edge_fvert(cur, edgeBlk);
        if (cur_efv.isValid()) {

			assert(ItemFlags::is_off(flagsfver[cur_efv], ItemFlags::DELETED));
			// assign new legit fv to vkeep
			vertBlk[vertToKeep].fv = cur_efv;

			keepEdges.push_back(cur);
        }
      }

    for (auto &cur : cycle_b)
      if (ItemFlags::is_off(flagsedge[cur], ItemFlags::DELETED)) {
        FvertHdl cur_efv = get_edge_fvert(cur, edgeBlk);
        if (cur_efv.isValid()) {

			assert(ItemFlags::is_off(flagsfver[cur_efv], ItemFlags::DELETED));
			// assign new legit fv to vkeep
			vertBlk[vertToKeep].fv = cur_efv;

			keepEdges.push_back(cur);
        }
      }

    if (keepEdges.size()) {
      std::sort(keepEdges.begin(), keepEdges.end());
      auto newend = std::unique(keepEdges.begin(), keepEdges.end());
      keepEdges.erase(newend, keepEdges.end());
    }

    if (keepEdges.empty()) {
	  ItemFlags::enable(flagsvert[vertToKeep], ItemFlags::DELETED);
      vertBlk[vertToKeep].fv.invalidate();
      vKeepDeleted = true; 
    }
    
    for (size_t i = 0; i < keepEdges.size(); i++) {
      auto ei = keepEdges[i];
      auto ej = keepEdges[(i + 1) % keepEdges.size()];
      EdgeHdl *nextPtr = get_next_edge(vertToKeep, ei, edgeBlk);
      *nextPtr = ej;
    }
  }
#endif

  ///  ------------

  for (auto cur : taggedEdges)
    ItemFlags::disable(flagsedge[cur], ItemFlags::TAGGED);

  for (auto cur : x_taggedEdges)
    ItemFlags::disable(flagsedge[cur], ItemFlags::TAGGED_X);

  for (auto cur : x_taggedVerts)
    ItemFlags::disable(flagsvert[cur], ItemFlags::TAGGED_X);

#if 0
  for (auto &cur : flagsedge) 
    {
      assert(ItemFlags::is_off(cur, ItemFlags::TAGGED));
	}
#endif

  if ( vKeepDeleted ) {
    collapsedOn.invalidate(); 
  } else {
    collapsedOn = vertToKeep;
  }
  return true;
}

/// @brief inserts a Vert on an Edge
/// @tparam VertBlk
/// @tparam FaceBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param edge_hdl
/// @param faceBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk
/// @return new Vert

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
VertHdl edge_insert_vert(EdgeHdl edge_hdl /* edge to insert the vertex onto */,
                         FaceBlk &faceBlk, FvertBlk &fvertBlk, EdgeBlk &edgeBlk,
                         VertBlk &vertBlk) {

  // std::cout << "edge_insert_vert ..\n";

  int new_vertid = vertBlk.size();
  int new_edgeid = edgeBlk.size();

  int new_fvert0_id = fvertBlk.size();
  int new_fvert1_id = new_fvert0_id + 1;

  FvertHdl edge_fvert;
  edge_fvert = get_edge_fvert(edge_hdl, edgeBlk);
  //  edge_fvert.id = 3;

  VertHdl vert_alpha = get_v(edgeBlk, fvertBlk, edge_fvert);
  VertHdl vert_beta = get_v(edgeBlk, fvertBlk, fvertBlk[edge_fvert.id].next);

  vertBlk.pushBack(Vert{});

  edgeBlk.pushBack(Edge{VertHdl(new_vertid), vert_beta, {}, {}});
  fvertBlk.pushBack(Fvert{});
  fvertBlk.pushBack(Fvert{});

  auto tmp0next = fvertBlk[edge_fvert.id].next;
  auto tmp0radial = fvertBlk[edge_fvert.id].radial;

  fvertBlk[edge_fvert.id].next.id = new_fvert0_id;
  fvertBlk[new_fvert0_id].next = tmp0next;
  fvertBlk[edge_fvert.id].radial.id = new_fvert1_id;
  fvertBlk[new_fvert1_id].radial = edge_fvert;

  auto tmp1next = fvertBlk[tmp0radial.id].next;

  fvertBlk[tmp0radial.id].next.id = new_fvert1_id;
  fvertBlk[new_fvert1_id].next = tmp1next;
  fvertBlk[tmp0radial.id].radial.id = new_fvert0_id;
  fvertBlk[new_fvert0_id].radial = tmp0radial;

  vertBlk[new_vertid].fv.id = new_fvert0_id;
  vertBlk[vert_beta.id].fv.id = new_fvert0_id;
  vertBlk[vert_alpha.id].fv = edge_fvert;

  //

  // f
  fvertBlk[new_fvert1_id].f = fvertBlk[tmp0radial.id].f;
  fvertBlk[new_fvert0_id].f = fvertBlk[edge_fvert.id].f;

  // edge_fvert
  fvertBlk[tmp0radial.id].e = FVEHDL(new_edgeid, , );
  fvertBlk[new_fvert0_id].e = FVEHDL(new_edgeid, !, );

  auto relpos = (unsigned)(vert_beta == edgeBlk[edge_hdl.id].v1);

  auto new_fve = FVEHDL(edge_hdl.id, , );
  new_fve.v = relpos;

  fvertBlk[new_fvert1_id].e = new_fve;
  new_fve.v = !relpos;
  fvertBlk[edge_fvert.id].e = new_fve;

  //
#if 1

  edgePopOff(edge_hdl, vert_beta, faceBlk, edgeBlk, vertBlk);

#endif

  reinterpret_cast<int *>(&edgeBlk[edge_hdl.id])[relpos] = new_vertid;
  reinterpret_cast<int *>(&edgeBlk[edge_hdl.id])[relpos + 2] =
      -1; // make it invalid
          // std::cout   << "OKAY\n" ;

  construct_add_to_edge_cycle(vert_beta, EdgeHdl{new_edgeid}, faceBlk, fvertBlk,
                              edgeBlk, vertBlk);
  // std::cout << "NOWAY\n";

  construct_add_to_edge_cycle(VertHdl(new_vertid), EdgeHdl{new_edgeid}, faceBlk,
                              fvertBlk, edgeBlk, vertBlk);

  construct_add_to_edge_cycle(VertHdl(new_vertid), edge_hdl, faceBlk, fvertBlk,
                              edgeBlk, vertBlk);

  return VertHdl(new_vertid);
}
/// @brief creates an Edge starting at a Vert and landing on an Edge
/// @tparam VertBlk
/// @tparam FaceBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param va_hdl
/// @param eb_hdl
/// @param faceBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk
template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk,
          size_t FaceSizeHint = 4>
void vert_edge_connect(VertHdl va_hdl, EdgeHdl eb_hdl, FaceBlk &faceBlk,
                       FvertBlk &fvertBlk, EdgeBlk &edgeBlk, VertBlk &vertBlk) {

  auto verte = vert_edge_ring(vertBlk, fvertBlk, edgeBlk, va_hdl);

  VertHdl edgeB_v =
      edge_insert_vert(eb_hdl, faceBlk, fvertBlk, edgeBlk, vertBlk);

  FaceHdl face_to_connect_on = verts_common_face(va_hdl, edgeBlk[eb_hdl.id].v0,
                                                 fvertBlk, edgeBlk, vertBlk);

  vert_vert_connect(va_hdl, edgeB_v, face_to_connect_on, faceBlk, fvertBlk,
                    edgeBlk, vertBlk);
}

/// @brief connects an Edge A to an Edge B, creating a new Edge in the process
/// @tparam VertBlk
/// @tparam FaceBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param ea_hdl
/// @param eb_hdl
/// @param faceBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk
template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk,
          size_t FaceSizeHint = 4>
void edge_edge_connect(EdgeHdl ea_hdl, EdgeHdl eb_hdl, FaceBlk &faceBlk,
                       FvertBlk &fvertBlk, EdgeBlk &edgeBlk, VertBlk &vertBlk) {

  // auto edgeA_v0 = edgeBlk[ea_hdl.id].v0;
  // auto edgeA_v1 = edgeBlk[ea_hdl.id].v1;

  // auto edgeB_v0 = edgeBlk[eb_hdl.id].v0;
  // auto edgeB_v1 = edgeBlk[eb_hdl.id].v1;

  VertHdl edgeA_v =
      edge_insert_vert(ea_hdl, faceBlk, fvertBlk, edgeBlk, vertBlk);

  VertHdl edgeB_v =
      edge_insert_vert(eb_hdl, faceBlk, fvertBlk, edgeBlk, vertBlk);
 
   
  auto split_common_face = verts_common_faces(edgeA_v, edgeB_v, fvertBlk, edgeBlk, vertBlk);
  FaceHdl face_to_connect_on = split_common_face[0]; 
      

  vert_vert_connect(edgeA_v, edgeB_v, face_to_connect_on, faceBlk, fvertBlk,
                    edgeBlk, vertBlk);
}

// assume edge is not already in the cycle 
template <class EdgeBlk, class VertBlk, class FvertBlk>
void add_edge_to_cycle(EdgeHdl ehdl, VertHdl vhdl, EdgeBlk &edgeBlk,
                       FvertBlk &fvertBlk, VertBlk &vertBlk) {

  assert(edge_has_vert(ehdl, vhdl, edgeBlk));

  auto edge_ring = vert_edge_ring(vertBlk, fvertBlk, edgeBlk, vhdl);
  *get_next_edge(vhdl, ehdl, edgeBlk) = edge_ring[0];
  *get_next_edge(vhdl, edge_ring.back(), edgeBlk) = ehdl;
}





/// @brief connects a Vert A to another Vert B
/// @tparam VertBlk
/// @tparam FaceBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param va_hdl
/// @param vb_hdl
/// @param face_handle Face to connect it on
/// @param faceBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk
/// @param faceSizeHintArg
template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
FaceHdl vert_vert_connect(
    VertHdl va_hdl, VertHdl vb_hdl,
    FaceHdl face_handle /*which face to connect the pair on */,
    FaceBlk &faceBlk, FvertBlk &fvertBlk, EdgeBlk &edgeBlk, VertBlk &vertBlk) 
{

  if (!face_handle.isValid()) {
    auto common_faces =
        verts_common_faces(va_hdl, vb_hdl, fvertBlk, edgeBlk, vertBlk);
    assert(common_faces.size());
    face_handle = common_faces[0];
  }

  FvertHdl va_prefv, vb_prefv;
  bool consecutive = false;
  bool inface =
      face_ccw_vert_pair_info(va_hdl, vb_hdl, face_handle, faceBlk, fvertBlk,
                              edgeBlk, va_prefv, vb_prefv, consecutive);
  assert(inface);
  assert(!consecutive);

  FvertHdl pre_newFv_0 = va_prefv;
  FvertHdl pre_newFv_1 = vb_prefv;

  FvertHdl newFv_0_next = fvertBlk[vb_prefv].next;
  FvertHdl newFv_1_next = fvertBlk[va_prefv].next;

  FvertHdl newFv_0 = fvertBlk.size();
  FvertHdl newFv_1 = fvertBlk.size() + 1;
  FaceHdl newFace = faceBlk.size();
  EdgeHdl newEdge = edgeBlk.size();

  edgeBlk.pushBack(Edge{va_hdl, vb_hdl, newEdge, newEdge, FvertHdl(newFv_0)});

  fvertBlk.pushBack(
      Fvert{FVEHDL(newEdge.id, !, ), newFace, newFv_0_next, newFv_1});
  fvertBlk.pushBack(
      Fvert{FVEHDL(newEdge.id, , ), face_handle, newFv_1_next, newFv_0});

  faceBlk.pushBack(Face{newFv_0});

  fvertBlk[pre_newFv_0].next = newFv_0;
  fvertBlk[pre_newFv_1].next = newFv_1;

  faceBlk[face_handle].fv = newFv_1;

  for (auto &&cur : facefv_stack(faceBlk, fvertBlk, newFace)) {
    fvertBlk[cur].f = newFace;
  }

  add_edge_to_cycle(newEdge, va_hdl, edgeBlk, fvertBlk, vertBlk);
  add_edge_to_cycle(newEdge, vb_hdl, edgeBlk, fvertBlk, vertBlk);


  return newFace;
}

/// @brief delete an edge superficially : no faces are removed in the process
///           so using this to delete an edge in the middle of two triangles
///           will result in a quad
/// @tparam VertBlk
/// @tparam FaceBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param deadface
/// @param deadfvert
/// @param deadedge
/// @param deadvert
/// @param edgehdl
/// @param faceBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
void delete_edge_superficial(
    BitArray &deadface /* bit array indicating deleted faces */,
    BitArray &deadfvert /* bit array indicating deleted fverts */,
    BitArray &deadedge /* bit array indicating deleted edges */,
    const BitArray &deadvert /* bit array indicating deleted verts */,
    EdgeHdl edgehdl, FaceBlk &faceBlk, FvertBlk &fvertBlk, EdgeBlk &edgeBlk,
    VertBlk &vertBlk) {

  /// when you use that one, no vert ever gets deleted
  /// also, only one face ever gets deleted and this face is also 'superficial'
  /// in the sense that deleting it does not create a border
  /// a superficial edge cannot be a border edge
  /// a superficial edge cannot be a non manifold
  // TODO do just another little check for mistakes later, but seems really good

  const VertHdl v0 = edgeBlk[edgehdl.id].v0;
  const VertHdl v1 = edgeBlk[edgehdl.id].v1;

  FvertHdl edge_fv =
      get_edge_fvert(edgehdl, faceBlk, fvertBlk, edgeBlk, vertBlk);

  FaceHdl adjacent0, adjacent1;
  adjacent0 = fvertBlk[edge_fv.id].f;
  adjacent1 = fvertBlk[fvertBlk[edge_fv.id].radial.id].f;

  // printf("adjacent0 %d\n", adjacent0.id);
  // printf("adjacent1 %d\n", adjacent1.id);

  if (adjacent0 == adjacent1) {
    return;
  }

  // delete the superficial edge
  deadedge.turnBitOff(edgehdl.id);

  // delete the superficial face
  deadface.turnBitOff(adjacent0.id);

  // pop the edge off the respective cycle of its verts
  edgePopOff(edgehdl, v0, faceBlk, fvertBlk, edgeBlk, vertBlk);
  edgePopOff(edgehdl, v1, faceBlk, fvertBlk, edgeBlk, vertBlk);

  FvertHdl first;
  FvertHdl cur;
  FvertHdl curnext;
  first = cur = curnext = edge_fv;

  // get the next fvert
  FvertHdl lead0 = fvertBlk[edge_fv.id].next;
  FvertHdl follower0;

  // get the fvert preceding
  do {
    cur = curnext;
    curnext = fvertBlk[cur.id].next;
  } while (curnext != first);
  follower0 = cur;
  // printf("fol0=%d\n", follower0.id);
  // printf("lead0=%d\n", lead0.id);

  // now in the adjacent face
  edge_fv = fvertBlk[edge_fv.id].radial;
  first = cur = curnext = edge_fv;

  // get the next fvert
  FvertHdl lead1 = fvertBlk[edge_fv.id].next;
  FvertHdl follower1;

  // get the fvert preceding
  do {
    cur = curnext;
    curnext = fvertBlk[cur.id].next;
  } while (curnext != first);
  follower1 = cur;

  // printf("fol1=%d\n", follower1.id);
  // printf("lead1=%d\n", lead1.id);

  // kill the fvert of the sup edge
  deadfvert.turnBitOff(edge_fv.id);
  deadfvert.turnBitOff(fvertBlk[edge_fv.id].radial.id);

  // set the correct fvert ordering
  fvertBlk[follower0.id].next = lead1;
  fvertBlk[follower1.id].next = lead0;

  // repair face fv
  faceBlk[adjacent1.id].fv = lead1;

  // assign the unique face to all its fverts
  first = lead1;
  cur = first;
  do {
    fvertBlk[cur.id].f = adjacent1;
    // printf("ew\n");
  } while ((cur = fvertBlk[cur.id].next) != first);

  //
}

/////////////////// FACE  /////////////////// ///////////////////
////////////////////// /////////////////// ///////////////////
////////////////////// /////////////////// ///////////////////
////////////////////// /////////////////// ///////////////////
//////////////////////

/// @brief connects points across a Face topologically, resulting in a new Face.
/// think of it as a coded multi-cut tool with you putting points on the Face
/// before finishing on some Vert
/// @tparam VertBlk
/// @tparam FaceBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param num_floating number of (floating) Vert 's to be added on top of the
/// Face
/// @param pre_beginfv
/// @param pre_endfv
/// @param faceBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk

template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
void make_face_on_top(const int num_floating, const FvertHdl pre_beginfv,
                      const FvertHdl pre_endfv, FaceBlk &faceBlk,
                      FvertBlk &fvertBlk, EdgeBlk &edgeBlk, VertBlk &vertBlk) {

  /*

  1)
<<<=================== =|
|                       |
vpre                    ^end
|                       |
vbeg  floating * vtx    ^pre
|                       |
|===================>>> |

2)

<<<=================== =|
|                       |
vpre                    ^end
|                       |
vbeg---e1---*vtx---e0---^pre
|                       |
|===================>>> |

*/

  FaceHdl facehdl = fvertBlk[pre_beginfv.id].f;

  constexpr size_t expected_capac = 8;

  llvm_vecsmall::SmallVector<Fvert, expected_capac * 2>
      new_fverts; // fvert and counterpart (radial)
  llvm_vecsmall::SmallVector<Edge, expected_capac> new_edges;

  new_fverts.resize((num_floating + 1) * 2);
  new_edges.resize(num_floating + 1);

  FvertHdl &pre_beginfv_next = fvertBlk[pre_beginfv.id].next;
  FvertHdl beginfv = pre_beginfv_next;

  FvertHdl &pre_endfv_next = fvertBlk[pre_endfv.id].next;
  FvertHdl endfv = pre_endfv_next;

  const int newvtx_id = (int)vertBlk.size();
  const int newfv_id = (int)fvertBlk.size();
  const int newe_id = (int)edgeBlk.size();
  const int newf_id = (int)faceBlk.size();

  VertHdl vert_alpha = get_v(edgeBlk, fvertBlk, beginfv);
  VertHdl vert_beta = get_v(edgeBlk, fvertBlk, endfv);

  VertHdl preced_vert = vert_beta;

  const int last_fv_index = new_fverts.size() - 1;
  const int last_edge_idx = new_edges.size() - 1;

  for (int i = 0; i < new_edges.size() - 1; ++i) {

    FvEdgeHdl fvert_edge_hdl = FVEHDL(newe_id + i, !, );

    FvEdgeHdl fvert_edge_hdl_flip = FVEHDL(newe_id + i, , );

    new_fverts[i].next.id = newfv_id + i + 1;

    new_fverts[i].radial.id = newfv_id + num_floating + 1 + i;

    new_fverts[i].f = facehdl;
    new_fverts[i].e = fvert_edge_hdl;

    //
    new_fverts[num_floating + 1 + i].next.id = newfv_id + num_floating + i;

    new_fverts[num_floating + 1 + i].radial.id = newfv_id + i;
    new_fverts[num_floating + 1 + i].f.id = newf_id;
    new_fverts[num_floating + 1 + i].e = fvert_edge_hdl_flip;

    vertBlk.pushBack(Vert{FvertHdl(newfv_id + i)});
  }

  new_fverts[num_floating + 1 + 0].next = endfv;
  pre_beginfv_next.id = newfv_id + last_fv_index;

  new_fverts[num_floating].next = beginfv;
  pre_endfv_next.id = newfv_id;

  {

    FvEdgeHdl fvert_edge_hdl = FVEHDL(newe_id + num_floating, !, );
    FvEdgeHdl fvert_edge_hdl_flip = FVEHDL(newe_id + num_floating, , );

    EdgeHdl beginfv_edge = fvertBlk[beginfv.id].e;

    new_fverts[num_floating].next = beginfv;
    new_fverts[num_floating].radial.id = newfv_id + last_fv_index;
    new_fverts[num_floating].f = facehdl;
    new_fverts[num_floating].e = fvert_edge_hdl;

    new_fverts[last_fv_index].next.id = newfv_id + (last_fv_index - 1);
    new_fverts[last_fv_index].radial.id = newfv_id + num_floating;
    new_fverts[last_fv_index].f.id = newf_id;
    new_fverts[last_fv_index].e = fvert_edge_hdl_flip;
    //
  }

  for (int i = 1; i < new_edges.size(); i++) {
    new_edges[i].v0.id = newvtx_id + (i - 1);
    new_edges[i].v1.id = newvtx_id + i;
    new_edges[i].next0.id = newe_id + (i - 1);
    new_edges[i - 1].next1.id = newe_id + i;
  }

  new_edges.back().v1 = vert_alpha;
  new_edges.front().v0 = vert_beta;
  new_edges.front().v1 = new_edges[1].v0;

  for (auto cur_fv : new_fverts) {
    fvertBlk.pushBack(cur_fv);
  }
  for (auto cur_edge : new_edges) {
    edgeBlk.pushBack(cur_edge);
  }

  construct_add_to_edge_cycle(vert_alpha, EdgeHdl{newe_id + last_edge_idx},
                              faceBlk, fvertBlk, edgeBlk, vertBlk);
  construct_add_to_edge_cycle(vert_beta, EdgeHdl{newe_id + 0}, faceBlk,
                              fvertBlk, edgeBlk, vertBlk);

  faceBlk.pushBack(Face{FvertHdl(newfv_id + last_fv_index)});

  faceBlk[facehdl.id].fv = FvertHdl(newfv_id);

  {
    FvertHdl first = faceBlk[newf_id].fv;
    FvertHdl cur = first;

    do {
      fvertBlk[cur.id].f = FaceHdl{newf_id};
    } while ((cur = fvertBlk[cur.id].next) != first);

    first = faceBlk[facehdl.id].fv;
    cur = first;

    do {
      fvertBlk[cur.id].f = facehdl;
    } while ((cur = fvertBlk[cur.id].next) != first);
  }
}


/// @brief deletes Face 's
/// @tparam VertBlk
/// @tparam FaceBlk
/// @tparam FvertBlk
/// @tparam EdgeBlk
/// @param deadface
/// @param deadfvert
/// @param deadedge
/// @param deadvert
/// @param facehdls Face 's to delete
/// @param len_faces
/// @param faceBlk
/// @param fvertBlk
/// @param edgeBlk
/// @param vertBlk
template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
void delete_faces(BitArray &deadface /* bit array indicating deleted faces */,
                  BitArray &deadfvert /* bit array indicating deleted fverts */,
                  BitArray &deadedge /* bit array indicating deleted edges */,
                  BitArray &deadvert /* bit array indicating deleted verts */,
                  const FaceHdl *facehdls, int len_faces, FaceBlk &faceBlk,
                  FvertBlk &fvertBlk, EdgeBlk &edgeBlk, VertBlk &vertBlk) {

  for (size_t i = 0; i < len_faces; i++) {
    delete_face(deadface, deadfvert, deadedge, deadvert, facehdls[i], faceBlk,
                fvertBlk, edgeBlk, vertBlk);
  }
}
// note: these bit arrays are full of ones
template <class VertBlk, class FaceBlk, class FvertBlk, class EdgeBlk>
void delete_face(BitArray &deadface /* bit array indicating deleted faces */,
                 BitArray &deadfvert /* bit array indicating deleted fverts */,
                 BitArray &deadedge /* bit array indicating deleted edges */,
                 BitArray &deadvert /* bit array indicating deleted verts */,
                 FaceHdl facehdl, FaceBlk &faceBlk, FvertBlk &fvertBlk,
                 EdgeBlk &edgeBlk, VertBlk &vertBlk) {

  deadface.turnBitOff(facehdl.id);

  FvertHdl firstfv = faceBlk[facehdl.id].fv;
  FvertHdl cur = firstfv;
  FvertHdl curnext;

  int deledge = 0;
  do {
    curnext = fvertBlk[cur.id].next;
    VertHdl cur_verthdl = get_v(edgeBlk, fvertBlk, cur);
    VertHdl next_verthdl = get_v(edgeBlk, fvertBlk, curnext);

    FvertHdl radial = fvertBlk[cur.id].radial;

    vertBlk[cur_verthdl.id].fv = radial;

    EdgeHdl edge = fvertBlk[cur.id].e;

    deadfvert.turnBitOff(cur.id);

    if (!(deledge + vert_has_more_than_two_edges(edgeBlk, cur_verthdl, edge))) {
      deadvert.turnBitOff(next_verthdl.id);
    }
    deledge = int(radial == cur);

    if (deledge) {

      deadedge.turnBitOff(edge.id);
      edgePopOff(edge, cur_verthdl, faceBlk, fvertBlk, edgeBlk, vertBlk);
      edgePopOff(edge, next_verthdl, faceBlk, fvertBlk, edgeBlk, vertBlk);

    } else {

      radialPopOff(radial, cur, fvertBlk);
    }

  } while ((cur = curnext) != firstfv);

  //
}



/////////////////// EDGE CYCLE /////////////////// ///////////////////
////////////////////// /////////////////// ///////////////////
////////////////////// /////////////////// ///////////////////
////////////////////// /////////////////// ///////////////////
//////////////////////

/*
 * \brief  gets rid of the edge in the cycle of the vert
 */
template <class VertBlk, class EdgeBlk, class FaceBlk>
void edgePopOff(EdgeHdl edgehdl, VertHdl ofvert, FaceBlk &faceBlk,
                EdgeBlk &edgeBlk, VertBlk &vertBlk) {

  EdgeHdl first = edgehdl;
  EdgeHdl cur = first;
  EdgeHdl preceding;

  EdgeHdl first_following = *get_next_edge(ofvert, cur, edgeBlk);

  do {

    preceding = cur;

  } while ((cur = *get_next_edge(ofvert, cur, edgeBlk)) != first);

  *get_next_edge(ofvert, preceding, edgeBlk) = first_following;
}

/////////////////// RADIAL CYCLE  /////////////////// ///////////////////
////////////////////// /////////////////// ///////////////////
////////////////////// /////////////////// ///////////////////
////////////////////// /////////////////// ///////////////////
//////////////////////

/*
 * \brief gets rid of the face/radial in the cyclce of the edge
 */
template <class FvertBlk>
void radialPopOff(FvertHdl fromCycle, FvertHdl toPop, FvertBlk &fvertBlk) {

  FvertHdl first = fromCycle;
  FvertHdl cur = first;
  FvertHdl curnext;

  do {
    curnext = fvertBlk[cur.id].radial;
    if (curnext == toPop) {
      FvertHdl curnext_next = fvertBlk[curnext.id].radial;
      fvertBlk[cur.id].radial = curnext_next;
      break;
    }
    cur = curnext;
  } while (cur != first);
}
