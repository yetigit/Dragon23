.. Dragon documentation master file, created by
   sphinx-quickstart on Mon Jan 31 13:30:10 2022.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Welcome to Dragon's documentation
==================================

.. toctree::
   :maxdepth: 2
   :caption: Contents:



Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`


This documentation will focus on two mesh types: Index Mesh and Brep Mesh.

Boundary Representation Meshes  (Brep Mesh)
===========================================

A Brep Mesh is basically a more elaborate half-edge mesh data structure, allowing for support of non-manifolds and better traversal. 
Now in Dragon it's made even better by the usage of block arrays and a smarter structure with less class members and smalle byte size objects.


.. doxygenfunction:: structure_from_indices


Brep Mesh Handles
=================

.. doxygenstruct:: BaseHdl
   :members:
.. doxygenstruct:: EdgeHdl
.. doxygenstruct:: FaceHdl
.. doxygenstruct:: VertHdl
.. doxygenstruct:: FvertHdl
.. doxygenstruct:: FvEdgeHdl
   :members:
.. doxygenstruct:: FvertHdlPair
   :members:
.. doxygenstruct:: VertHdlPair
   :members:
.. doxygenstruct:: FaceHdlPair
   :members:
.. doxygenstruct:: FvertHdlTriple
   :members:
.. doxygenstruct:: VertHdlTriple
   :members:

Brep Mesh Items
===============

.. doxygenstruct:: Edge
   :members:
.. doxygenstruct:: Vert
   :members:
.. doxygenstruct:: Face
   :members:
.. doxygenstruct:: Fvert
   :members:

Brep Mesh Stacks
================

.. doxygenfunction:: self_plus_two
.. doxygenfunction:: get_tri_v
.. doxygenfunction:: fv_loop_stack
.. doxygenfunction:: fv_radial_stack
.. doxygenfunction:: faceverts_stack
.. doxygenfunction:: facefv_stack(FaceBlk const &faceBlk, const FvertBlk &fvertBlk, const FaceHdl fhdl, GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg = GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{}) -> llvm_vecsmall::SmallVector<FvertHdl, vcount>
.. doxygenfunction:: face_edges_stack
.. doxygenfunction:: vert_edge_ring
.. doxygenfunction:: vert_edge_ring_unordered(const VertBlk &vertBlk, const FvertBlk &fvertBlk, const EdgeBlk &edgeBlk, const VertHdl vhdl, GNM_NAMESPACE::meta::WrapTValue<size_t, vcount> vcountarg = GNM_NAMESPACE::meta::WrapTValue<size_t, vcount>{}) -> llvm_vecsmall::SmallVector<EdgeHdl, vcount>
.. doxygenfunction:: vert_vert_ring
.. doxygenfunction:: vert_outgoing_fverts
.. doxygenfunction:: vert_face_ring

Brep Mesh Other Queries
=======================

.. doxygenfunction:: face_count
.. doxygenfunction:: get_v
.. doxygenfunction:: getprev
.. doxygenfunction:: edge_common_face
.. doxygenfunction:: is_border_fv
.. doxygenfunction:: is_boundary_vert
.. doxygenfunction:: is_boundary_edge
.. doxygenfunction:: edge_adjacent_faces2
.. doxygenfunction:: get_edge_fvert
.. doxygenfunction:: is_not_trimesh

Brep Mesh Geometric Queries
===========================

.. doxygenfunction:: dihedral_angle_filter
.. doxygenfunction:: smooth_by_edge_tags
.. doxygenfunction:: get_all_edge_lengths
.. doxygenfunction:: vertexSmoothNormal_all
.. doxygenfunction:: is_face_planar
.. doxygenfunction:: is_face_convex
.. doxygenfunction:: get_all_face_normals
.. doxygenfunction:: get_face_area
.. doxygenfunction:: laplacian

Editing a Brep Mesh
===================

.. doxygenfunction:: edge_edge_connect
.. doxygenfunction:: vert_edge_connect
.. doxygenfunction:: edge_collapse
.. doxygenfunction:: vert_vert_connect
.. doxygenfunction:: delete_edge_superficial
.. doxygenfunction:: edge_insert_vert
.. doxygenfunction:: make_face_on_top
.. doxygenfunction:: delete_faces
.. doxygenfunction:: fan_triangulate
.. doxygenfunction:: triangulate_concave_and_nonplanar


Index Mesh
==========

.. doxygenclass:: IndexMesh
   :members:

.. doxygenstruct:: IMPropertyFlags
   :members:
.. doxygenstruct:: IndexMeshFlags
   :members:
.. doxygenstruct:: IMUVData
   :members:
.. doxygenstruct:: IMWireframe
   :members:
.. doxygenstruct:: IMCachedNormals
   :members:
.. doxygenstruct:: IMPointSplit
   :members: