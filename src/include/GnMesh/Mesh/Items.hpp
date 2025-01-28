#pragma once

#include <GnMesh/Mesh/Handles.hpp>

/**
\brief Edge Item 
\note v0 and v1 has to exist independently of radials
 because duplicate can occur in case of non manifolds
 we can find the Fvert of an Edge by circulating v0 and v1 around and find an
 incident common Edge
*/
struct Edge {

    /// @brief one of the Vert of this Edge
    VertHdl v0;

    /// @brief one of the Vert of this Edge
    VertHdl v1;

    /// \brief in the cycle of v0 for this Edge, next0 is the next Edge 
    EdgeHdl next0;

    /// \brief in the cycle of v1 for this Edge, next1 is the next Edge 
    EdgeHdl next1;


    FvertHdl fv;


    bool operator==( Edge const & other) const { 
     return v0 == other.v0 
         && v1 == other.v1 && 
         next0 == other.next0 && next1 == other.next1 && fv == other.fv 
         ; 
    }

};


/**
\brief Vert Item 
*/
struct Vert {

    /// \brief the .fv can point to this Vert (then one of the radial of .fv has its .v equal to this Vert) 
    ///  or be "outgoing" relative to this Vert
    ///  (then Vert.fv.v == Vert)
  /*
  
    vtx-----out fv----->>
    ^   
    |         OR
    | in fv
    |
    |
    |
    
     
  */

    FvertHdl fv; 

};



/**
\brief Face Item 
*/
struct Face {

    /// @brief this fv marks the beginning of the boundary of this Face.
    /// To go around a Face all that is required is fv.next 
    FvertHdl fv;

};


/**
\brief Fvert Item 
*/
struct Fvert {
    
    /// @brief this **encapsulate** the Edge hdl and VertHdl at this Fvert
    /// \note this handle like any item handle is 4 bytes
    FvEdgeHdl e;

    /// @brief Face bounded by this Fvert
    FaceHdl f;

    /// @brief this is a handle to the next Fvert describing the boundary of its Face 
    FvertHdl next;
    /// @brief this is a handle to a adjacent Face boundary sharing the Edge of this Fvert
    FvertHdl radial;
};
//
//static_assert((sizeof(Fvert) == 16), "Fvert class unexpected size");
//static_assert((sizeof(Face) == 4), "Face class unexpected size");
//static_assert((sizeof(Vert) == 4), "Vert class unexpected size");
//static_assert((sizeof(Edge) == 20), "Edge class unexpected size");
