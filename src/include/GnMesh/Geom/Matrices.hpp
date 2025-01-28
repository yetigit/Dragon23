#pragma once

#include <GnMesh/Geom/GnmEigen.PCH.hpp>


namespace {

    // row major vector and matrix
void vector3_matrix3_rm_mult(float *v, float *m, float *delta) {
  // this is exactly a dot product by each column of the matrix
  delta[0] = v[0] * m[0] + v[1] * m[3] + v[2] * m[6];
  delta[1] = v[0] * m[1] + v[1] * m[4] + v[2] * m[7];
  delta[2] = v[0] * m[2] + v[1] * m[5] + v[2] * m[8];
}


}

template <class S>
typename Gnm_matrixType<S>::Mat33_t
matrix_from_basis(const typename Gnm_matrixType<S>::Vector_t &x,
                  const typename Gnm_matrixType<S>::Vector_t &y,
                  const typename Gnm_matrixType<S>::Vector_t &z) {

  typename Gnm_matrixType<S>::Mat33_t res = Gnm_matrixType<S>::Mat33_t::Identity();

  res.template block<1, 3>(0, 0) = x;
  res.template block<1, 3>(1, 0) = y;
  res.template block<1, 3>(2, 0) = z;

  return res;
}

// orientation must be normalized
template <class S>
typename Gnm_matrixType<S>::Mat33_t
do_orient_matrix3(const typename Gnm_matrixType<S>::Vector_t &orientation) {

  typename Gnm_matrixType<S>::Mat33_t res =
      typename Gnm_matrixType<S>::Mat33_t::Identity();

  const typename Gnm_matrixType<S>::Vector_t up = Gnm_matrixType<S>::Vector_t::UnitY();

  typename Gnm_matrixType<S>::Vector_t zvec =
      up - (orientation * orientation.dot(up));
  zvec.normalize();

  typename Gnm_matrixType<S>::Vector_t xvec = orientation.cross(zvec);
  xvec.normalize();

  res.block<1, 3>(0, 0) = xvec;
  res.block<1, 3>(1, 0) = orientation;
  res.block<1, 3>(2, 0) = zvec;

  return res;
}
