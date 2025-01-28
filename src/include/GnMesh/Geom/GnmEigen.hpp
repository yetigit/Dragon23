#pragma once

#pragma warning(push)
#pragma warning(disable : 4127)
#include <Eigen/Dense>
#pragma warning(pop)

//
#define GNM_EIGEN_STORAGE_ORDER Eigen::RowMajor

template <typename ScalarT> struct Gnm_matrixType {
  using Mat33_t = Eigen::Matrix<ScalarT, 3, 3, GNM_EIGEN_STORAGE_ORDER>;
  using Mat44_t = Eigen::Matrix<ScalarT, 4, 4, GNM_EIGEN_STORAGE_ORDER>;
  using Vector_t = Eigen::Matrix<ScalarT, 1, 3, GNM_EIGEN_STORAGE_ORDER>;
  using Vector4_t = Eigen::Matrix<ScalarT, 1, 4, GNM_EIGEN_STORAGE_ORDER>;
  using Vector2_t = Eigen::Matrix<ScalarT, 1, 2, GNM_EIGEN_STORAGE_ORDER>;

  using Vector2f_t = Eigen::Matrix<float, 1, 2, GNM_EIGEN_STORAGE_ORDER>;
  using Vector4f_t = Eigen::Matrix<float, 1, 4, GNM_EIGEN_STORAGE_ORDER>;
};

namespace Eigen {

// DEPRECATED  just for bakwrd compat
using Mat33_t = Eigen::Matrix<double, 3, 3, GNM_EIGEN_STORAGE_ORDER>;
using Mat44_t = Eigen::Matrix<double, 4, 4, GNM_EIGEN_STORAGE_ORDER>;
using Vector_t = Eigen::Matrix<double, 1, 3, GNM_EIGEN_STORAGE_ORDER>;
using Vector4_t = Eigen::Matrix<double, 1, 4, GNM_EIGEN_STORAGE_ORDER>;
using Vector2_t = Eigen::Matrix<double, 1, 2, GNM_EIGEN_STORAGE_ORDER>;

using Vector2f_t = Eigen::Matrix<float, 1, 2, GNM_EIGEN_STORAGE_ORDER>;
using Vector4f_t = Eigen::Matrix<float, 1, 4, GNM_EIGEN_STORAGE_ORDER>;

} // namespace Eigen
namespace {

template <class VecTy> auto AngleBetween(const VecTy &a, const VecTy &b) {

  VecTy::Scalar ret = a.dot(b) / (a.norm() * b.norm());
  ret = std::clamp(ret, (VecTy::Scalar)-1.0, (VecTy::Scalar)1.0);
  ret = std::acos(ret);

  return ret;
}
template <class VecTy> auto Cotangent(const VecTy &a, const VecTy &b) {

  VecTy::Scalar ret = a.dot(b) / (1e-5 + a.cross(b).norm());

  return ret;
}

Eigen::Vector3f Matrix_getRotation_(Gnm_matrixType<float>::Mat44_t &self) {
  Eigen::Matrix<float, 3, 3, Eigen::RowMajor> rot = self.block<3, 3>(0, 0);
  Eigen::Vector3f current_scale = {rot.block<1, 3>(0, 0).norm(),
                                   rot.block<1, 3>(1, 0).norm(),
                                   rot.block<1, 3>(2, 0).norm()};
  // preserve scale only for those rows that actually need it
  if (current_scale[0] != 0.f) {
    rot.block<1, 3>(0, 0) /= current_scale[0];
  }
  if (current_scale[1] != 0.f) {
    rot.block<1, 3>(1, 0) /= current_scale[1];
  }
  if (current_scale[2] != 0.f) {
    rot.block<1, 3>(2, 0) /= current_scale[2];
  }

  Eigen::Vector3f res = rot.block<3, 3>(0, 0).eulerAngles(2, 1, 0);

  Eigen::Vector3f new_val;
  new_val = {res[2], res[1], res[0]};
  return new_val;
}

void Matrix_setRotation2_(Gnm_matrixType<float>::Mat44_t &self,
                          Eigen::Vector3f vec, const float coef) {

  Eigen::Vector3f x_axis(1.f * coef, 0.f, 0.f);
  Eigen::Vector3f y_axis(0.f, 1.f * coef, 0.f);
  Eigen::Vector3f z_axis(0.f, 0.f, 1.f);
  Eigen::Affine3f rx = Eigen::Affine3f(Eigen::AngleAxisf(vec[0], x_axis));
  Eigen::Affine3f ry = Eigen::Affine3f(Eigen::AngleAxisf(vec[1], y_axis));
  Eigen::Affine3f rz = Eigen::Affine3f(Eigen::AngleAxisf(vec[2], z_axis));
  Eigen::Affine3f data = rz * ry * rx;
  Eigen::Vector3f current_scale = {self.block<1, 3>(0, 0).norm(),
                                   self.block<1, 3>(1, 0).norm(),
                                   self.block<1, 3>(2, 0).norm()};
  self.block<3, 3>(0, 0) = data.rotation().transpose();
  // self.block<3, 3>(0, 0) = data.rotation();
  //  preserve scale only for those rows that actually need it
  if (current_scale[0] != 0.f) {
    self.block<1, 3>(0, 0) /= current_scale[0];
  }
  if (current_scale[1] != 0.f) {
    self.block<1, 3>(1, 0) /= current_scale[1];
  }
  if (current_scale[2] != 0.f) {
    self.block<1, 3>(2, 0) /= current_scale[2];
  }
}

void Matrix_setRotation_(Gnm_matrixType<float>::Mat44_t &self,
                         Eigen::Vector3f vec) {

  Eigen::Vector3f x_axis(1.f, 0.f, 0.f);
  Eigen::Vector3f y_axis(0.f, 1.f, 0.f);
  Eigen::Vector3f z_axis(0.f, 0.f, 1.f);
  Eigen::Affine3f rx = Eigen::Affine3f(Eigen::AngleAxisf(vec[0], x_axis));
  Eigen::Affine3f ry = Eigen::Affine3f(Eigen::AngleAxisf(vec[1], y_axis));
  Eigen::Affine3f rz = Eigen::Affine3f(Eigen::AngleAxisf(vec[2], z_axis));
  Eigen::Affine3f data = rz * ry * rx;
  Eigen::Vector3f current_scale = {self.block<1, 3>(0, 0).norm(),
                                   self.block<1, 3>(1, 0).norm(),
                                   self.block<1, 3>(2, 0).norm()};
  self.block<3, 3>(0, 0) = data.rotation();
  // self.block<3, 3>(0, 0) = data.rotation();
  //  preserve scale only for those rows that actually need it
  if (current_scale[0] != 0.f) {
    self.block<1, 3>(0, 0) /= current_scale[0];
  }
  if (current_scale[1] != 0.f) {
    self.block<1, 3>(1, 0) /= current_scale[1];
  }
  if (current_scale[2] != 0.f) {
    self.block<1, 3>(2, 0) /= current_scale[2];
  }
}
} // namespace
