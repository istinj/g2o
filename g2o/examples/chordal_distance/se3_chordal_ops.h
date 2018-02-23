#pragma once

#include "g2o/core/eigen_types.h"

namespace g2o {
  namespace chordal {

    typedef Eigen::Matrix<double, 6, 6>     Matrix6;
    typedef Eigen::Matrix<double, 12, 12>   Matrix12;

    typedef Eigen::Matrix<double, 6, 1>     Vector6;
    typedef Eigen::Matrix<double, 7, 1>     Vector7;
    typedef Eigen::Matrix<double, 9, 1>     Vector9;
    typedef Eigen::Matrix<double, 12, 1>    Vector12;

    inline Matrix3 skew(const Vector3& p) {
      Matrix3 s = Matrix3::Zero();
      s <<    0,       -p.z(),       p.y(),
          p.z(),            0,      -p.x(),
         -p.y(),        p.x(),           0;
      return s;
    }

    inline Isometry3 v2tQUAT(const Vector7& v) {
      Isometry3 T = Isometry3::Identity();
      Eigen::Quaternion<double> q;
      q.x() = v[3];
      q.y() = v[4];
      q.z() = v[5];
      q.w() = v[6];

      T.linear() = q.matrix();
      T.translation() = v.block<3,1>(0,0);
      return T;
    }

    inline Vector7 t2vQUAT(const Isometry3& t) {
      Vector7 v;
      v.head<3>()=t.translation();
      Eigen::Quaternion<double> q(t.linear());
      v[3] = q.x();
      v[4] = q.y();
      v[5] = q.z();
      v[6] = q.w();

      return v;
    }

    inline Isometry3 v2t(const Vector6& v) {
      Isometry3 T = Isometry3::Identity();
      Matrix3 Rx, Ry, Rz;
      Rx = Eigen::AngleAxis<double>(v(3), Vector3::UnitX());
      Ry = Eigen::AngleAxis<double>(v(4), Vector3::UnitY());
      Rz = Eigen::AngleAxis<double>(v(5), Vector3::UnitZ());
      T.linear() = Rx * Ry * Rz;
      T.translation() = v.block<3,1>(0,0);
      return T;
    }

    inline Vector6 t2v(const Isometry3& t) {
      Vector6 v;
      v.head<3>()=t.translation();

      Matrix3 r = t.linear();
      Vector3 euler_angles = r.eulerAngles(0,1,2);
      v.block<3,1>(3,0) = euler_angles;

      return v;
    }

    inline Vector12 flattenIsometry(const Isometry3& T_) {
      Vector12 v;
      v.block<3,1>(0,0) = T_.matrix().block<3,1>(0,0);
      v.block<3,1>(3,0) = T_.matrix().block<3,1>(0,1);
      v.block<3,1>(6,0) = T_.matrix().block<3,1>(0,2);
      v.block<3,1>(9,0) = T_.matrix().block<3,1>(0,3);
      return v;
    }

    inline Isometry3 unflattenIsometry(const Vector12& vector_, const bool reconditionate_rotation_) {
      Isometry3 T = Isometry3::Identity();
      T.matrix().block<3,1>(0,0) = vector_.block<3,1>(0,0);
      T.matrix().block<3,1>(0,1) = vector_.block<3,1>(3,0);
      T.matrix().block<3,1>(0,2) = vector_.block<3,1>(6,0);
      T.matrix().block<3,1>(0,3) = vector_.block<3,1>(9,0);

      if (reconditionate_rotation_) {
        Matrix3 R = T.linear();
        Eigen::JacobiSVD<Matrix3> svd(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Matrix3 R_enforced = svd.matrixU() * svd.matrixV().transpose();
        T.linear() = R_enforced;
      }

      return T;
    }

    inline Matrix3 reconditionateRotationMatrix(const Matrix3& R) {
      Eigen::JacobiSVD<Matrix3> svd(R, Eigen::ComputeThinU | Eigen::ComputeThinV);
      Matrix3 R_enforced = svd.matrixU() * svd.matrixV().transpose();
      return R_enforced;
    }

  } //ia end namespace chordal
} //ia end namespace g2o