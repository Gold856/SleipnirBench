// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include "frc/EigenCore.h"

namespace frc {

/**
 * Returns true if (A, B) is a stabilizable pair.
 *
 * (A, B) is stabilizable if and only if the uncontrollable eigenvalues of A, if
 * any, have absolute values less than one, where an eigenvalue is
 * uncontrollable if rank([λI - A, B]) < n where n is the number of states.
 *
 * @tparam States Number of states.
 * @tparam Inputs Number of inputs.
 * @param A System matrix.
 * @param B Input matrix.
 */
template <int States, int Inputs>
bool IsStabilizable(const Matrixd<States, States>& A,
                    const Matrixd<States, Inputs>& B) {
  Eigen::EigenSolver<Matrixd<States, States>> es{A, false};

  for (int i = 0; i < A.rows(); ++i) {
    if (std::norm(es.eigenvalues()[i]) < 1) {
      continue;
    }

    if constexpr (States != Eigen::Dynamic && Inputs != Eigen::Dynamic) {
      Eigen::Matrix<std::complex<double>, States, States + Inputs> E;
      E << es.eigenvalues()[i] * Eigen::Matrix<std::complex<double>, States,
                                               States>::Identity() -
               A,
          B;

      Eigen::ColPivHouseholderQR<
          Eigen::Matrix<std::complex<double>, States, States + Inputs>>
          qr{E};
      if (qr.rank() < States) {
        return false;
      }
    } else {
      Eigen::MatrixXcd E{A.rows(), A.rows() + B.cols()};
      E << es.eigenvalues()[i] *
                   Eigen::MatrixXcd::Identity(A.rows(), A.rows()) -
               A,
          B;

      Eigen::ColPivHouseholderQR<Eigen::MatrixXcd> qr{E};
      if (qr.rank() < A.rows()) {
        return false;
      }
    }
  }
  return true;
}

/**
 * Returns true if (A, C) is a detectable pair.
 *
 * (A, C) is detectable if and only if the unobservable eigenvalues of A, if
 * any, have absolute values less than one, where an eigenvalue is unobservable
 * if rank([λI - A; C]) < n where n is the number of states.
 *
 * @tparam States Number of states.
 * @tparam Outputs Number of outputs.
 * @param A System matrix.
 * @param C Output matrix.
 */
template <int States, int Outputs>
bool IsDetectable(const Matrixd<States, States>& A,
                  const Matrixd<Outputs, States>& C) {
  return IsStabilizable<States, Outputs>(A.transpose(), C.transpose());
}

}  // namespace frc
