#include "kinematics_jacobian.h"
#include "transformed_tips.h"
#include <iostream>

void kinematics_jacobian(
  const Skeleton & skeleton,
  const Eigen::VectorXi & b,
  Eigen::MatrixXd & J)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  J = Eigen::MatrixXd::Zero(b.size()*3,skeleton.size()*3);
  double ep = 1e-7;
  Eigen::VectorXd pos = transformed_tips(skeleton, b);

  for (int i = 0; i < skeleton.size(); i++) {
    for (int j = 0; j < 3; j++) {
      Skeleton copy = skeleton;
      copy[i].xzx(j) += ep;
      Eigen::VectorXd trans_pos = transformed_tips(copy, b) - pos;
      for (int k = 0; k < b.size() * 3; k++) {
        J(k, i*3 + j) = trans_pos[k] / ep;
      }
    }
  }
  /////////////////////////////////////////////////////////////////////////////
}
