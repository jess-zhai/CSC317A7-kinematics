#include "transformed_tips.h"
#include "forward_kinematics.h"

Eigen::VectorXd transformed_tips(
  const Skeleton & skeleton, 
  const Eigen::VectorXi & b)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  std::vector<Eigen::Affine3d,Eigen::aligned_allocator<Eigen::Affine3d>> T;
  forward_kinematics(skeleton, T);
  Eigen::VectorXd pos = Eigen::VectorXd::Zero(3*b.size());
  for (int i = 0; i < b.size(); i++) {
    Eigen::Vector4d tip = T[b[i]] * skeleton[b[i]].rest_T * Eigen::Vector4d(skeleton[b[i]].length, 0, 0, 1);
    pos[i*3] = tip[0];
    pos[i*3+1] = tip[1];
    pos[i*3+2] = tip[2];
  }

  return pos;
  /////////////////////////////////////////////////////////////////////////////
}
