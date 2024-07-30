#include "line_search.h"
#include <iostream>

double line_search(
  const std::function<double(const Eigen::VectorXd &)> & f,
  const std::function<void(Eigen::VectorXd &)> & proj_z,
  const Eigen::VectorXd & z,
  const Eigen::VectorXd & dz,
  const double max_step)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  double alpha = max_step;
  Eigen::VectorXd delta_z = z - max_step * dz;
  proj_z(delta_z);

  double f0 = f(z);
  while (f(delta_z) > f0){
    delta_z = z - alpha * dz;
    proj_z(delta_z);
    if (f(delta_z) == f0){
      return 0.0;
    }
    alpha = alpha * 0.5;
  }
  return alpha;
  /////////////////////////////////////////////////////////////////////////////
}
