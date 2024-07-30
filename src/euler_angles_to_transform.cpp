#include "euler_angles_to_transform.h"

Eigen::Affine3d euler_angles_to_transform(
  const Eigen::Vector3d & xzx)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  double angle1 = xzx[0] * M_PI/180.0;
  double angle2 = xzx[1] * M_PI/180.0;
  double angle3 = xzx[2] * M_PI/180.0;
  Eigen::AngleAxisd rotation_x1(angle1, Eigen::Vector3d::UnitX());
  Eigen::AngleAxisd rotation_z(angle2, Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd rotation_x2(angle3, Eigen::Vector3d::UnitX());
  
  Eigen::Affine3d A(rotation_x1 * rotation_z * rotation_x2);
  return A;
  /////////////////////////////////////////////////////////////////////////////
}
