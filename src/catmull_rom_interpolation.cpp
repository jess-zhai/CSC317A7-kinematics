#include "catmull_rom_interpolation.h"
#include <Eigen/Dense>

Eigen::Vector3d catmull_rom_interpolation(
  const std::vector<std::pair<double, Eigen::Vector3d> > & keyframes,
  double t)
{
  /////////////////////////////////////////////////////////////////////////////
  // Replace with your code
  if (keyframes.empty()) {
    return Eigen::Vector3d(0,0,0);
  }
  double time = std::fmod(t, keyframes.back().first);

  int i;
  for (i = 0; i < keyframes.size(); i++) {
    if (keyframes[i].first > time && (time < keyframes[i + 1].first)) {
      break;
    } else if (keyframes[i].first == time) {
      return keyframes[i].second;
    }
  }

  double t0, t1, t2, t3;
  if (i == 0){
    t0 = keyframes[i].first;
  } else{
    t0 = keyframes[i - 1].first;
  }
  t1 = keyframes[i].first;
  t2 = keyframes[i + 1].first;
  t3 = keyframes[i + 2].first;

  Eigen::Vector3d p0, p1, p2, p3;
  p0 = keyframes[i - 2].second;
  p1 = keyframes[i - 1].second;
  p2 = keyframes[i].second;
  p3 = keyframes[i + 1].second;

  Eigen::Vector3d tan0, tan1;
  tan0 = (p2-p0) / (t2-t0);
  tan1 = (p3-p1) / (t3-t1);

  double d = (time - t1) / (t2-t1);
  Eigen::Vector3d ret = (2 * pow(d, 3) - 3* pow(d, 2) + 1) * p1 + (pow(d, 3) - 2 * pow(d, 2) + d) * tan0 + 
        (-2 * pow(d, 3) + 3 * pow(d, 2)) * p2 + (pow(d, 3) - pow(d, 2))  * tan1;

  return ret;
  /////////////////////////////////////////////////////////////////////////////
}
