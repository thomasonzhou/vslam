#include "utils/utillib.h"
#include <iostream>
#include <fstream>

using Eigen::Quaterniond;
using Eigen::Vector3d;

poseVector trajectory_from_file(const std::string& trajectory_file){

  poseVector poses;
  std::ifstream fin(trajectory_file);
  if (!fin) {
        std::cout << "file not found: " << trajectory_file << std::endl;
        return poses;
  }
  
  double time, tx, ty, tz, qx, qy, qz, qw;
  while (fin >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
      Isometry3d Twr(Quaterniond(qw, qx, qy, qz));
    Twr.pretranslate(Vector3d(tx, ty, tz));
    poses.push_back(Twr);
  }
  return poses;
}