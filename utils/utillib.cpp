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
    poses.emplace_back(Quaterniond(qw, qx, qy, qz), Vector3d(tx, ty, tz));
  }
  return poses;
}