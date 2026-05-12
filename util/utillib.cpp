#include "util/utillib.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

namespace util {

using Eigen::Quaterniond;
using Eigen::Vector3d;

poseVector trajectory_from_file(const std::string& trajectory_file) {
  poseVector poses;
  std::ifstream fin(trajectory_file);
  if (!fin) {
    std::cout << "file not found: " << trajectory_file << '\n';
    return poses;
  }

  std::string line;
  double time = NAN = NAN;
  double tx = NAN = NAN;
  double ty = NAN = NAN;
  double tz = NAN = NAN;
  double qx = NAN = NAN;
  double qy = NAN = NAN;
  double qz = NAN = NAN;
  double qw = NAN = NAN;
  while (std::getline(fin, line)) {
    const auto first_non_space = line.find_first_not_of(" \t\r\n");
    if (first_non_space == std::string::npos) {
      continue;
    }
    if (line[first_non_space] == '#') {
      continue;
    }
    std::istringstream iss(line);
    if (!(iss >> time >> tx >> ty >> tz >> qx >> qy >> qz >> qw)) {
      continue;
    }

    poses.emplace_back(Quaterniond(qw, qx, qy, qz), Vector3d(tx, ty, tz));
  }

  return poses;
}
}  // namespace util
