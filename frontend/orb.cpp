#include "orb.h"

namespace frontend {
void compute_orb(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints,
                 std::vector<Descriptor>& descriptors) {
  // for each keypoint, determine the orientation
  constexpr int FAST_block_size = 16;
  constexpr int FAST_block_radius = FAST_block_size / 2;
  constexpr int BRIEF_radius = 16;
  constexpr int border = std::max(BRIEF_radius, FAST_block_radius) * 2;

  for (const cv::KeyPoint& kp : keypoints) {
    const int center_x = cvRound(kp.pt.x);
    const int center_y = cvRound(kp.pt.y);

    if (center_x < border || center_y < border ||
        center_x >= img.cols - border || center_y >= img.rows - border) {
      descriptors.push_back({});
      continue;
    }

    double m10 = 0.0;  // horizontal mass
    double m01 = 0.0;  // vertical mass

    for (int r = -FAST_block_radius; r <= FAST_block_radius; ++r) {
      for (int c = -FAST_block_radius; c <= FAST_block_radius; ++c) {
        uchar val = img.at<uchar>(center_y + r, center_x + c);
        m10 += c * val;
        m01 += r * val;
      }
    }

    constexpr double epsilon = 10e-9;  // for numerical stability
    const double radius = std::sqrt(m10 * m10 + m01 * m01) + epsilon;
    const double sin_theta = m01 / radius;
    const double cos_theta = m10 / radius;

    // rotation matrix is
    // cos_theta sin_theta
    // -sin_theta cos_theta

    Descriptor d(ORB_chunks, 0);
    for (int chunk = 0; chunk < ORB_chunks; ++chunk) {
      std::uint32_t di = 0;
      for (int bit = 0; bit < ORB_chunk_bits; ++bit) {
        const int pair_idx = (chunk * ORB_chunk_bits + bit) * 4;

        const cv::Point2d p1(ORB_bit_pattern_31[pair_idx],
                             ORB_bit_pattern_31[pair_idx + 1]);
        const cv::Point2d p2(ORB_bit_pattern_31[pair_idx + 2],
                             ORB_bit_pattern_31[pair_idx + 3]);

        const cv::Point2d p1r(p1.x * cos_theta - p1.y * sin_theta,
                              p1.x * sin_theta + p1.y * cos_theta);
        const cv::Point2d p2r(p2.x * cos_theta - p2.y * sin_theta,
                              p2.x * cos_theta + p2.y * sin_theta);

        const int y1 = center_y + cvRound(p1r.y);
        const int x1 = center_x + cvRound(p1r.x);
        const int y2 = center_y + cvRound(p2r.y);
        const int x2 = center_x + cvRound(p2r.x);

        const uchar val1 = img.at<uchar>(y1, x1);
        const uchar val2 = img.at<uchar>(y2, x2);

        if (val1 < val2) {
          di |= (std::uint32_t(1) << bit);
        }
      }
      d[chunk] = di;
    }
    descriptors.emplace_back(d);
  }
}

void brute_force_match(const std::vector<Descriptor>& d1,
                       const std::vector<Descriptor>& d2,
                       std::vector<cv::DMatch>& matches) {
  for (int i1 = 0; i1 < d1.size(); ++i1) {
    if (d1[i1].empty()) continue;
    cv::DMatch match{i1, placeholder_idx, max_hamming_dist};
    for (int i2 = 0; i2 < d2.size(); ++i2) {
      if (d2[i2].empty()) continue;

      int hamming_dist = 0;
      for (int chunk = 0; chunk < ORB_chunks; ++chunk) {
        hamming_dist += __builtin_popcount(d1[i1][chunk] ^ d2[i2][chunk]);
      }
      if (hamming_dist < match.distance) {
        match.distance = hamming_dist;
        match.trainIdx = i2;
      }
    }

    if (match.distance < accepted_hamming_dist) {
      matches.push_back(match);
    }
  }
}
};  // namespace frontend