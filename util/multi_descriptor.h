#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>

namespace util {

void multi_imread(std::vector<cv::Mat>& images, const std::string& img_prefix,
                  const int img_start, const int img_end) {
  const int image_count = img_end - img_start + 1;
  images.reserve(image_count);
  for (int img_num = img_start; img_num <= img_end; ++img_num) {
    std::vector<cv::KeyPoint> orb_keypoints;
    const std::string kImagePath =
        img_prefix + std::to_string(img_num) + ".png";
    images.emplace_back(cv::imread(kImagePath));
  }
}

void multi_descriptor_detect(std::vector<cv::Mat>& orb_descriptors,
                             const std::vector<cv::Mat>& images) {
  orb_descriptors.reserve(images.size());
  cv::Ptr<cv::FeatureDetector> orb_detector = cv::ORB::create();

  for (const cv::Mat& img : images) {
    std::vector<cv::KeyPoint> orb_keypoints;
    cv::Mat descriptor;
    orb_detector->detectAndCompute(img, cv::noArray(), orb_keypoints,
                                   descriptor);
    orb_descriptors.push_back(descriptor);
  }
}
}  // namespace util