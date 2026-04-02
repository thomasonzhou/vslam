#include <opencv2/core.hpp>
#include <vector>
#include <opencv2/core.hpp>

namespace frontend {

constexpr int kHalfPatchSize = 4;
constexpr int kMaxIter = 10;

class OpticalFlowTracker{
public:
OpticalFlowTracker(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const std::vector<cv::Point2d> &p1,
    std::vector<cv::Point2d> &p2,
    std::vector<bool> &status,
    std::vector<double> &error
) : img1_(img1), img2_(img2), p1_(p1), p2_(p2), status_(status), error_(error) {}

void trackFeatures(const cv::Range &range);

private:

const cv::Mat &img1_;
const cv::Mat &img2_;
const std::vector<cv::Point2d> &p1_;
std::vector<cv::Point2d> &p2_;
std::vector<bool> &status_;
std::vector<double> &error_;

};

void optical_flow_one_level(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const std::vector<cv::Point2d> &p1,
    std::vector<cv::Point2d> &p2,
    std::vector<bool> &status,
    std::vector<double> &error
);
    
void optical_flow_pyramid(
    const cv::Mat &img1,
    const cv::Mat &img2,
    const std::vector<cv::Point2d> &p1,
    std::vector<cv::Point2d> &p2,
    std::vector<bool> &status,
    std::vector<double> &error
);

}  // namespace frontend