#include <opencv2/core.hpp>

namespace geometry {

[[nodiscard]] inline double bilinear_interpolation(const cv::Mat &img, double x, double y){
    if (x < 0) x = 0.0;
    else if (x >= img.cols) x = img.cols - 1.0;

    if (y < 0) y = 0.0;
    else if (y >= img.rows) y = img.rows - 1.0;

    const int x0 = std::floor(x);
    const int y0 = std::floor(y);

    const int x1 = std::min(x0 + 1, img.cols - 1);
    const int y1 = std::min(y0 + 1, img.rows - 1);

    const double dx = x - static_cast<double>(x0);
    const double dy = y - static_cast<double>(y0);
    const double inv_dx = 1.0 - dx;
    const double inv_dy = 1.0 - dy;

    const double i00 = static_cast<double> (img.at<uchar>(y0, x0));
    const double i10 = static_cast<double> (img.at<uchar>(y1, x0));
    const double i01 = static_cast<double> (img.at<uchar>(y0, x1));
    const double i11 = static_cast<double> (img.at<uchar>(y1, x1));

    return (inv_dx * inv_dy * i00) + (inv_dx * dy * i10) + (dx * inv_dy * i01) + (dx * dy * i11);
}

}  // namespace geometry