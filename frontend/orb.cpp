#include <string>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

#include <vector>
#include <chrono>
#include <iostream>
#include <algorithm>

int main(int argc, char** argv){
    const std::string img1_path = "../1.png";
    const std::string img2_path = "../2.png";

    cv::Mat img1 = cv::imread(img1_path, cv::IMREAD_COLOR);
    cv::Mat img2 = cv::imread(img2_path, cv::IMREAD_COLOR);

    assert(img1.data != nullptr && img2.data != nullptr);

    std::vector<cv::KeyPoint> keypoints1;
    std::vector<cv::KeyPoint> keypoints2;

    cv::Mat descriptors1;
    cv::Mat descriptors2;

    cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
    cv::Ptr<cv::DescriptorExtractor> descriptor = cv::ORB::create();
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create("BruteForce-Hamming");

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    detector->detect(img1, keypoints1);
    detector->detect(img2, keypoints2);

    descriptor->compute(img1, keypoints1, descriptors1);
    descriptor->compute(img2, keypoints2, descriptors2);

    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
    std::chrono::duration<double> orb_time = std::chrono::duration_cast<std::chrono::duration<double>> (t2 - t1);

    std::cout << "ORB time: " << orb_time.count() << std::endl;

    cv::Mat outimg1;
    cv::drawKeypoints(img1, keypoints1, outimg1);
    cv::imshow("ORB keypoints", outimg1);
    
    std::vector<cv::DMatch> matches;
    t1 = std::chrono::steady_clock::now();
    matcher->match(descriptors1, descriptors2, matches);
    t2 = std::chrono::steady_clock::now();
    std::cout << "match time " << std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() << " seconds" << std::endl;
    
    const auto min_max = std::minmax_element(matches.begin(), matches.end(), [](const cv::DMatch &m1, const cv::DMatch &m2){return m1.distance < m2.distance;});
    const double min_element = min_max.first->distance;
    const double max_element = min_max.second->distance;
    std::cout << "maxdist " << max_element << std::endl;
    std::cout << "mindist " << min_element << std::endl;

    // outlier rejection
    std::vector<cv::DMatch> good_matches;

    const double threshold = std::max(min_element * 2.0, 30.0);
    for (const cv::DMatch& match: matches){
        if (match.distance <= threshold){
            good_matches.emplace_back(match);
        }
    }

    cv::Mat img_match;
    cv::Mat img_goodmatch;

    cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, img_match);
    cv::drawMatches(img1, keypoints1, img2, keypoints2, good_matches, img_goodmatch);

    cv::imshow("matches", img_match);
    cv::imshow("good matches", img_goodmatch);

    cv::waitKey(0);
}