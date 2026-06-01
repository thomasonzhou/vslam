#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <DBoW3/DBoW3.h>

int main(int argc, char **argv){
    // load images
    if (argc != 2){
        std::cout << "Second argument is training image path" << std::endl;
        return 1;
    }
    const std::string kImagePrefix = std::string(argv[1]) + "/";

    constexpr int kImageStart = 1;
    constexpr int kImageEnd = 10;
    constexpr int kImages = kImageEnd - kImageStart + 1;

    std::vector<cv::Mat> training_images;
    training_images.reserve(kImages);
    std::vector<cv::Mat> orb_descriptors;
    orb_descriptors.reserve(kImages);
    
    cv::Ptr<cv::FeatureDetector> orb_detector = cv::ORB::create();

    for (int img_num = kImageStart; img_num <= kImageEnd; ++img_num){
        std::vector<cv::KeyPoint> orb_keypoints;
        const std::string kImagePath = kImagePrefix + std::to_string(img_num) + ".png";
        training_images.emplace_back(cv::imread(kImagePath));
        cv::Mat descriptor;
        orb_detector->detectAndCompute(training_images.back(), cv::noArray(), orb_keypoints, descriptor);
        orb_descriptors.push_back(descriptor);
    }

    DBoW3::Vocabulary vocab;
    vocab.create(orb_descriptors);
    std::cout << vocab << std::endl;

    const std::string kSaveDir = kImagePrefix + "vocabulary.yml.gz";
    vocab.save(kSaveDir);
    return 0;

}