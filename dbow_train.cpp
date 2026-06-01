#include <opencv2/core.hpp>

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <DBoW3/DBoW3.h>

#include "util/multi_descriptor.h"

int main(int argc, char **argv){
    // load images
    if (argc != 2){
        std::cout << "Second argument is training image path" << std::endl;
        return 1;
    }
    const std::string kImagePrefix = std::string(argv[1]) + "/";

    constexpr int kImageStart = 1;
    constexpr int kImageEnd = 10;


    std::vector<cv::Mat> training_images;
    std::vector<cv::Mat> orb_descriptors;
    //

    util::multi_imread(training_images, kImagePrefix, kImageStart, kImageEnd);
    util::multi_descriptor_detect(orb_descriptors, training_images);

    DBoW3::Vocabulary vocab;
    vocab.create(orb_descriptors);
    std::cout << vocab << std::endl;

    const std::string kSaveDir = kImagePrefix + "vocabulary.bin";
    vocab.save(kSaveDir);
    return 0;

}