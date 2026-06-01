#include <iostream>
#include <string>

#include <DBoW3/DBoW3.h>

#include "util/multi_descriptor.h"

int main(int argc, char **argv){
    if (argc != 2){
        std::cerr << "second arg is the path to vocabulary file" << std::endl;
        return 1;
    }

    const std::string kFilePrefix = std::string(argv[1]) + "/";
    const std::string kDatabasePath = kFilePrefix + "vocabulary.bin";
    DBoW3::Vocabulary vocab(kDatabasePath);

    if (vocab.empty()){
        std::cerr << "vocab is empty" << std::endl;
        return 1;   
    }

    std::vector<cv::Mat> eval_images;

    constexpr int kImageStart = 1;
    constexpr int kImageEnd = 10;
    util::multi_imread(eval_images, kFilePrefix, kImageStart, kImageEnd);
    std::vector<cv::Mat> orb_descriptors;
    util::multi_descriptor_detect(orb_descriptors, eval_images);

    std::cout << "performing image to image comparison" << std::endl;

    for (size_t i = 0; i < orb_descriptors.size(); ++i){
        DBoW3::BowVector v1;
        vocab.transform(orb_descriptors[i], v1);
        for (size_t j = i; j < orb_descriptors.size(); ++j){
            DBoW3::BowVector v2;
            vocab.transform(orb_descriptors[j], v2);

            const double similarity_score = vocab.score(v1, v2);
            std::cout << "images " << i << ", " << j << ": " << similarity_score << std::endl;
        }
    }

    std::cout << "performing image to database comparison" << std::endl;

    constexpr bool kUseDirectIndex = false;
    constexpr int kDirectIndexLevels = 0;
    DBoW3::Database db(vocab, kUseDirectIndex, kDirectIndexLevels);

    for (const cv::Mat& descriptor: orb_descriptors){
        db.add(descriptor);
    }
    std::cout << db << std::endl;

    constexpr int kMaxQueryReturn = 4;
    for (size_t i = 0; i < orb_descriptors.size(); ++i){
        DBoW3::QueryResults res;
        db.query(orb_descriptors[i], res, kMaxQueryReturn);

        std::cout << "query image " << i << " returns " << res << std::endl;
    }

    return 0;
}