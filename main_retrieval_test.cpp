#include <cstdlib>
#include <iostream>

#include <theia/image/image.h>
#include <theia/image/descriptor/descriptor_extractor.h>

#include <colmap/feature/sift.h>

#include "reconstruction/ImageRetrieval.h"
#include "reconstruction/PopSiftDescriptorExtractor.h"
#include "reconstruction/SiftGpuDescriptorExtractor.h"

int main(int argc, char** argv) {

    // Set path to images
    std::string project_path =
            "/home/kristian/Documents/reconstruction_code/realtime_reconstruction/dataset/test/";
    std::string images_path = project_path + "images/";

    std::vector<std::string> image_names;
    for (int i = 0; i < 62; i++) {
        std::stringstream ss;
        ss << std::setw(3) << std::setfill('0') << std::to_string(i);
        image_names.emplace_back("frame" + ss.str() + ".png");
    }

    // Prepare objects
    popsift::Config sift_options;
    sift_options.setMode(popsift::Config::VLFeat);
    sift_options.setNormMode(popsift::Config::NormMode::RootSift);
    sift_options.setOctaves(4);
    sift_options.setLevels(3);
    sift_options.setEdgeLimit(10.0f);
    sift_options.setThreshold(0.08f);
    theia::PopSiftDescriptorExtractor descriptor_extractor(sift_options);

    // colmap::SiftExtractionOptions sift_options;
    // theia::SiftGpuDescriptorExtractor descriptor_extractor(sift_options);

    ImageRetrieval::Options image_retrieval_options;
    image_retrieval_options.vocab_tree_path =
            "/home/kristian/Documents/reconstruction_code/realtime_reconstruction/resources/vocab_tree_flickr100K_words32K.bin";
    image_retrieval_options.query_options.max_num_images = 8;
    ImageRetrieval image_retrieval(image_retrieval_options);

    // Image retrieval test
    colmap::Timer timer;
    for (int i = 0; i < image_names.size(); i++) {

        std::string image_fullpath = images_path + image_names[i];

        // Extract features
        theia::FloatImage image(image_fullpath);
        std::vector<theia::Keypoint> keypoints;
        std::vector<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>> descriptors;

        descriptor_extractor.DetectAndExtractDescriptors(
                image, &keypoints, &descriptors);

        // Debug
        // std::cout << "keypoint: " << keypoints[0].x() << ", " << keypoints[0].y() << std::endl;
        // std::cout << "descriptor: " << descriptors[0].cast<int>() << std::endl;

        // Add to index
        timer.Restart();
        std::cout << colmap::StringPrintf("Indexing image [%d/%d]", i + 1, image_names.size()) << std::flush;

        image_retrieval.AddImage(i, keypoints, descriptors);

        std::cout << colmap::StringPrintf(" in %.3fs", timer.ElapsedSeconds()) << std::endl;

        // Query
        timer.Restart();
        std::cout << colmap::StringPrintf("Querying for image %s [%d/%d]",
                                          image_names[i].c_str(), i + 1,
                                          image_names.size()) << std::flush;

        std::vector<colmap::retrieval::ImageScore> image_scores =
                image_retrieval.QueryImage(keypoints, descriptors);

        std::cout << colmap::StringPrintf(" in %.3fs", timer.ElapsedSeconds()) << std::endl;

        // Print results
        for (const auto& image_score : image_scores) {
            std::cout << colmap::StringPrintf("  image_id=%d, image_name=%s, score=%f",
                    image_score.image_id,
                    image_names[image_score.image_id].c_str(),
                    image_score.score) << std::endl;
        }
    }

    return EXIT_SUCCESS;
}