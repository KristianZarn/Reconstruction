#include <cstdlib>
#include <iostream>

#include <colmap/util/option_manager.h>
#include <colmap/util/string.h>

#include <colmap/base/image_reader.h>
#include <colmap/feature/utils.h>
#include <colmap/feature/extraction.h>
#include <colmap/retrieval/visual_index.h>

int main(int argc, char** argv) {

    std::string project_path =
            "/home/kristian/Documents/reconstruction_code/realtime_reconstruction/dataset/test/";
    std::string images_path = project_path + "images/";

    // Prepare database
    colmap::ImageReaderOptions reader_options;
    reader_options.database_path = project_path + "/database.db";
    reader_options.image_path = images_path;
    reader_options.single_camera = true;

    colmap::Database database(reader_options.database_path);

    // Extract features
    colmap::SiftExtractionOptions sift_options;
    colmap::SiftFeatureExtractor feature_extractor(reader_options, sift_options);
    feature_extractor.Start();
    feature_extractor.Wait();

    // Get images for image retrieval
    std::vector<colmap::Image> database_images;
    database_images.reserve(database.NumImages());
    for (const auto& image : database.ReadAllImages()) {
        database_images.push_back(image);
    }

    // Perform image indexing
    std::string vocab_tree_path =
            "/home/kristian/Documents/reconstruction_code/realtime_reconstruction/resources/vocab_tree_flickr100K_words32K.bin";
    colmap::retrieval::VisualIndex<> visual_index;
    visual_index.Read(vocab_tree_path);

    colmap::retrieval::VisualIndex<>::QueryOptions query_options;
    query_options.max_num_images = 8;

    std::unordered_map<colmap::image_t, const colmap::Image*> image_id_to_image;
    image_id_to_image.reserve(database_images.size());

    colmap::Timer timer;
    for (size_t i = 0; i < database_images.size(); ++i) {
        image_id_to_image.emplace(database_images[i].ImageId(), &database_images[i]);

        auto keypoints = database.ReadKeypoints(database_images[i].ImageId());
        auto descriptors = database.ReadDescriptors(database_images[i].ImageId());

        // Add image to index
        timer.Restart();
        std::cout << colmap::StringPrintf("Indexing image [%d/%d]", i + 1, database_images.size()) << std::flush;

        visual_index.Add(colmap::retrieval::VisualIndex<>::IndexOptions(),
                         database_images[i].ImageId(), keypoints, descriptors);
        visual_index.Prepare();

        std::cout << colmap::StringPrintf(" in %.3fs", timer.ElapsedSeconds()) << std::endl;

        // Query image
        timer.Restart();
        std::cout << colmap::StringPrintf("Querying for image %s [%d/%d]",
                                          database_images[i].Name().c_str(), i + 1,
                                          database_images.size()) << std::flush;

        std::vector<colmap::retrieval::ImageScore> image_scores;
        visual_index.Query(query_options, keypoints, descriptors, &image_scores);

        std::cout << colmap::StringPrintf(" in %.3fs", timer.ElapsedSeconds()) << std::endl;

        // Print results
        for (const auto& image_score : image_scores) {
            const auto& image = *image_id_to_image.at(image_score.image_id);
            std::cout << colmap::StringPrintf("  image_id=%d, image_name=%s, score=%f",
                                              image_score.image_id, image.Name().c_str(), image_score.score) << std::endl;
        }
    }

    return EXIT_SUCCESS;
}