#include "ImageRetrieval.h"

#include <colmap/feature/types.h>

ImageRetrieval::ImageRetrieval(const Options &options)
        : options_(options) {
    visual_index_.Read(options_.vocab_tree_path);
}

void ImageRetrieval::AddImage(theia::ViewId view_id,
                              const std::vector<theia::Keypoint>& keypoints,
                              const std::vector<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>>& descriptors) {

    assert(!keypoints.empty());
    assert(keypoints.size() == descriptors.size());

    // Convert from Theia to Colmap
    colmap::FeatureKeypoints colmap_keypoints;
    for (const auto& keypoint : keypoints) {
        colmap_keypoints.emplace_back(
                colmap::FeatureKeypoint(
                        static_cast<float>(keypoint.x()),
                        static_cast<float>(keypoint.y()),
                        static_cast<float>(keypoint.scale()),
                        static_cast<float>(keypoint.orientation())
                )
        );
    }

    colmap::FeatureDescriptors colmap_descriptors(descriptors.size(), descriptors.front().size());
    for (int i = 0; i < descriptors.size(); i++) {
        colmap_descriptors.row(i) = descriptors[i];
    }

    // Add to visual index
    visual_index_.Add(options_.index_options, view_id,
                      colmap_keypoints, colmap_descriptors);
    visual_index_.Prepare();
}

std::vector<colmap::retrieval::ImageScore> ImageRetrieval::QueryImage(
        const std::vector<theia::Keypoint>& keypoints,
        const std::vector<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>>& descriptors) {

    // Convert from Theia to Colmap
    colmap::FeatureKeypoints colmap_keypoints;
    for (const auto& keypoint : keypoints) {
        colmap_keypoints.emplace_back(
                colmap::FeatureKeypoint(
                        static_cast<float>(keypoint.x()),
                        static_cast<float>(keypoint.y()),
                        static_cast<float>(keypoint.scale()),
                        static_cast<float>(keypoint.orientation())
                )
        );
    }

    colmap::FeatureDescriptors colmap_descriptors(descriptors.size(), descriptors.front().size());
    for (int i = 0; i < descriptors.size(); i++) {
        colmap_descriptors.row(i) = descriptors[i];
    }

    // Query
    std::vector<colmap::retrieval::ImageScore> image_scores;
    visual_index_.Query(options_.query_options, colmap_keypoints, colmap_descriptors, &image_scores);
    return image_scores;
}