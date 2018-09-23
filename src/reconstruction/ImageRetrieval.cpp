#include "ImageRetrieval.h"

#include <colmap/feature/utils.h>

ImageRetrieval::ImageRetrieval(const Options &options)
        : options_(options) {
    visual_index_.Read(options_.vocab_tree_path);
}

void ImageRetrieval::AddImage(theia::ViewId view_id,
                              const std::vector<theia::Keypoint>& keypoints,
                              const std::vector<Eigen::VectorXf>& descriptors) {

    assert(!keypoints.empty());
    assert(keypoints.size() == descriptors.size());

    // Convert from Theia to Colmap
    colmap::FeatureKeypoints colmap_keypoints;
    colmap::FeatureDescriptors colmap_descriptors;
    convertFromTheiaToColmap(keypoints, descriptors, &colmap_keypoints, &colmap_descriptors);

    // Add to visual index
    visual_index_.Add(options_.index_options, view_id, colmap_keypoints, colmap_descriptors);
    visual_index_.Prepare();
}

std::vector<colmap::retrieval::ImageScore> ImageRetrieval::QueryImage(
        const std::vector<theia::Keypoint>& keypoints,
        const std::vector<Eigen::VectorXf>& descriptors) {

    assert(!keypoints.empty());
    assert(keypoints.size() == descriptors.size());

    // Convert from Theia to Colmap
    colmap::FeatureKeypoints colmap_keypoints;
    colmap::FeatureDescriptors colmap_descriptors;
    convertFromTheiaToColmap(keypoints, descriptors, &colmap_keypoints, &colmap_descriptors);

    // Query
    std::vector<colmap::retrieval::ImageScore> image_scores;
    visual_index_.Query(options_.query_options, colmap_keypoints, colmap_descriptors, &image_scores);
    return image_scores;
}

void ImageRetrieval::convertFromTheiaToColmap(const std::vector<theia::Keypoint> &keypoints_theia,
                                              const std::vector<Eigen::VectorXf> &descriptors_theia,
                                              colmap::FeatureKeypoints *keypoints_colmap,
                                              colmap::FeatureDescriptors *descriptors_colmap) {

    // Convert keypoints
    for (const auto &keypoint : keypoints_theia) {
        keypoints_colmap->emplace_back(
                colmap::FeatureKeypoint(
                        static_cast<float>(keypoint.x()),
                        static_cast<float>(keypoint.y()),
                        static_cast<float>(keypoint.scale()),
                        static_cast<float>(keypoint.orientation())
                )
        );
    }

    // Convert descriptors
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
            descriptors_float(descriptors_theia.size(), descriptors_theia.front().size());
    for (int i = 0; i < descriptors_theia.size(); i++) {
        descriptors_float.row(i) = descriptors_theia[i];
    }
    *descriptors_colmap = colmap::FeatureDescriptorsToUnsignedByte(descriptors_float);

    // Keep largest-scale features
    colmap::ExtractTopScaleFeatures(keypoints_colmap, descriptors_colmap,
                                    static_cast<size_t>(options_.max_num_features));
}