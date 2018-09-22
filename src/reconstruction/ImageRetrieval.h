#ifndef REALTIME_RECONSTRUCTION_IMAGERETRIEVAL_H
#define REALTIME_RECONSTRUCTION_IMAGERETRIEVAL_H

#include <theia/image/keypoint_detector/keypoint.h>
#include <theia/sfm/types.h>

#include <colmap/retrieval/visual_index.h>

class ImageRetrieval {
public:
    struct Options {
        // Path to pretrained vocabulary tree
        std::string vocab_tree_path;

        // Options for image indexing
        colmap::retrieval::VisualIndex<>::IndexOptions index_options;

        // Options for image query
        colmap::retrieval::VisualIndex<>::QueryOptions query_options;
    };

    explicit ImageRetrieval(const Options& options);

    void AddImage(theia::ViewId view_id,
                  const std::vector<theia::Keypoint>& keypoints,
                  const std::vector<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>>& descriptors);

    std::vector<colmap::retrieval::ImageScore> QueryImage(
            const std::vector<theia::Keypoint>& keypoints,
            const std::vector<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>>& descriptors);

private:
    Options options_;
    colmap::retrieval::VisualIndex<> visual_index_;
};


#endif //REALTIME_RECONSTRUCTION_IMAGERETRIEVAL_H
