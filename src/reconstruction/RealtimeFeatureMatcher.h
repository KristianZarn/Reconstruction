#ifndef THEIA_RECONSTRUCTION_REALTIMEFEATUREMATCHER_H
#define THEIA_RECONSTRUCTION_REALTIMEFEATUREMATCHER_H

#include <theia/matching/image_pair_match.h>
#include <theia/matching/cascade_hasher.h>
#include <theia/sfm/two_view_match_geometric_verification.h>

class RealtimeFeatureMatcher {
public:
    struct Options {
        // Random number generator used in cascade hasher
        std::shared_ptr<theia::RandomNumberGenerator> rng;

        // Number of threads to use in parallel for matching.
        int num_threads = 1;

        // Only keep the matches that pass the lowes ratio test such that the distance
        // of the best best match is less than lowes_ratio of the distance of the
        // second nearest neighbor match.
        bool use_lowes_ratio = true;
        float lowes_ratio = 0.8;

        // After performing feature matching with descriptors typically the 2-view
        // geometry is estimated using RANSAC (from the matched descriptors) and only
        // the features that support the estimated geometry are "verified" as
        // plausible and are kept. If set to true, geometric verification will be
        // performed to obtain higher quality matches.
        bool perform_geometric_verification = true;

        // The parameter settings for geometric verification.
        theia::TwoViewMatchGeometricVerification::Options geometric_verification_options;

        // Only images that contain more feature matches than this number will be returned.
        int min_num_feature_matches = 200;
    };

    RealtimeFeatureMatcher(const Options& matcher_options, const theia::CameraIntrinsicsPrior& intrinsics);

    // Adds an image to the matcher. The intrinsics are used for geometric verification.
    void AddImage(const std::string& image_name,
                  const std::vector<theia::Keypoint>& keypoints,
                  const std::vector<Eigen::VectorXf>& descriptors);

    void RemoveImage(const std::string & image_name);

    // Matches features between image pairs. Only the matches which pass the have greater than
    // min_num_feature_matches are returned.
    void MatchImages(std::vector<theia::ImagePairMatch>* matches,
                     const std::vector<std::pair<std::string, std::string>>& pairs_to_match);

    // Performs geometric verification.
    bool GeometricVerification(const theia::KeypointsAndDescriptors& features1,
                               const theia::KeypointsAndDescriptors& features2,
                               const std::vector<theia::IndexedFeatureMatch>& putative_matches,
                               theia::ImagePairMatch* image_pair_match);

    // Returns true if the image pair is a valid match.
    bool MatchImagePair(const theia::KeypointsAndDescriptors& features1,
                        const theia::KeypointsAndDescriptors& features2,
                        std::vector<theia::IndexedFeatureMatch>* matches);



    // Initializes the cascade hasher (only if needed).
    void InitializeCascadeHasher(int descriptor_dimension);

    Options options_;
    theia::CameraIntrinsicsPrior intrinsics_;
    std::vector<std::string> image_names_;
    std::unordered_map<std::string, theia::KeypointsAndDescriptors> keypoints_and_descriptors_;

    std::unordered_map<std::string, theia::HashedImage> hashed_images_;
    std::unique_ptr<theia::CascadeHasher> cascade_hasher_;
};

#endif //THEIA_RECONSTRUCTION_REALTIMEFEATUREMATCHER_H
