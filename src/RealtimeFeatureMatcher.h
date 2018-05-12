#ifndef THEIA_RECONSTRUCTION_REALTIMEFEATUREMATCHER_H
#define THEIA_RECONSTRUCTION_REALTIMEFEATUREMATCHER_H

#include <theia/matching/image_pair_match.h>
#include <theia/matching/cascade_hasher.h>
#include <theia/sfm/two_view_match_geometric_verification.h>

namespace theia {

    class RealtimeFeatureMatcher {
    public:
        struct Options {
            // Random number generator used in cascade hasher
            std::shared_ptr<RandomNumberGenerator> rng;

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
            TwoViewMatchGeometricVerification::Options geometric_verification_options;

            // Only images that contain more feature matches than this number will be returned.
            int min_num_feature_matches = 30;
        };

        RealtimeFeatureMatcher(const Options& matcher_options, const CameraIntrinsicsPrior& intrinsics);

        // Adds an image to the matcher. The intrinsics are used for geometric verification.
        void AddImage(const std::string& image_name,
                      const std::vector<Keypoint>& keypoints,
                      const std::vector<Eigen::VectorXf>& descriptors);

        void RemoveImage(const std::string & image_name);

        // Matches features between image pairs. Only the matches which pass the have greater than
        // min_num_feature_matches are returned.
        void MatchImages(std::vector<ImagePairMatch>* matches,
                         const std::vector<std::pair<std::string, std::string>>& pairs_to_match);

    private:
        // Returns true if the image pair is a valid match.
        bool MatchImagePair(const KeypointsAndDescriptors& features1,
                            const KeypointsAndDescriptors& features2,
                            std::vector<IndexedFeatureMatch>* matches);

        // Performs geometric verification.
        bool GeometricVerification(const KeypointsAndDescriptors& features1,
                                   const KeypointsAndDescriptors& features2,
                                   const std::vector<IndexedFeatureMatch>& putative_matches,
                                   ImagePairMatch* image_pair_match);

        // Initializes the cascade hasher (only if needed).
        void InitializeCascadeHasher(int descriptor_dimension);

        Options options_;
        CameraIntrinsicsPrior intrinsics_;
        std::vector<std::string> image_names_;
        std::unordered_map<std::string, KeypointsAndDescriptors> keypoints_and_descriptors_;

        std::unordered_map<std::string, HashedImage> hashed_images_;
        std::unique_ptr<CascadeHasher> cascade_hasher_;
    };

} // namespace theia

#endif //THEIA_RECONSTRUCTION_REALTIMEFEATUREMATCHER_H
