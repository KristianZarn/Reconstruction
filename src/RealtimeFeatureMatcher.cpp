#include "RealtimeFeatureMatcher.h"
#include <memory>
#include <algorithm>

#include <theia/util/map_util.h>
#include <theia/matching/feature_matcher_utils.h>

namespace theia {

    RealtimeFeatureMatcher::RealtimeFeatureMatcher(const RealtimeFeatureMatcher::Options &options,
                                                   const CameraIntrinsicsPrior &intrinsics)
            : options_(options), intrinsics_(intrinsics) {}

    void RealtimeFeatureMatcher::AddImage(const std::string &image_name,
                                          const std::vector<Keypoint> &keypoints,
                                          const std::vector<Eigen::VectorXf> &descriptors) {

        image_names_.push_back(image_name);

        // Insert the features and descriptors into map.
        KeypointsAndDescriptors keypoints_and_descriptors;
        keypoints_and_descriptors.image_name = image_name;
        keypoints_and_descriptors.keypoints = keypoints;
        keypoints_and_descriptors.descriptors = descriptors;
        keypoints_and_descriptors_.insert(std::make_pair(image_name, keypoints_and_descriptors));

        // Initialize cascade hasher if necessary.
        InitializeCascadeHasher(static_cast<int>(descriptors[0].size()));

        // Create the hashing information.
        if (!ContainsKey(hashed_images_, image_name)) {
            hashed_images_[image_name] = cascade_hasher_->CreateHashedSiftDescriptors(descriptors);
        }
    }

    void RealtimeFeatureMatcher::RemoveImage(const std::string &image_name) {
        image_names_.erase(std::remove(image_names_.begin(), image_names_.end(), image_name), image_names_.end());
        keypoints_and_descriptors_.erase(image_name);
        hashed_images_.erase(image_name);
    }

    void RealtimeFeatureMatcher::MatchImages(std::vector<ImagePairMatch> *matches,
                                             const std::vector<std::pair<std::string, std::string> > &pairs_to_match) {
        for (const auto& pair_to_match : pairs_to_match) {
            const std::string image1_name = pair_to_match.first;
            const std::string image2_name = pair_to_match.second;

            ImagePairMatch image_pair_match;
            image_pair_match.image1 = image1_name;
            image_pair_match.image2 = image2_name;

            const KeypointsAndDescriptors& features1 = keypoints_and_descriptors_[image1_name];
            const KeypointsAndDescriptors& features2 = keypoints_and_descriptors_[image2_name];

            // Compute the visual matches from feature descriptors.
            std::vector<IndexedFeatureMatch> putative_matches;
            if (!MatchImagePair(features1, features2, &putative_matches)) {
                continue;
            }

            // Perform geometric verification if applicable.
            if (options_.perform_geometric_verification) {
                // If geometric verification fails, do not add the match to the output.
                if (!GeometricVerification(features1, features2, putative_matches, &image_pair_match)) {
                    continue;
                }
            } else {
                // If no geometric verification is performed then the putative matches are output.
                image_pair_match.correspondences.reserve(putative_matches.size());
                for (const auto& match :putative_matches) {
                    const Keypoint& keypoint1 = features1.keypoints[match.feature1_ind];
                    const Keypoint& keypoint2 = features2.keypoints[match.feature2_ind];
                    image_pair_match.correspondences.emplace_back(
                            Feature(keypoint1.x(), keypoint1.y()),
                            Feature(keypoint2.x(), keypoint2.y()));
                }
            }

            // Add pair match to matches.
            matches->push_back(image_pair_match);
        }
    }

    bool RealtimeFeatureMatcher::MatchImagePair(const KeypointsAndDescriptors &features1,
                                                const KeypointsAndDescriptors &features2,
                                                std::vector<IndexedFeatureMatch> *matches) {
        const double lowes_ratio = options_.lowes_ratio;

        // Get references to the hashed images for each set of features.
        HashedImage& hashed_features1 = hashed_images_[features1.image_name];
        HashedImage& hashed_features2 = hashed_images_[features2.image_name];

        cascade_hasher_->MatchImages(hashed_features1, features1.descriptors,
                                     hashed_features2, features2.descriptors,
                                     lowes_ratio, matches);

        // Symmetric matching
        if (matches->size() >= options_.min_num_feature_matches) {
            std::vector<IndexedFeatureMatch> backwards_matches;
            cascade_hasher_->MatchImages(hashed_features2,
                                         features2.descriptors,
                                         hashed_features1,
                                         features1.descriptors,
                                         lowes_ratio,
                                         &backwards_matches);
            IntersectMatches(backwards_matches, matches);
        }

        return (matches->size() >= options_.min_num_feature_matches);
    }

    bool RealtimeFeatureMatcher::GeometricVerification(const KeypointsAndDescriptors &features1,
                                                       const KeypointsAndDescriptors &features2,
                                                       const std::vector<IndexedFeatureMatch> &putative_matches,
                                                       ImagePairMatch *image_pair_match) {

        TwoViewMatchGeometricVerification geometric_verification(
                options_.geometric_verification_options,
                intrinsics_, intrinsics_,
                features1, features2,
                putative_matches);

        // Return whether geometric verification succeeds.
        bool success = geometric_verification.VerifyMatches(
                &image_pair_match->correspondences,
                &image_pair_match->twoview_info);

        return success;
    }

    void RealtimeFeatureMatcher::InitializeCascadeHasher(int descriptor_dimension) {
        if (cascade_hasher_ == nullptr && descriptor_dimension > 0) {
            cascade_hasher_ = std::make_unique<CascadeHasher>(options_.rng);
            cascade_hasher_->Initialize(descriptor_dimension);
        }
    }

} // namespace theia
