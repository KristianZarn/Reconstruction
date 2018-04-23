#include "RealtimeReconstructionBuilder.h"

#include "theia/util/filesystem.h"
#include "theia/image/image.h"
#include "theia/image/descriptor/create_descriptor_extractor.h"
#include "theia/matching/create_feature_matcher.h"
#include "theia/matching/image_pair_match.h"
#include <theia/sfm/track.h>

namespace theia {

    RealtimeReconstructionBuilder::RealtimeReconstructionBuilder(
            const theia::RealtimeReconstructionBuilderOptions &options)
            : options_(options) {

        // Initialize descriptor extractor
        descriptor_extractor_ = CreateDescriptorExtractor(options_.descriptor_type,
                                                          options_.feature_density);
        descriptor_extractor_->Initialize();

        // Initialize matcher
        options_.matcher_options.num_threads = options_.num_threads;
        feature_matcher_ = CreateFeatureMatcher(options_.matching_strategy,
                                                options_.matcher_options);

        // Initialize SfM objects
        options_.reconstruction_estimator_options.num_threads = options_.num_threads;
        track_builder_ = std::make_unique<TrackBuilder>(options_.min_track_length,
                                                        options_.max_track_length);
        view_graph_ = std::make_unique<ViewGraph>();
        reconstruction_ = std::make_unique<Reconstruction>();
        reconstruction_estimator_.reset(
                ReconstructionEstimator::Create(options_.reconstruction_estimator_options));
    }

    bool RealtimeReconstructionBuilder::InitializeReconstruction(const std::string &image1_filepath,
                                                                 const std::string &image2_filepath) {
        // Read images
        std::string image1_filename;
        GetFilenameFromFilepath(image1_filepath, false, &image1_filename);
        FloatImage image1(image1_filepath);

        std::string image2_filename;
        GetFilenameFromFilepath(image2_filepath, false, &image2_filename);
        FloatImage image2(image2_filepath);

        // Feature extraction
        std::vector<Keypoint> image1_keypoints;
        std::vector<Eigen::VectorXf> image1_descriptors;
        descriptor_extractor_->DetectAndExtractDescriptors(image1, &image1_keypoints, &image1_descriptors);

        std::vector<Keypoint> image2_keypoints;
        std::vector<Eigen::VectorXf> image2_descriptors;
        descriptor_extractor_->DetectAndExtractDescriptors(image2, &image2_keypoints, &image2_descriptors);

        // Feature matching
        feature_matcher_->AddImage(image1_filename, image1_keypoints, image1_descriptors, options_.intrinsics_prior);
        feature_matcher_->AddImage(image2_filename, image2_keypoints, image2_descriptors, options_.intrinsics_prior);

        std::vector<ImagePairMatch> matches;
        feature_matcher_->MatchImages(&matches);

        if (matches.empty()) {
            return false;
        }

        // Add to reconstruction
        ViewId view1_id = reconstruction_->AddView(image1_filename, 0);
        ViewId view2_id = reconstruction_->AddView(image2_filename, 1);

        // Set intrinsics priors
        View* view1 = reconstruction_->MutableView(view1_id);
        *(view1->MutableCameraIntrinsicsPrior()) = options_.intrinsics_prior;

        View* view2 = reconstruction_->MutableView(view2_id);
        *(view2->MutableCameraIntrinsicsPrior()) = options_.intrinsics_prior;

        // Add matches to view graph
        ImagePairMatch pair_match = matches.front();

        view_graph_->AddEdge(view1_id, view2_id, pair_match.twoview_info);

        // Add tracks to reconstruction
        for (const auto& match : pair_match.correspondences) {
            track_builder_->AddFeatureCorrespondence(
                    view1_id, match.feature1, view2_id, match.feature2);
        }
        track_builder_->BuildTracks(reconstruction_.get());

        // Build reconstruction
        ReconstructionEstimatorSummary summary =
                reconstruction_estimator_->Estimate(view_graph_.get(), reconstruction_.get());

        return summary.success;
    }

    bool RealtimeReconstructionBuilder::ExtendReconstruction() {
        return false;
    }

    Eigen::MatrixXd RealtimeReconstructionBuilder::GetReconstructedPoints() {
        int num_points = reconstruction_->NumTracks();
        std::vector<TrackId> track_ids = reconstruction_->TrackIds();
        Eigen::MatrixXd points(num_points, 3);

        for (int i = 0; i < num_points; i++) {
            const Track* track = reconstruction_->Track(track_ids[i]);
            Eigen::Vector4d point = track->Point();
            points(i, 0) = point(0);
            points(i, 1) = point(1);
            points(i, 2) = point(2);
        }

        return points;
    }

    Eigen::MatrixXd RealtimeReconstructionBuilder::GetCameraPositions() {

    }

}

