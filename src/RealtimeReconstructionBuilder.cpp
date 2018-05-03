#include "RealtimeReconstructionBuilder.h"

#include <algorithm>

#include <theia/util/filesystem.h>
#include <theia/image/image.h>
#include <theia/image/descriptor/create_descriptor_extractor.h>
#include <theia/matching/create_feature_matcher.h>
#include <theia/matching/image_pair_match.h>
#include <theia/sfm/colorize_reconstruction.h>
#include <theia/sfm/reconstruction_estimator_utils.h>
#include <theia/io/write_ply_file.h>

namespace theia {

    RealtimeReconstructionBuilder::RealtimeReconstructionBuilder(const theia::ReconstructionBuilderOptions& options,
                                                                 const CameraIntrinsicsPrior& intrinsics_prior)
            : options_(options), intrinsics_prior_(intrinsics_prior) {

        // Initialize descriptor extractor
        descriptor_extractor_ = CreateDescriptorExtractor(options_.descriptor_type, options_.feature_density);
        descriptor_extractor_->Initialize();

        // Initialize matcher
        feature_matcher_ = CreateFeatureMatcher(options_.matching_strategy, options_.matching_options);

        // Initialize SfM objects
        view_graph_ = std::make_unique<ViewGraph>();
        reconstruction_ = std::make_unique<Reconstruction>();
        reconstruction_estimator_.reset(ReconstructionEstimator::Create(options_.reconstruction_estimator_options));
    }

    ReconstructionEstimatorSummary RealtimeReconstructionBuilder::InitializeReconstruction(
            const std::string &image1_fullpath,
            const std::string &image2_fullpath) {

        ReconstructionEstimatorSummary summary;

        // Check if already initialized
        if (reconstruction_->NumViews() > 0) {
            summary.message = "Reconstruction already initialized.";
            return summary;
        }

        // Read the images
        std::string image1_filename;
        GetFilenameFromFilepath(image1_fullpath, true, &image1_filename);
        FloatImage image1(image1_fullpath);
        image_filenames_.push_back(image1_filename);

        std::string image2_filename;
        GetFilenameFromFilepath(image2_fullpath, true, &image2_filename);
        FloatImage image2(image2_fullpath);
        image_filenames_.push_back(image2_filename);

        // Feature extraction
        std::vector<Keypoint> image1_keypoints;
        std::vector<Eigen::VectorXf> image1_descriptors;
        descriptor_extractor_->DetectAndExtractDescriptors(image1, &image1_keypoints, &image1_descriptors);

        std::vector<Keypoint> image2_keypoints;
        std::vector<Eigen::VectorXf> image2_descriptors;
        descriptor_extractor_->DetectAndExtractDescriptors(image2, &image2_keypoints, &image2_descriptors);

        // Feature matching
        feature_matcher_->AddImage(image1_filename, image1_keypoints, image1_descriptors, intrinsics_prior_);
        feature_matcher_->AddImage(image2_filename, image2_keypoints, image2_descriptors, intrinsics_prior_);

        std::vector<ImagePairMatch> matches;
        feature_matcher_->MatchImages(&matches);

        if (matches.empty()) {
            summary.message = "No matches found.";
            return summary;
        }

        // Add to reconstruction
        ViewId view1_id = reconstruction_->AddView(image1_filename, 0);
        ViewId view2_id = reconstruction_->AddView(image2_filename, 0);

        // Set intrinsics priors
        View* view1 = reconstruction_->MutableView(view1_id);
        *(view1->MutableCameraIntrinsicsPrior()) = intrinsics_prior_;

        View* view2 = reconstruction_->MutableView(view2_id);
        *(view2->MutableCameraIntrinsicsPrior()) = intrinsics_prior_;

        // Add matches to view graph
        ImagePairMatch match = matches.front();
        view_graph_->AddEdge(view1_id, view2_id, match.twoview_info);

        // Add tracks to reconstruction
        for (const auto& correspondence : match.correspondences) {
            const auto image1_feature = std::make_pair(view1_id, correspondence.feature1);
            const auto image2_feature = std::make_pair(view2_id, correspondence.feature2);

            // Build track
            std::vector<std::pair<ViewId, Feature>> track;
            track.emplace_back(image1_feature);
            track.emplace_back(image2_feature);

            TrackId track_id = reconstruction_->AddTrack(track);
            image_feature_to_track_id_.insert( {image1_feature, track_id} );
            image_feature_to_track_id_.insert( {image2_feature, track_id} );
        }

        // Build reconstruction
        summary = reconstruction_estimator_->Estimate(view_graph_.get(), reconstruction_.get());
        return summary;
    }

    ReconstructionEstimatorSummary RealtimeReconstructionBuilder::ExtendReconstruction(
            const std::string& image_fullpath) {

        ReconstructionEstimatorSummary summary;

        // Check if initialized
        if (reconstruction_->NumViews() == 0) {
            summary.message = "Reconstruction not initialized yet.";
            return summary;
        }

        // Read the image
        std::string image_filename;
        GetFilenameFromFilepath(image_fullpath, true, &image_filename);
        FloatImage image(image_fullpath);
        image_filenames_.push_back(image_filename);

        // Feature extraction
        std::vector<Keypoint> image_keypoints;
        std::vector<Eigen::VectorXf> image_descriptors;
        descriptor_extractor_->DetectAndExtractDescriptors(image, &image_keypoints, &image_descriptors);

        // Feature matching
        feature_matcher_->AddImage(image_filename, image_keypoints, image_descriptors, intrinsics_prior_);

        std::vector<std::pair<std::string, std::string>> pairs_to_match;
        for (int i = 0; i < (image_filenames_.size() - 1); i++) {
            std::pair<std::string, std::string> pair = {image_filenames_[i], image_filename};
            pairs_to_match.push_back(pair);
        }
        feature_matcher_->SetImagePairsToMatch(pairs_to_match);

        std::vector<ImagePairMatch> matches;
        feature_matcher_->MatchImages(&matches);

        if (matches.empty()) {
            image_filenames_.pop_back();
            summary.message = "No matches found.";
            return summary;
        }

        // Add to reconstruction
        ViewId view_id = reconstruction_->AddView(image_filename, 0);

        // Set intrinsics prior
        View* view = reconstruction_->MutableView(view_id);
        *(view->MutableCameraIntrinsicsPrior()) = intrinsics_prior_;

        // Add matches to view graph
        for (const auto& match : matches) {
            ViewId view1_id = reconstruction_->ViewIdFromName(match.image1);
            ViewId view2_id = reconstruction_->ViewIdFromName(match.image2);
            view_graph_->AddEdge(view1_id, view2_id, match.twoview_info);

            // Add tracks and observations to reconstruction
            for (const auto& correspondence : match.correspondences) {
                const auto image1_feature = std::make_pair(view1_id, correspondence.feature1);
                const auto image2_feature = std::make_pair(view2_id, correspondence.feature2);

                // Check if feature from the second view is not yet added
                if (!image_feature_to_track_id_.count(image2_feature)) {

                    // Check if feature from the first view is already added
                    if (image_feature_to_track_id_.count(image1_feature)) {

                        // Insert feature from second view
                        TrackId track_id = image_feature_to_track_id_[image1_feature];

                        // Because of outliers track may already contain the view_id we want to add,
                        // so before adding, we check if view is already in track (or view has feature in track)
                        if (view->GetFeature(track_id) == nullptr) {
                            reconstruction_->AddObservation(view2_id, track_id, correspondence.feature2);
                            image_feature_to_track_id_.insert( {image2_feature, track_id} );
                        }
                    } else {

                        // Build track
                        std::vector<std::pair<ViewId, Feature>> track;
                        track.emplace_back(image1_feature);
                        track.emplace_back(image2_feature);

                        TrackId track_id = reconstruction_->AddTrack(track);
                        image_feature_to_track_id_.insert( {image1_feature, track_id} );
                        image_feature_to_track_id_.insert( {image2_feature, track_id} );
                    }
                }
            }
        }

        // Build reconstruction
        summary = reconstruction_estimator_->Estimate(view_graph_.get(), reconstruction_.get());

        // TODO: remove after debugging
        if (reconstruction_->NumViews() != NumEstimatedViews(*reconstruction_)) {
            std::cout << "debug" << std::endl;
        }

        return summary;
    }

    bool RealtimeReconstructionBuilder::IsInitialized() {
        // TODO is initialized
        return false;
    }

    Eigen::MatrixXd RealtimeReconstructionBuilder::GetReconstructedPoints() {
        int num_points = reconstruction_->NumTracks();
        std::vector<TrackId> track_ids = reconstruction_->TrackIds();
        Eigen::MatrixXd points(num_points, 3);

        for (int i = 0; i < num_points; i++) {
            const Track* track = reconstruction_->Track(track_ids[i]);
            Eigen::Vector3d point = track->Point().hnormalized();
            points(i, 0) = point(0);
            points(i, 1) = point(1);
            points(i, 2) = point(2);
        }

        return points;
    }

    Eigen::MatrixXd RealtimeReconstructionBuilder::GetPointColors() {
        int num_points = reconstruction_->NumTracks();
        std::vector<TrackId> track_ids = reconstruction_->TrackIds();
        Eigen::MatrixXd colors(num_points, 3);

        for (int i = 0; i < num_points; i++) {
            const Track* track = reconstruction_->Track(track_ids[i]);
            Eigen::Matrix<uint8_t, 3, 1> color = track->Color();
            colors(i, 0) = color(0);
            colors(i, 1) = color(1);
            colors(i, 2) = color(2);
        }

        return colors;
    }

    Eigen::MatrixXd RealtimeReconstructionBuilder::GetCameraPositions() {
        int num_views = reconstruction_->NumViews();
        std::vector<ViewId> view_ids = reconstruction_->ViewIds();
        Eigen::MatrixXd cameras(num_views, 3);

        for (int i = 0; i < num_views; i++) {
            Eigen::Vector3d position = reconstruction_->View(view_ids[i])->Camera().GetPosition();
            cameras(i, 0) = position(0);
            cameras(i, 1) = position(1);
            cameras(i, 2) = position(2);
        }

        return cameras;
    }

    Reconstruction* RealtimeReconstructionBuilder::GetReconstruction() {
        return reconstruction_.get();
    }

    void RealtimeReconstructionBuilder::ColorizeReconstruction(const std::string& images_path) {
        theia::ColorizeReconstruction(images_path, options_.num_threads, reconstruction_.get());
    }

    void RealtimeReconstructionBuilder::WritePly(const std::string& output_fullpath) {
        theia::WritePlyFile(output_fullpath, *reconstruction_, 2);
    }

    void RealtimeReconstructionBuilder::RemoveView(ViewId view_id) {
        // TODO check if view_id exists
        // Remove from image filenames
        std::string image_filename = reconstruction_->View(view_id)->Name();
        image_filenames_.erase(std::remove(image_filenames_.begin(), image_filenames_.end(), image_filename),
                               image_filenames_.end());

        // Remove from reconstruction
        reconstruction_->RemoveView(view_id);

        // Remove tracks smaller than min_track_length
        for (const auto& track_id : reconstruction_->TrackIds()) {
            const Track* track = reconstruction_->Track(track_id);
            if (track->NumViews() < options_.min_track_length) {
                reconstruction_->RemoveTrack(track_id);
            }
        }

        // Remove view from view_graph
        view_graph_->RemoveView(view_id);

        // Remove from image_feature_to_track_id map
        for(auto it = image_feature_to_track_id_.begin(); it != image_feature_to_track_id_.end(); ) {
            std::pair<ViewId, Feature> key = it->first;
            if(key.first == view_id) {
                it = image_feature_to_track_id_.erase(it);
            } else {
                it++;
            }
        }
    }

    void RealtimeReconstructionBuilder::ResetReconstruction() {
        for (const auto& view_id :reconstruction_->ViewIds()) {
            RemoveView(view_id);
        }
    }

    void RealtimeReconstructionBuilder::PrintStatistics(std::ostream &stream,
                                                        bool print_images, bool print_reconstruction,
                                                        bool print_view_graph, bool print_feature_track_map) {
        if (print_images) {
            stream << "Image filenames: ";
            for (const auto &image_filename : image_filenames_) {
                stream << "\n\t" << image_filename
                       << " (id: " << reconstruction_->ViewIdFromName(image_filename) << ")";
            }
            stream << "\n\n";
        }

        if (print_reconstruction) {
            stream << "Reconstruction: ";
            stream << "\n\tNum views = " << reconstruction_->NumViews()
                   << "\n\tNum estimated views = " << NumEstimatedViews(*reconstruction_)
                   << "\n\tNum tracks = " << reconstruction_->NumTracks()
                   << "\n\tNum estimated tracks = " << NumEstimatedTracks(*reconstruction_)
                   << "\n\n";
        }

        if (print_view_graph) {
            stream << "View Graph: ";
            stream << "\n\tNum views = " << view_graph_->NumViews()
                   << "\n\tNum edges = " << view_graph_->NumEdges()
                   << "\n\tEdges = ";
            for (const auto &edge : view_graph_->GetAllEdges()) {
                ViewIdPair view_pair = edge.first;
                stream << "\t(" << view_pair.first << ", " << view_pair.second << ")";
            }
            stream << "\n\n";
        }

        if (print_feature_track_map) {
            stream << "Feature to track id map: ";
            stream << "\n\tNum elements = " << image_feature_to_track_id_.size() << "\n\n";
        }
    }
}

