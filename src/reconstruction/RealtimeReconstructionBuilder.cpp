#include "RealtimeReconstructionBuilder.h"
#include <algorithm>

#include <theia/util/filesystem.h>
#include <theia/image/image.h>
#include <theia/image/descriptor/sift_descriptor.h>
#include <theia/matching/create_feature_matcher.h>
#include <theia/matching/image_pair_match.h>
#include <theia/sfm/colorize_reconstruction.h>
#include <theia/sfm/reconstruction_estimator_utils.h>
#include <theia/io/write_ply_file.h>

namespace theia {

    RealtimeReconstructionBuilder::RealtimeReconstructionBuilder(const Options& options,
                                                                 const CameraIntrinsicsPrior& intrinsics_prior)
            : options_(options), intrinsics_prior_(intrinsics_prior) {

        // Initialize descriptor extractor
        // SiftParameters sift_params;
        // descriptor_extractor_ = std::make_unique<SiftDescriptorExtractor>(sift_params);
        // descriptor_extractor_ = std::make_unique<CudaSiftDescriptorExtractor>(options_.descriptor_extractor_options);
        descriptor_extractor_ = std::make_unique<PopSiftDescriptorExtractor>(options_.descriptor_extractor_options);
        descriptor_extractor_->Initialize();

        // Initialize matcher
        feature_matcher_ = std::make_unique<RealtimeFeatureMatcher>(options_.matching_options, intrinsics_prior_);

        // Initialize SfM objects
        view_graph_ = std::make_unique<ViewGraph>();
        reconstruction_ = std::make_unique<Reconstruction>();
        reconstruction_estimator_.reset(ReconstructionEstimator::Create(options_.reconstruction_estimator_options));
    }

    ReconstructionEstimatorSummary RealtimeReconstructionBuilder::InitializeReconstruction(
            const std::string &image1_fullpath,
            const std::string &image2_fullpath) {

        ReconstructionEstimatorSummary summary;

        // Check if initialized
        if (IsInitialized()) {
            summary.success = false;
            summary.message = "Reconstruction is already initialized.";
            return summary;
        }

        // Read the images
        if (!theia::FileExists(image1_fullpath)) {
            summary.success = false;
            summary.message = "Image: " + image1_fullpath + " does not exist\n";
            return summary;
        }
        std::string image1_filename;
        GetFilenameFromFilepath(image1_fullpath, true, &image1_filename);
        FloatImage image1(image1_fullpath);

        if (!theia::FileExists(image2_fullpath)) {
            summary.success = false;
            summary.message = "Image: " + image2_fullpath + " does not exist\n";
            return summary;
        }
        std::string image2_filename;
        GetFilenameFromFilepath(image2_fullpath, true, &image2_filename);
        FloatImage image2(image2_fullpath);

        // Feature extraction
        std::vector<Keypoint> image1_keypoints;
        std::vector<Eigen::VectorXf> image1_descriptors;
        descriptor_extractor_->DetectAndExtractDescriptors(image1, &image1_keypoints, &image1_descriptors);

        std::vector<Keypoint> image2_keypoints;
        std::vector<Eigen::VectorXf> image2_descriptors;
        descriptor_extractor_->DetectAndExtractDescriptors(image2, &image2_keypoints, &image2_descriptors);

        // Feature matching
        feature_matcher_->AddImage(image1_filename, image1_keypoints, image1_descriptors);
        feature_matcher_->AddImage(image2_filename, image2_keypoints, image2_descriptors);

        std::vector<ImagePairMatch> matches;
        std::vector<std::pair<std::string, std::string>> pairs_to_match;
        pairs_to_match.emplace_back(std::make_pair(image1_filename, image2_filename));
        feature_matcher_->MatchImages(&matches, pairs_to_match);

        // Add to reconstruction
        ViewId view1_id = reconstruction_->AddView(image1_filename, 0);
        ViewId view2_id = reconstruction_->AddView(image2_filename, 0);

        // Set intrinsics priors
        View* view1 = reconstruction_->MutableView(view1_id);
        *(view1->MutableCameraIntrinsicsPrior()) = intrinsics_prior_;

        View* view2 = reconstruction_->MutableView(view2_id);
        *(view2->MutableCameraIntrinsicsPrior()) = intrinsics_prior_;

        // Add matches to view graph
        if (!matches.empty()) {
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
        } else {
            summary.success = false;
            summary.message = "No matches found.";
            ResetReconstruction();
            return summary;
        }

        // Build reconstruction
        summary = reconstruction_estimator_->Estimate(view_graph_.get(), reconstruction_.get());

        // Check if both views were estimated successfully
        if (reconstruction_->NumViews() != NumEstimatedViews(*reconstruction_)) {
            summary.success = false;
            summary.message = "Views were not estimated.";
            ResetReconstruction();
        }
        return summary;
    }

    ReconstructionEstimatorSummary RealtimeReconstructionBuilder::ExtendReconstruction(
            const std::string& image_fullpath) {

        ReconstructionEstimatorSummary summary;

        // Check if initialized
        if (!IsInitialized()) {
            summary.success = false;
            summary.message = "Reconstruction is not initialized.";
            return summary;
        }

        // Read the image
        if (!theia::FileExists(image_fullpath)) {
            summary.success = false;
            summary.message = "Image: " + image_fullpath + " does not exist\n";
            return summary;
        }
        std::string image_filename;
        GetFilenameFromFilepath(image_fullpath, true, &image_filename);
        FloatImage image(image_fullpath);

        // Feature extraction
        std::vector<Keypoint> image_keypoints;
        std::vector<Eigen::VectorXf> image_descriptors;
        descriptor_extractor_->DetectAndExtractDescriptors(image, &image_keypoints, &image_descriptors);

        // Feature matching
        feature_matcher_->AddImage(image_filename, image_keypoints, image_descriptors);

        std::vector<std::pair<std::string, std::string>> pairs_to_match;
        for (const auto& view_id : reconstruction_->ViewIds()) {
            std::string other_filename = reconstruction_->View(view_id)->Name();
            pairs_to_match.emplace_back(std::make_pair(other_filename, image_filename));
        }

        std::vector<ImagePairMatch> matches;
        feature_matcher_->MatchImages(&matches, pairs_to_match);

        // Add to reconstruction
        ViewId view_id = reconstruction_->AddView(image_filename, 0);

        // Set intrinsics prior
        View* view = reconstruction_->MutableView(view_id);
        *(view->MutableCameraIntrinsicsPrior()) = intrinsics_prior_;

        // Add matches to view graph
        if (!matches.empty()) {
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
                            // so before adding, check if view is already in track (or view has feature in track)
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
        } else {
            summary.success = false;
            summary.message = "No matches found.";
            return summary;
        }

        // Build reconstruction
        summary = reconstruction_estimator_->Estimate(view_graph_.get(), reconstruction_.get());
        UpdateImageFeatureToTrackId();

        // Check if view was added successfully
        if (reconstruction_->NumViews() != NumEstimatedViews(*reconstruction_)) {
            summary.success = false;
            summary.message = "View could not be added.";
        }
        return summary;
    }

    void RealtimeReconstructionBuilder::RemoveView(ViewId view_id) {
        const View* view = reconstruction_->View(view_id);
        if (view != nullptr) {
            // Remove from matcher
            feature_matcher_->RemoveImage(reconstruction_->View(view_id)->Name());

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

            // Update image_feature_to_track_id because tracks were removed
            UpdateImageFeatureToTrackId();
        }
    }

    void RealtimeReconstructionBuilder::ResetReconstruction() {
        for (const auto& view_id :reconstruction_->ViewIds()) {
            RemoveView(view_id);
        }
    }

    bool RealtimeReconstructionBuilder::LocalizeImage(theia::FloatImage image) {

    }

    bool RealtimeReconstructionBuilder::IsInitialized() {
        return (reconstruction_->NumViews() > 0);
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

    void RealtimeReconstructionBuilder::PrintStatistics(std::ostream &stream,
                                                        bool print_images,
                                                        bool print_reconstruction,
                                                        bool print_view_graph,
                                                        bool print_feature_track_map) {
        if (print_images) {
            stream << "Images: ";
            for (const auto& view_id : reconstruction_->ViewIds()) {
                stream << "\n\tid: " << view_id << ", name: " << reconstruction_->View(view_id)->Name();
            }
            stream << "\n";
        }
        if (print_reconstruction) {
            stream << "Reconstruction: ";
            stream << "\n\tNum views = " << reconstruction_->NumViews()
                   << "\n\tNum estimated views = " << NumEstimatedViews(*reconstruction_)
                   << "\n\tNum tracks = " << reconstruction_->NumTracks()
                   << "\n\tNum estimated tracks = " << NumEstimatedTracks(*reconstruction_)
                   << "\n";
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
            stream << "\n";
        }
        if (print_feature_track_map) {
            stream << "Feature to track id map: ";
            stream << "\n\tNum elements = " << image_feature_to_track_id_.size() << "\n";
        }
    }

    void RealtimeReconstructionBuilder::UpdateImageFeatureToTrackId() {
        // Remove from image_feature_to_track_id if track_id is not in reconstruction
        for(auto it = image_feature_to_track_id_.begin(); it != image_feature_to_track_id_.end(); ) {
            TrackId value = it->second;
            const Track* track = reconstruction_->Track(value);
            if(track == nullptr) {
                it = image_feature_to_track_id_.erase(it);
            } else {
                it++;
            }
        }
    }

} // namespace theia
