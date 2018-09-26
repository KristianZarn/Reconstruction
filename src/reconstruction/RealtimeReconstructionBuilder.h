#ifndef REALTIME_RECONSTRUCTION_REALTIMERECONSTRUCTIONBUILDER_H
#define REALTIME_RECONSTRUCTION_REALTIMERECONSTRUCTIONBUILDER_H

#include <ostream>

#include <theia/image/descriptor/descriptor_extractor.h>
#include <theia/image/descriptor/create_descriptor_extractor.h>
#include <theia/matching/feature_matcher.h>
#include <theia/matching/create_feature_matcher.h>
#include <theia/matching/feature_matcher_options.h>
#include <theia/sfm/types.h>
#include <theia/sfm/reconstruction.h>
#include <theia/sfm/view_graph/view_graph.h>
#include <theia/sfm/reconstruction_estimator.h>
#include <theia/sfm/reconstruction_estimator_options.h>
#include <theia/sfm/reconstruction_builder.h>
#include <theia/sfm/estimators/estimate_calibrated_absolute_pose.h>

#include "SiftGpuDescriptorExtractor.h"
#include "ImageRetrieval.h"
#include "RealtimeFeatureMatcher.h"

namespace theia {

    class RealtimeReconstructionBuilder {
    public:
        struct Options {
            // The random number generator used to generate random numbers through the
            // reconstruction building process. If this is a nullptr then the random
            // generator will be initialized based on the current time.
            std::shared_ptr<RandomNumberGenerator> rng;

            // Number of threads used.
            int num_threads = 4;

            // Minimum allowable track length. Tracks that are too short are often not
            // well-constrained for triangulation and bundle adjustment.
            int min_track_length = 2;

            // Maximum allowable track length. Tracks that are too long are exceedingly
            // likely to contain outliers.
            int max_track_length = 50;

            // Options for descriptor extractor
            SiftGpuDescriptorExtractor::Options descriptor_extractor_options;

            // Options for computing matches between images.
            RealtimeFeatureMatcher::Options matching_options;

            // Options for estimating the reconstruction.
            ReconstructionEstimatorOptions reconstruction_estimator_options;
        };

        RealtimeReconstructionBuilder(const Options& options,
                                      const CameraIntrinsicsPrior& intrinsics_prior);

        // Initialize reconstruction with two images
        bool InitializeReconstruction(const std::string& image1_fullpath, const std::string& image2_fullpath);

        // Extend the reconstruction with additional image
        bool ExtendReconstruction(const std::string& image_fullpath);

        // Remove view from reconstruction by id
        bool RemoveView(ViewId view_id);

        // Remove all unestimated views from reconstruction
        bool RemoveUnestimatedViews();

        // Reset reconstruction by removing all views
        bool ResetReconstruction();

        // Global localization
        bool LocalizeImage(const FloatImage& image,
                           CalibratedAbsolutePose& pose);

        // Localization based on previous pose
        bool LocalizeImage(const FloatImage& image,
                           const CalibratedAbsolutePose& prev_pose,
                           CalibratedAbsolutePose& pose);

        // Helper function for localization
        bool LocalizeImage(const FloatImage& image,
                           const std::vector<ViewId>& views_to_match,
                           CalibratedAbsolutePose& pose);

        // Check if reconstruction is initialized
        bool IsInitialized();

        // Colorize reconstruction
        bool ColorizeReconstruction(const std::string& images_path);

        // Output point cloud to ply file
        bool WritePly(const std::string& output_fullpath);

        // Print reconstruction statistics
        void PrintStatistics(std::ostream& stream,
                             bool print_images = true,
                             bool print_reconstruction = true,
                             bool print_view_graph = true);

        // Get constant reference to reconstruction
        const Reconstruction& GetReconstruction();

        // Get message
        std::string GetMessage();

    private:
        Options options_;
        CameraIntrinsicsPrior intrinsics_prior_;
        std::string reconstruction_message_;

        // Feature extraction and matching
        std::unique_ptr<SiftGpuDescriptorExtractor> descriptor_extractor_;
        std::unique_ptr<ImageRetrieval> image_retrieval_;
        std::unique_ptr<RealtimeFeatureMatcher> feature_matcher_;

        // SfM objects
        std::unique_ptr<ViewGraph> view_graph_;
        std::unique_ptr<Reconstruction> reconstruction_;
        std::unique_ptr<ReconstructionEstimator> reconstruction_estimator_;
    };

} // namespace theia

#endif //REALTIME_RECONSTRUCTION_REALTIMERECONSTRUCTIONBUILDER_H
