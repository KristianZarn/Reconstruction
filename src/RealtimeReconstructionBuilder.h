#ifndef REALTIME_RECONSTRUCTION_REALTIMERECONSTRUCTIONBUILDER_H
#define REALTIME_RECONSTRUCTION_REALTIMERECONSTRUCTIONBUILDER_H

#include <ostream>

#include <theia/image/descriptor/descriptor_extractor.h>
#include <theia/image/descriptor/create_descriptor_extractor.h>
#include <theia/matching/feature_matcher.h>
#include <theia/matching/create_feature_matcher.h>
#include <theia/matching/feature_matcher_options.h>
#include <theia/sfm/types.h>
#include <theia/sfm/track.h>
#include <theia/sfm/track_builder.h>
#include <theia/sfm/reconstruction.h>
#include <theia/sfm/view_graph/view_graph.h>
#include <theia/sfm/reconstruction_estimator.h>
#include <theia/sfm/reconstruction_estimator_options.h>
#include <theia/sfm/reconstruction_builder.h>

#include "RealtimeFeatureMatcher.h"
// #include "CudaSiftDescriptorExtractor.h"
#include "PopSiftDescriptorExtractor.h"

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
            // CudaSiftDescriptorExtractor::Options descriptor_extractor_options;
            popsift::Config descriptor_extractor_options;

            // Options for computing matches between images.
            RealtimeFeatureMatcher::Options matching_options;

            // Options for estimating the reconstruction.
            ReconstructionEstimatorOptions reconstruction_estimator_options;
        };

        RealtimeReconstructionBuilder(const Options& options,
                                      const CameraIntrinsicsPrior& intrinsics_prior);

        // Builds initial reconstruction from two images
        ReconstructionEstimatorSummary InitializeReconstruction(const std::string& image1_fullpath,
                                                                const std::string& image2_fullpath);

        // Adds new image to the reconstruction
        ReconstructionEstimatorSummary ExtendReconstruction(const std::string& image_fullpath);

        bool IsInitialized();

        Reconstruction* GetReconstruction();

        void ColorizeReconstruction(const std::string& images_path);

        void WritePly(const std::string& output_fullpath);

        void RemoveView(ViewId view_id);

        void ResetReconstruction();

        void PrintStatistics(std::ostream& stream,
                             bool print_images = true,
                             bool print_reconstruction = true,
                             bool print_view_graph = true,
                             bool print_feature_track_map = true);

    private:
        Options options_;
        CameraIntrinsicsPrior intrinsics_prior_;

        // Feature extraction and matching
        std::unique_ptr<DescriptorExtractor> descriptor_extractor_;
        std::unique_ptr<RealtimeFeatureMatcher> feature_matcher_;

        std::unordered_map<std::pair<ViewId, Feature>, TrackId> image_feature_to_track_id_;

        // SfM objects
        std::unique_ptr<ViewGraph> view_graph_;
        std::unique_ptr<Reconstruction> reconstruction_;
        std::unique_ptr<ReconstructionEstimator> reconstruction_estimator_;

        // Helper functions
        void UpdateImageFeatureToTrackId();
    };

} // namespace theia

#endif //REALTIME_RECONSTRUCTION_REALTIMERECONSTRUCTIONBUILDER_H
