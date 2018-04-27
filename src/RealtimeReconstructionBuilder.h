#ifndef REALTIME_RECONSTRUCTION_REALTIMERECONSTRUCTIONBUILDER_H
#define REALTIME_RECONSTRUCTION_REALTIMERECONSTRUCTIONBUILDER_H

#include "theia/image/descriptor/descriptor_extractor.h"
#include "theia/image/descriptor/create_descriptor_extractor.h"
#include "theia/matching/feature_matcher.h"
#include "theia/matching/create_feature_matcher.h"
#include "theia/matching/feature_matcher_options.h"
#include "theia/sfm/types.h"
#include "theia/sfm/track_builder.h"
#include "theia/sfm/reconstruction.h"
#include "theia/sfm/view_graph/view_graph.h"
#include <theia/sfm/reconstruction_estimator.h>
#include <theia/sfm/reconstruction_estimator_options.h>

namespace theia {

    struct RealtimeReconstructionBuilderOptions {
        int num_threads = 4;

        // Calibration
        CameraIntrinsicsPrior intrinsics_prior;

        // Feature extraction options
        DescriptorExtractorType descriptor_type = DescriptorExtractorType::SIFT;
        FeatureDensity feature_density = FeatureDensity::NORMAL;

        // Matching options
        MatchingStrategy matching_strategy = MatchingStrategy::CASCADE_HASHING;
        FeatureMatcherOptions matcher_options;

        // SfM options
        int min_track_length = 2;
        int max_track_length = 50;
        ReconstructionEstimatorOptions reconstruction_estimator_options;
    };

    class RealtimeReconstructionBuilder {
    public:
        explicit RealtimeReconstructionBuilder(const RealtimeReconstructionBuilderOptions& options);

        // Builds initial reconstruction from two images
        ReconstructionEstimatorSummary InitializeReconstruction(const std::string& image1_filepath,
                                                                const std::string& image2_filepath);

        // Adds new image to the reconstruction
        bool ExtendReconstruction();

        // Return points (eigen matrix #V x 3)
        Eigen::MatrixXd GetReconstructedPoints();

        // Return point colors (eigen matrix #V x 3)
        Eigen::MatrixXd GetPointColors();

        // Return camera positions (eigen matrix #C x 3)
        Eigen::MatrixXd GetCameraPositions();

        // Get pointer to reconstruction object
        Reconstruction* GetReconstruction();

        // Reset reconstruction
        void ResetReconstruction();

    private:
        RealtimeReconstructionBuilderOptions options_;

        // Initialization flag
        bool initialized_ = false;

        // Images
        std::vector<std::string> image_filepaths_;

        // Feature extraction and matching
        std::unique_ptr<DescriptorExtractor> descriptor_extractor_;
        std::unique_ptr<FeatureMatcher> feature_matcher_;

        // SfM objects
        std::unique_ptr<TrackBuilder> track_builder_;
        std::unique_ptr<ViewGraph> view_graph_;
        std::unique_ptr<Reconstruction> reconstruction_;
        std::unique_ptr<ReconstructionEstimator> reconstruction_estimator_;
    };

} // namespace theia

#endif //REALTIME_RECONSTRUCTION_REALTIMERECONSTRUCTIONBUILDER_H
