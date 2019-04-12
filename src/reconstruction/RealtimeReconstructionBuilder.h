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

class RealtimeReconstructionBuilder {
public:
    struct Options {
        // The random number generator used to generate random numbers through the
        // reconstruction building process. If this is a nullptr then the random
        // generator will be initialized based on the current time.
        std::shared_ptr<theia::RandomNumberGenerator> rng;

        // Number of threads used.
        int num_threads = 4;

        // Camera intrinsics prior information
        theia::CameraIntrinsicsPrior intrinsics_prior;

        // Options for descriptor extractor
        SiftGpuDescriptorExtractor::Options descriptor_extractor_options;

        // Options for image retrieval
        ImageRetrieval::Options image_retrieval_options;

        // Options for computing matches between images.
        RealtimeFeatureMatcher::Options matching_options;

        // Options for estimating the reconstruction.
        theia::ReconstructionEstimatorOptions reconstruction_estimator_options;
    };

    explicit RealtimeReconstructionBuilder(const Options& options);

    // Initialize reconstruction with two images
    bool InitializeReconstruction(const std::string& image1_fullpath, const std::string& image2_fullpath);

    // Extend the reconstruction with additional image
    bool ExtendReconstruction(const std::string& image_fullpath);

    // Remove view from reconstruction by id
    bool RemoveView(theia::ViewId view_id);

    // Remove all unestimated views from reconstruction
    bool RemoveUnestimatedViews();

    // Reset reconstruction by removing all views
    bool ResetReconstruction();

    // Global localization
    bool LocalizeImage(const theia::FloatImage& image,
                       theia::CalibratedAbsolutePose& pose);

    // Localization based on previous pose
    bool LocalizeImage(const theia::FloatImage& image,
                       const theia::CalibratedAbsolutePose& prev_pose,
                       theia::CalibratedAbsolutePose& pose);

    // Helper function for localization
    bool LocalizeImage(const std::vector<theia::Keypoint>& image_keypoints,
                       const std::vector<Eigen::VectorXf>& image_descriptors,
                       const std::vector<theia::ViewId>& views_to_match,
                       theia::CalibratedAbsolutePose& pose);

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

    // Utility functions
    bool AllEstimated();
    int NumEstimated();
    bool IsEstimated(theia::ViewId view_id);

    // Getters
    theia::ViewId GetLastAddedViewId();
    const theia::Reconstruction& GetReconstruction();
    Options GetOptions();
    std::string GetMessage();

private:
    Options options_;
    std::string reconstruction_message_;

    // Feature extraction and matching
    std::unique_ptr<SiftGpuDescriptorExtractor> descriptor_extractor_;
    std::unique_ptr<ImageRetrieval> image_retrieval_;
    std::unique_ptr<RealtimeFeatureMatcher> feature_matcher_;

    // SfM objects
    std::unique_ptr<theia::ViewGraph> view_graph_;
    std::unique_ptr<theia::Reconstruction> reconstruction_;
    std::unique_ptr<theia::ReconstructionEstimator> reconstruction_estimator_;
};

#endif //REALTIME_RECONSTRUCTION_REALTIMERECONSTRUCTIONBUILDER_H
