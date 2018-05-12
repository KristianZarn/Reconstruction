#include "helpers.h"

#include <fstream>

#include <theia/util/random.h>

theia::RealtimeReconstructionBuilder::Options SetRealtimeReconstructionBuilderOptions() {
    theia::RealtimeReconstructionBuilder::Options options;

    options.rng = std::make_shared<theia::RandomNumberGenerator>(0);
    options.num_threads = 4;

    // Feature detection options
    options.descriptor_type = theia::DescriptorExtractorType::SIFT;
    options.feature_density = theia::FeatureDensity::NORMAL;

    // Matching options
    options.min_track_length = 2;
    options.max_track_length = 50;

    options.matching_options.num_threads = 4;
    options.matching_options.min_num_feature_matches = 30;
    options.matching_options.lowes_ratio = 0.8;
    options.matching_options.perform_geometric_verification = true;

    options.matching_options.geometric_verification_options.estimate_twoview_info_options.rng = options.rng;
    options.matching_options.geometric_verification_options.estimate_twoview_info_options.max_sampson_error_pixels = 4.0;
    options.matching_options.geometric_verification_options.bundle_adjustment = true;
    options.matching_options.geometric_verification_options.triangulation_max_reprojection_error = 15.0;
    options.matching_options.geometric_verification_options.min_triangulation_angle_degrees = 4.0;
    options.matching_options.geometric_verification_options.final_max_reprojection_error = 4.0;

    // Reconstruction Estimator Options.
    theia::ReconstructionEstimatorOptions& reconstruction_estimator_options = options.reconstruction_estimator_options;

    reconstruction_estimator_options.rng = options.rng;
    reconstruction_estimator_options.num_threads = 4;
    reconstruction_estimator_options.min_num_two_view_inliers = 30;
    reconstruction_estimator_options.intrinsics_to_optimize = theia::OptimizeIntrinsicsType::FOCAL_LENGTH;
    reconstruction_estimator_options.max_reprojection_error_in_pixels = 4.0;

    // Which type of SfM pipeline to use (e.g., incremental, global, etc.);
    reconstruction_estimator_options.reconstruction_estimator_type = theia::ReconstructionEstimatorType::INCREMENTAL;

    // Incremental SfM Options.
    reconstruction_estimator_options.absolute_pose_reprojection_error_threshold = 4.0;
    reconstruction_estimator_options.min_num_absolute_pose_inliers = 30;
    reconstruction_estimator_options.full_bundle_adjustment_growth_percent = 5.0;
    reconstruction_estimator_options.partial_bundle_adjustment_num_views = 20;

    // Triangulation options (used by all SfM pipelines).
    reconstruction_estimator_options.min_triangulation_angle_degrees = 4.0;
    reconstruction_estimator_options.triangulation_max_reprojection_error_in_pixels = 15.0;
    reconstruction_estimator_options.bundle_adjust_tracks = true;

    // Bundle adjustment options (used by all SfM pipelines).
    reconstruction_estimator_options.bundle_adjustment_loss_function_type = theia::LossFunctionType::TRIVIAL;
    reconstruction_estimator_options.bundle_adjustment_robust_loss_width = 10.0;

    // Track subsampling options.
    reconstruction_estimator_options.subsample_tracks_for_bundle_adjustment = false;
    reconstruction_estimator_options.track_subset_selection_long_track_length_threshold = 10;
    reconstruction_estimator_options.track_selection_image_grid_cell_size_pixels = 100;
    reconstruction_estimator_options.min_num_optimized_tracks_per_view = 100;

    return options;
}

theia::CameraIntrinsicsPrior read_calibration(const std::string &filename) {
    // Read calibration from file into vector
    std::vector<double> calib;
    std::ifstream infile(filename);

    std::string line;
    double number;

    while (std::getline(infile, line)) {
        std::istringstream stream(line);

        while (stream) {
            if (stream >> number) {
                calib.push_back(number);
            }
        }
    }

    theia::CameraIntrinsicsPrior camera_intrinsics_prior;
    if (calib.size() >= 2) {
        camera_intrinsics_prior.image_width = static_cast<int>(calib[0]);
        camera_intrinsics_prior.image_height = static_cast<int>(calib[1]);
    }

    if (calib.size() >= 3) {
        camera_intrinsics_prior.focal_length.is_set = true;
        camera_intrinsics_prior.focal_length.value[0] = calib[2];
    }

    if (calib.size() >= 5) {
        camera_intrinsics_prior.principal_point.is_set = true;
        camera_intrinsics_prior.principal_point.value[0] = calib[3];
        camera_intrinsics_prior.principal_point.value[1] = calib[4];
    }

    if (calib.size() >= 6) {
        camera_intrinsics_prior.aspect_ratio.is_set = true;
        camera_intrinsics_prior.aspect_ratio.value[0] = calib[5];
    }

    if (calib.size() >= 7) {
        camera_intrinsics_prior.skew.is_set = true;
        camera_intrinsics_prior.skew.value[0] = calib[6];
    }

    if (calib.size() >= 9) {
        camera_intrinsics_prior.radial_distortion.is_set = true;
        camera_intrinsics_prior.radial_distortion.value[0] = calib[7];
        camera_intrinsics_prior.radial_distortion.value[1] = calib[8];
    }

    return camera_intrinsics_prior;
}

void PrintReconstructionSummary(const theia::ReconstructionEstimatorSummary& summary) {
    if (summary.success) {
        std::cout << "Summary: reconstruction successful " << std::endl;
        std::cout << "\n\tNum estimated views = " << summary.estimated_views.size()
                  << "\n\tNum estimated tracks = " << summary.estimated_tracks.size()
                  << "\n\tPose estimation time = " << summary.pose_estimation_time
                  << "\n\tTriangulation time = " << summary.triangulation_time
                  << "\n\tBundle Adjustment time = " << summary.bundle_adjustment_time
                  << "\n\tTotal time = " << summary.total_time
                  << "\n\tMessage = " << summary.message << "\n\n";
    } else {
        std::cout << "Summary: reconstruction failed: \n";
        std::cout << "\n\tMessage = " << summary.message << "\n\n";
    }
}