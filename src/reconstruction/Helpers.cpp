#include "Helpers.h"

#include <fstream>

#include <theia/util/random.h>

theia::RealtimeReconstructionBuilder::Options SetRealtimeReconstructionBuilderOptions() {
    theia::RealtimeReconstructionBuilder::Options options;

    options.rng = std::make_shared<theia::RandomNumberGenerator>(0);
    options.num_threads = 4;

    // Feature detection options

    // Image retrieval options
    options.image_retrieval_options.vocab_tree_path =
            "/home/kristian/Documents/reconstruction_code/realtime_reconstruction/resources/vocab_tree_flickr100K_words32K.bin";
    options.image_retrieval_options.query_options.max_num_images = 5;

    // Matching options
    options.min_track_length = 2;
    options.max_track_length = 50;

    options.matching_options.rng = options.rng;
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

theia::CameraIntrinsicsPrior ReadCalibration(const std::string &filename) {
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

bool TheiaToMVS(const theia::Reconstruction &reconstruction,
                const std::string &images_path,
                MVS::Scene &mvs_scene) {

    // Map from Theia view_id to MVS vector index
    std::unordered_map<theia::ViewId, int> viewid_to_imageidx;

    // Define platform and camera
    for (const auto& group_id : reconstruction.CameraIntrinsicsGroupIds()) {
        MVS::Platform& platform = mvs_scene.platforms.AddEmpty();
        MVS::Platform::Camera& camera = platform.cameras.AddEmpty();

        // Get camera intrinsics from Theia view
        std::unordered_set<theia::ViewId> view_ids = reconstruction.GetViewsInCameraIntrinsicGroup(group_id);
        const theia::View* tmp_view = reconstruction.View(*(view_ids.begin()));
        Eigen::Matrix3d K;
        tmp_view->Camera().GetCalibrationMatrix(&K);

        // Normalize camera intrinsics
        double scale = 1.0 / std::max(tmp_view->Camera().ImageWidth(), tmp_view->Camera().ImageHeight());
        camera.K = K;
        camera.K(0, 0) *= scale;
        camera.K(1, 1) *= scale;
        camera.K(0, 2) *= scale;
        camera.K(1, 2) *= scale;
        camera.R = Eigen::Matrix3d::Identity();
        camera.C = Eigen::Vector3d::Zero();

        // Define images and poses
        for (const auto& view_id : view_ids) {
            const theia::View* view = reconstruction.View(view_id);

            if (view->IsEstimated()) {
                const std::string image_fullpath = images_path + view->Name();
                auto idx = static_cast<int>(mvs_scene.images.size());
                viewid_to_imageidx[view_id] = idx;

                MVS::Image& image = mvs_scene.images.AddEmpty();
                image.name = image_fullpath;
                image.platformID = group_id;
                image.cameraID = 0;
                image.poseID = static_cast<uint32_t>(platform.poses.size());
                image.width = static_cast<uint32_t>(view->Camera().ImageWidth());
                image.height = static_cast<uint32_t>(view->Camera().ImageHeight());
                image.scale = 1;

                MVS::Platform::Pose& pose = platform.poses.AddEmpty();
                pose.R = view->Camera().GetOrientationAsRotationMatrix();
                pose.C = view->Camera().GetPosition();

                image.UpdateCamera(mvs_scene.platforms);
            }
        }
    }

    // Define structure
    for (const auto& track_id : reconstruction.TrackIds()) {
        const theia::Track* track = reconstruction.Track(track_id);

        if (track->IsEstimated()) {

            // Set position
            MVS::PointCloud::Point& point = mvs_scene.pointcloud.points.AddEmpty();
            point = track->Point().hnormalized().cast<float>();

            // Set views that see the point
            MVS::PointCloud::ViewArr& views = mvs_scene.pointcloud.pointViews.AddEmpty();

            std::vector<uint32_t> tmp;
            for (const auto& view_id : track->ViewIds()) {
                auto image_id = static_cast<uint32_t>(viewid_to_imageidx[view_id]);
                tmp.push_back(image_id);
            }
            std::sort(tmp.begin(), tmp.end());

            for (const auto& view_id : tmp) {
                MVS::PointCloud::View& view = views.AddEmpty();
                view = view_id;
            }

            // Set point color
            const Eigen::Matrix<uint8_t, 3, 1>& track_color = track->Color();
            MVS::PointCloud::Color& color = mvs_scene.pointcloud.colors.AddEmpty();
            color.r = track_color(0);
            color.g = track_color(1);
            color.b = track_color(2);
        }
    }
    return true;
}
