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

namespace theia {

    class RealtimeReconstructionBuilder {
    public:
        explicit RealtimeReconstructionBuilder(const ReconstructionBuilderOptions& options,
                                               const CameraIntrinsicsPrior& intrinsics_prior);

        // Builds initial reconstruction from two images
        ReconstructionEstimatorSummary InitializeReconstruction(const std::string& image1_fullpath,
                                                                const std::string& image2_fullpath);

        // Adds new image to the reconstruction
        ReconstructionEstimatorSummary ExtendReconstruction(const std::string& image_fullpath);

        // Check if reconstruction is initialized
        bool IsInitialized();

        // Return points (eigen matrix #V x 3)
        Eigen::MatrixXd GetReconstructedPoints();

        // Return point colors (eigen matrix #V x 3)
        Eigen::MatrixXd GetPointColors();

        // Return camera positions (eigen matrix #C x 3)
        Eigen::MatrixXd GetCameraPositions();

        Reconstruction* GetReconstruction();

        void ColorizeReconstruction(const std::string& images_path);

        void WritePly(const std::string& output_fullpath);

        void RemoveView(ViewId view_id);

        void ResetReconstruction();

        // Print statistics
        void PrintStatistics(std::ostream& stream, bool print_images = true, bool print_reconstruction = true,
                             bool print_view_graph = true, bool print_feature_track_map = true);

    private:
        ReconstructionBuilderOptions options_;
        CameraIntrinsicsPrior intrinsics_prior_;

        // Images
        std::vector<std::string> image_filenames_;

        // Feature extraction and matching
        std::unique_ptr<DescriptorExtractor> descriptor_extractor_;
        std::unique_ptr<FeatureMatcher> feature_matcher_;

        std::unordered_map<std::pair<ViewId, Feature>, TrackId> image_feature_to_track_id_;

        // SfM objects
        std::unique_ptr<ViewGraph> view_graph_;
        std::unique_ptr<Reconstruction> reconstruction_;
        std::unique_ptr<ReconstructionEstimator> reconstruction_estimator_;
    };

} // namespace theia

#endif //REALTIME_RECONSTRUCTION_REALTIMERECONSTRUCTIONBUILDER_H
