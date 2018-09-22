#include "SiftGpuDescriptorExtractor.h"

#include <colmap/util/bitmap.h>
#include <colmap/util/timer.h>
#include <colmap/feature/utils.h>

namespace theia {

    SiftGpuDescriptorExtractor::SiftGpuDescriptorExtractor(const colmap::SiftExtractionOptions &options)
            : options_(options) {

        bool success = colmap::CreateSiftGPUExtractor(options_, &sift_gpu_);
        assert(success && "ERROR: SiftGPU not fully supported.");
    }

    bool SiftGpuDescriptorExtractor::DetectAndExtractDescriptors(
            const FloatImage &img,
            std::vector<Keypoint> *keypoints,
            std::vector<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>> *descriptors) {

        // Convert image to appropriate format
        FloatImage gray_img = img.AsGrayscaleImage();

        colmap::Bitmap bitmap;
        bitmap.Allocate(gray_img.Width(), gray_img.Height(), false);
        for (int x = 0; x < gray_img.Width(); x++) {
            for (int y = 0; y < gray_img.Height(); y++) {
                float intensity_float = gray_img.GetXY(x, y, 0);
                auto intensity_uint8 = static_cast<uint8_t>(intensity_float * 255);
                bitmap.SetPixel(x, y, colmap::BitmapColor<uint8_t>(intensity_uint8));
            }
        }

        // Extract features
        colmap::FeatureKeypoints colmap_keypoints;
        colmap::FeatureDescriptors colmap_descriptors;
        bool success = colmap::ExtractSiftFeaturesGPU(
                options_, bitmap, &sift_gpu_, &colmap_keypoints, &colmap_descriptors);

        // colmap::ExtractTopScaleFeatures(&colmap_keypoints, &colmap_descriptors, options_.max_num_features);

        // Convert to Theia format
        auto num_features = colmap_keypoints.size();
        for (int i = 0; i < num_features; i++) {
            Keypoint keypoint(colmap_keypoints[i].x,
                              colmap_keypoints[i].y,
                              Keypoint::KeypointType::SIFT);

            keypoint.set_scale(colmap_keypoints[i].ComputeScale());
            keypoint.set_orientation(colmap_keypoints[i].ComputeOrientation());

            keypoints->push_back(keypoint);
            descriptors->emplace_back(colmap_descriptors.row(i));
        }

        return success;
    }
}