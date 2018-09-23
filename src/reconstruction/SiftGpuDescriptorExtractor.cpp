#include "SiftGpuDescriptorExtractor.h"

#include <colmap/util/bitmap.h>
#include <colmap/util/timer.h>
#include <colmap/util/opengl_utils.h>
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
            std::vector<Eigen::VectorXf> *descriptors) {

        // Convert image to appropriate format
        FloatImage gray_img = img.AsGrayscaleImage();
        gray_img.ScalePixels(255.0f);

        // Extract features
        const int result = sift_gpu_.RunSIFT(gray_img.Width(), gray_img.Height(),
                                             gray_img.Data(), GL_LUMINANCE, GL_FLOAT);

        // Download the extracted keypoints and descriptors
        const auto num_features = static_cast<size_t>(sift_gpu_.GetFeatureNum());

        std::vector<SiftKeypoint> keypoints_siftgpu(num_features);
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
                descriptors_siftgpu(num_features, 128);

        sift_gpu_.GetFeatureVector(keypoints_siftgpu.data(), descriptors_siftgpu.data());

        // Normalize the descriptors
        if (options_.normalization == colmap::SiftExtractionOptions::Normalization::L2) {
            descriptors_siftgpu = colmap::L2NormalizeFeatureDescriptors(descriptors_siftgpu);
        } else if (options_.normalization == colmap::SiftExtractionOptions::Normalization::L1_ROOT) {
            descriptors_siftgpu = colmap::L1RootNormalizeFeatureDescriptors(descriptors_siftgpu);
        }

        // Convert to Theia format
        for (int i = 0; i < num_features; i++) {
            Keypoint keypoint(keypoints_siftgpu[i].x,
                              keypoints_siftgpu[i].y,
                              Keypoint::KeypointType::SIFT);
            keypoint.set_scale(keypoints_siftgpu[i].s);
            keypoint.set_orientation(keypoints_siftgpu[i].o);

            keypoints->push_back(keypoint);
            descriptors->emplace_back(descriptors_siftgpu.row(i));
        }

        return (result == 1);
    }
}