#ifndef RECONSTRUCTION_THEIA_CUDASIFTDESCRIPTOREXTRACTOR_H
#define RECONSTRUCTION_THEIA_CUDASIFTDESCRIPTOREXTRACTOR_H

#include <vector>
#include <Eigen/Core>
#include <theia/image/image.h>
#include <theia/image/keypoint_detector/keypoint.h>
#include <theia/image/descriptor/descriptor_extractor.h>

#include "cudaImage.h"
#include "cudaSift.h"

namespace theia {
    class CudaSiftDescriptorExtractor : public DescriptorExtractor {
    public:
        struct Options {
            // Number of octaves in Gaussian pyramid
            int num_octaves = 5;
            // Amount of initial Gaussian blurring in standard deviations
            float init_blur = 1.6f;
            // Threshold on difference of Gaussians for feature pruning
            float thresh = 1.0f;
            // Minimum acceptable scale to remove fine-scale features
            float min_scale = 0.2f;
            // Whether to upscale image before extraction
            bool up_scale = false;
        };

        explicit CudaSiftDescriptorExtractor(const Options& options) : options_(options) {};
        ~CudaSiftDescriptorExtractor() override;

        // This method should be called before using any of the descriptor extractors.
        bool Initialize() override;

        // Computes a descriptor at a single keypoint.
        bool ComputeDescriptor(const FloatImage& image,
                               const Keypoint& keypoint,
                               Eigen::VectorXf* descriptor) override;

        // Compute multiple descriptors for keypoints from a single image.
        bool ComputeDescriptors(const FloatImage& image,
                                std::vector<Keypoint>* keypoints,
                                std::vector<Eigen::VectorXf>* descriptors) override;

        // Detect keypoints using the Sift keypoint detector and extracts them at the same time.
        bool DetectAndExtractDescriptors(
                const FloatImage& image,
                std::vector<Keypoint>* keypoints,
                std::vector<Eigen::VectorXf>* descriptors) override;

    private:
        Options options_;
        SiftData sift_data_;
    };
}

#endif //RECONSTRUCTION_THEIA_CUDASIFTDESCRIPTOREXTRACTOR_H
