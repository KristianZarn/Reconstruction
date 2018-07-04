#ifndef RECONSTRUCTION_THEIA_POPSIFTDESCRIPTOREXTRACTOR_H
#define RECONSTRUCTION_THEIA_POPSIFTDESCRIPTOREXTRACTOR_H

#include <vector>
#include <Eigen/Core>
#include <theia/image/image.h>
#include <theia/image/keypoint_detector/keypoint.h>
#include <theia/image/descriptor/descriptor_extractor.h>
#include <popsift/sift_conf.h>
#include <popsift/popsift.h>

namespace theia {
    class PopSiftDescriptorExtractor : public DescriptorExtractor {
    public:
        explicit PopSiftDescriptorExtractor(const popsift::Config& options);
        ~PopSiftDescriptorExtractor() override;

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
        PopSift popsift_;
    };
}

#endif //RECONSTRUCTION_THEIA_POPSIFTDESCRIPTOREXTRACTOR_H
