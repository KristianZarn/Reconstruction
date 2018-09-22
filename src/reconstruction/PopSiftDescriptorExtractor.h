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
    class PopSiftDescriptorExtractor {
    public:
        explicit PopSiftDescriptorExtractor(const popsift::Config& options);
        ~PopSiftDescriptorExtractor();

        // Detect keypoints using the Sift keypoint detector and extracts them at the same time.
        bool DetectAndExtractDescriptors(
                const FloatImage& image,
                std::vector<Keypoint>* keypoints,
                std::vector<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>>* descriptors);

    private:
        PopSift popsift_;

        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
        FeatureDescriptorsToUnsignedByte(const Eigen::MatrixXf &descriptors);
    };
}

#endif //RECONSTRUCTION_THEIA_POPSIFTDESCRIPTOREXTRACTOR_H
