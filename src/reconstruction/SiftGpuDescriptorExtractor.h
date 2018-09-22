#ifndef REALTIME_RECONSTRUCTION_COLMAPDESCRIPTOREXTRACTOR_H
#define REALTIME_RECONSTRUCTION_COLMAPDESCRIPTOREXTRACTOR_H

#include <vector>

#include <theia/image/image.h>
#include <theia/image/keypoint_detector/keypoint.h>
#include <theia/image/descriptor/descriptor_extractor.h>

#include <SiftGPU/SiftGPU.h>
#include <colmap/feature/sift.h>
#include <colmap/util/cuda.h>
#include <colmap/util/misc.h>

namespace theia {
    class SiftGpuDescriptorExtractor {
    public:
        explicit SiftGpuDescriptorExtractor(const colmap::SiftExtractionOptions &options);

        // Detect keypoints using the Sift keypoint detector and extracts them at the same time.
        bool DetectAndExtractDescriptors(
                const FloatImage &image,
                std::vector<Keypoint> *keypoints,
                std::vector<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>> *descriptors);

    private:
        colmap::SiftExtractionOptions options_;
        SiftGPU sift_gpu_;
    };

}


#endif //REALTIME_RECONSTRUCTION_COLMAPDESCRIPTOREXTRACTOR_H
