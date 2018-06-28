#include <cudaSift.h>
#include "CudaSiftDescriptorExtractor.h"

namespace theia {

    CudaSiftDescriptorExtractor::~CudaSiftDescriptorExtractor() {
        FreeSiftData(sift_data_);
    }

    bool CudaSiftDescriptorExtractor::Initialize() {
        InitCuda(0);
        InitSiftData(sift_data_, 65536, true, true);
        return true;
    }

    bool CudaSiftDescriptorExtractor::ComputeDescriptor(const FloatImage &image,
                                                        const Keypoint &keypoint,
                                                        Eigen::VectorXf *descriptor) {
        return false;
    }

    bool CudaSiftDescriptorExtractor::ComputeDescriptors(const FloatImage &image,
                                                         std::vector<Keypoint> *keypoints,
                                                         std::vector<Eigen::VectorXf> *descriptors) {
        return false;
    }

    bool CudaSiftDescriptorExtractor::DetectAndExtractDescriptors(const FloatImage &img,
                                                             std::vector<Keypoint> *keypoints,
                                                             std::vector<Eigen::VectorXf> *descriptors) {
        // Convert image to grayscale
        FloatImage gray_img = img.AsGrayscaleImage();
        gray_img.ScalePixels(255.0f);

        // Copy image to device
        CudaImage cuda_img;
        cuda_img.Allocate(gray_img.Width(),
                          gray_img.Height(),
                          iAlignUp(gray_img.Width(), 128),
                          false, NULL,
                          gray_img.Data());
        cuda_img.Download();

        // Extract sift
        ExtractSift(sift_data_,
                    cuda_img,
                    options_.num_octaves,
                    options_.init_blur,
                    options_.thresh,
                    options_.min_scale,
                    options_.up_scale);

        // Convert to theia keypoints and descriptors
        for (int i = 0; i < sift_data_.numPts; i++) {
            SiftPoint cuda_point = sift_data_.h_data[i];

            Keypoint point;
            point.set_keypoint_type(Keypoint::KeypointType::SIFT);
            point.set_x(cuda_point.xpos);
            point.set_y(cuda_point.ypos);
            point.set_strength(cuda_point.score);
            point.set_scale(cuda_point.scale);
            point.set_orientation(cuda_point.orientation);
            keypoints->push_back(point);

            Eigen::VectorXf descriptor = Eigen::Map<Eigen::VectorXf>(cuda_point.data, 128);
            descriptors->push_back(descriptor);
        }
        return true;
    }
}