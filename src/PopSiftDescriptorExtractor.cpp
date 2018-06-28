#include "PopSiftDescriptorExtractor.h"

#include <popsift/features.h>
#include <popsift/common/device_prop.h>

namespace theia {

    PopSiftDescriptorExtractor::PopSiftDescriptorExtractor(const popsift::Config& options)
            : popsift_(options, popsift::Config::ExtractingMode, PopSift::FloatImages) {};

    PopSiftDescriptorExtractor::~PopSiftDescriptorExtractor() {
        popsift_.uninit();
    }

    bool PopSiftDescriptorExtractor::Initialize() {
        return true;
    }

    bool PopSiftDescriptorExtractor::ComputeDescriptor(const FloatImage &image,
                                                        const Keypoint &keypoint,
                                                        Eigen::VectorXf *descriptor) {
        return false;
    }

    bool PopSiftDescriptorExtractor::ComputeDescriptors(const FloatImage &image,
                                                         std::vector<Keypoint> *keypoints,
                                                         std::vector<Eigen::VectorXf> *descriptors) {
        return false;
    }

    bool PopSiftDescriptorExtractor::DetectAndExtractDescriptors(const FloatImage &img,
                                                                  std::vector<Keypoint> *keypoints,
                                                                  std::vector<Eigen::VectorXf> *descriptors) {
        // Convert image to grayscale
        FloatImage gray_img = img.AsGrayscaleImage();

        SiftJob* job = popsift_.enqueue(gray_img.Width(), gray_img.Height(), gray_img.Data());
        popsift::Features* feature_list = job->get();

        // Convert features to theia
        for (auto feature : *feature_list) {
            Keypoint point;
            point.set_keypoint_type(Keypoint::KeypointType::SIFT);
            point.set_x(feature.xpos);
            point.set_y(feature.ypos);
            point.set_scale(feature.sigma);
            point.set_orientation((feature.orientation[0]));
            keypoints->push_back(point);

            Eigen::VectorXf descriptor = Eigen::Map<Eigen::VectorXf>(feature.desc[0]->features, 128);
            descriptors->push_back(descriptor);
        }

        // Cleanup
        delete feature_list;
        delete job;
        return true;
    }
}