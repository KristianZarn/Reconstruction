#include "PopSiftDescriptorExtractor.h"

#include <popsift/features.h>
#include <popsift/common/device_prop.h>

#include <colmap/util/math.h>

namespace theia {

    PopSiftDescriptorExtractor::PopSiftDescriptorExtractor(const popsift::Config &options)
            : popsift_(options, popsift::Config::ExtractingMode, PopSift::FloatImages) {};

    PopSiftDescriptorExtractor::~PopSiftDescriptorExtractor() {
        popsift_.uninit();
    }

    bool PopSiftDescriptorExtractor::DetectAndExtractDescriptors(
            const FloatImage &img,
            std::vector<Keypoint> *keypoints,
            std::vector<Eigen::Matrix<uint8_t, Eigen::Dynamic, 1>> *descriptors) {

        // Convert image to grayscale
        FloatImage gray_img = img.AsGrayscaleImage();

        SiftJob *job = popsift_.enqueue(gray_img.Width(), gray_img.Height(), gray_img.Data());
        popsift::Features *feature_list = job->get();
        int num_features = feature_list->size();

        // Convert features to theia
        Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
                descriptors_data(num_features, 128);
        int i = 0;
        for (auto feature : *feature_list) {
            Keypoint point;
            point.set_keypoint_type(Keypoint::KeypointType::SIFT);
            point.set_x(feature.xpos);
            point.set_y(feature.ypos);
            point.set_scale(feature.sigma);
            point.set_orientation((feature.orientation[0]));
            keypoints->push_back(point);

            descriptors_data.row(i) = Eigen::Map<Eigen::VectorXf>(feature.desc[0]->features, 128);
            i++;
        }

        // Convert from float to unsigned byte
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
                descriptors_byte(num_features, 128);
        descriptors_byte = FeatureDescriptorsToUnsignedByte(descriptors_data);

        // Convert to theia descriptors
        for (int i = 0; i < num_features; i++) {
            descriptors->emplace_back(descriptors_byte.row(i));
        }

        // Cleanup
        delete feature_list;
        delete job;
        return true;
    }

    Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
    PopSiftDescriptorExtractor::FeatureDescriptorsToUnsignedByte(
            const Eigen::MatrixXf &descriptors) {
        Eigen::Matrix<uint8_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> descriptors_unsigned_byte(
                descriptors.rows(),
                descriptors.cols());
        for (Eigen::MatrixXf::Index r = 0; r < descriptors.rows(); ++r) {
            for (Eigen::MatrixXf::Index c = 0; c < descriptors.cols(); ++c) {
                const float scaled_value = std::round(512.0f * descriptors(r, c));
                descriptors_unsigned_byte(r, c) = colmap::TruncateCast<float, uint8_t>(scaled_value);
            }
        }
        return descriptors_unsigned_byte;
    }
}