#include "SiftGpuDescriptorExtractor.h"

#include <mutex>
#include <GL/gl.h>

#include <colmap/util/timer.h>
#include <colmap/util/misc.h>
#include <colmap/feature/utils.h>

namespace theia {

    SiftGpuDescriptorExtractor::SiftGpuDescriptorExtractor(const Options &options)
            : options_(options) {

        bool success = CreateSiftGPUExtractor(options_, &sift_gpu_);
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
        if (options_.normalization == Options::Normalization::L2) {
            descriptors_siftgpu = colmap::L2NormalizeFeatureDescriptors(descriptors_siftgpu);
        } else if (options_.normalization == Options::Normalization::L1_ROOT) {
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

    bool SiftGpuDescriptorExtractor::CreateSiftGPUExtractor(const Options& options, SiftGPU* sift_gpu) {
        CHECK_NOTNULL(sift_gpu);

        // SiftGPU uses many global static state variables and the initialization must
        // be thread-safe in order to work correctly. This is enforced here.
        static std::mutex mutex;
        std::unique_lock<std::mutex> lock(mutex);

        std::vector<int> gpu_indices = colmap::CSVToVector<int>(options.gpu_index);
        CHECK_EQ(gpu_indices.size(), 1) << "SiftGPU can only run on one GPU";

        std::vector<std::string> sift_gpu_args;

        sift_gpu_args.push_back("./sift_gpu");

        // Use CUDA version by default if darkness adaptivity is disabled.
        gpu_indices[0] = 0;
        sift_gpu_args.push_back("-cuda");
        sift_gpu_args.push_back(std::to_string(gpu_indices[0]));

        // No verbose logging.
        sift_gpu_args.push_back("-v");
        sift_gpu_args.push_back("0");

        // Fixed maximum image dimension.
        sift_gpu_args.push_back("-maxd");
        sift_gpu_args.push_back(std::to_string(options.max_image_size));

        // Keep the highest level features.
        sift_gpu_args.push_back("-tc2");
        sift_gpu_args.push_back(std::to_string(options.max_num_features));

        // First octave level.
        sift_gpu_args.push_back("-fo");
        sift_gpu_args.push_back(std::to_string(options.first_octave));

        // Number of octave levels.
        sift_gpu_args.push_back("-d");
        sift_gpu_args.push_back(std::to_string(options.octave_resolution));

        // Peak threshold.
        sift_gpu_args.push_back("-t");
        sift_gpu_args.push_back(std::to_string(options.peak_threshold));

        // Edge threshold.
        sift_gpu_args.push_back("-e");
        sift_gpu_args.push_back(std::to_string(options.edge_threshold));

        if (options.upright) {
            // Fix the orientation to 0 for upright features.
            sift_gpu_args.push_back("-ofix");
            // Maximum number of orientations.
            sift_gpu_args.push_back("-mo");
            sift_gpu_args.push_back("1");
        } else {
            // Maximum number of orientations.
            sift_gpu_args.push_back("-mo");
            sift_gpu_args.push_back(std::to_string(options.max_num_orientations));
        }

        std::vector<const char*> sift_gpu_args_cstr;
        sift_gpu_args_cstr.reserve(sift_gpu_args.size());
        for (const auto& arg : sift_gpu_args) {
            sift_gpu_args_cstr.push_back(arg.c_str());
        }

        sift_gpu->ParseParam(sift_gpu_args_cstr.size(), sift_gpu_args_cstr.data());
        sift_gpu->gpu_index = gpu_indices[0];

        return sift_gpu->VerifyContextGL() == SiftGPU::SIFTGPU_FULL_SUPPORTED;
    }
}