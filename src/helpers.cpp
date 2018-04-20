#include "helpers.h"
#include <fstream>

theia::CameraIntrinsicsPrior read_calibration(const std::string &filename) {
    // Read calibration from file into vector
    std::vector<double> calib;
    std::ifstream infile(filename);

    std::string line;
    double number;

    while (std::getline(infile, line)) {
        std::istringstream stream(line);

        while (stream) {
            if (stream >> number) {
                calib.push_back(number);
            }
        }
    }

    // Return intrinsics prior
    theia::CameraIntrinsicsPrior camera_intrinsics_prior;
    camera_intrinsics_prior.image_width = static_cast<int>(calib[0]);
    camera_intrinsics_prior.image_height = static_cast<int>(calib[1]);

    camera_intrinsics_prior.focal_length.is_set = true;
    camera_intrinsics_prior.focal_length.value[0] = calib[2];

    camera_intrinsics_prior.principal_point.is_set = true;
    camera_intrinsics_prior.principal_point.value[0] = calib[3];
    camera_intrinsics_prior.principal_point.value[1] = calib[4];

    camera_intrinsics_prior.aspect_ratio.is_set = true;
    camera_intrinsics_prior.aspect_ratio.value[0] = calib[5];

    camera_intrinsics_prior.skew.is_set = true;
    camera_intrinsics_prior.skew.value[0] = calib[6];

    camera_intrinsics_prior.radial_distortion.is_set = true;
    camera_intrinsics_prior.radial_distortion.value[0] = calib[7];
    camera_intrinsics_prior.radial_distortion.value[1] = calib[8];

    return camera_intrinsics_prior;
}