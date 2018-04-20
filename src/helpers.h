#ifndef REALTIME_RECONSTRUCTION_UTILITY_H
#define REALTIME_RECONSTRUCTION_UTILITY_H

#include <string>
#include <theia/sfm/camera_intrinsics_prior.h>

theia::CameraIntrinsicsPrior read_calibration(const std::string &filename);

#endif //REALTIME_RECONSTRUCTION_UTILITY_H
