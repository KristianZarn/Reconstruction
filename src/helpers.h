#ifndef REALTIME_RECONSTRUCTION_UTILITY_H
#define REALTIME_RECONSTRUCTION_UTILITY_H

#include <string>
#include <theia/sfm/camera_intrinsics_prior.h>
#include <theia/sfm/reconstruction_builder.h>
#include <theia/sfm/reconstruction_estimator.h>

theia::ReconstructionBuilderOptions SetReconstructionBuilderOptions();

theia::CameraIntrinsicsPrior read_calibration(const std::string &filename);

void PrintReconstructionSummary(const theia::ReconstructionEstimatorSummary& summary);

#endif //REALTIME_RECONSTRUCTION_UTILITY_H
