#ifndef REALTIME_RECONSTRUCTION_UTILITY_H
#define REALTIME_RECONSTRUCTION_UTILITY_H

#include <string>

#include <OpenMVS/MVS.h>
#include <theia/sfm/camera_intrinsics_prior.h>
#include <theia/sfm/reconstruction_builder.h>
#include <theia/sfm/reconstruction_estimator.h>

#include "RealtimeReconstructionBuilder.h"

theia::RealtimeReconstructionBuilder::Options SetRealtimeReconstructionBuilderOptions();

theia::CameraIntrinsicsPrior ReadCalibration(const std::string &filename);

void PrintReconstructionSummary(const theia::ReconstructionEstimatorSummary& summary);

bool theia_to_mvs(const theia::Reconstruction& reconstruction,
                  const std::string& images_path,
                  MVS::Scene& mvs_scene);

#endif //REALTIME_RECONSTRUCTION_UTILITY_H
